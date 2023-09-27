# Comau post processor, Version 1.00

from vcCommand import *
import os.path, time, re, math
import vcMatrix

class TranslatorHelper:
  #---
  # Capsulates commonly used handles and post processor settings
  #---
  def __init__(self, app, program, uri):
    
    # Settings
    self.create_variable_file = True        # Separate variables into their own file (.lvs)
    
    # String cache
    self.cache = ''                         # Generic temporary cache
    self.cache_vars = ''                    # Program global variables
    self.cache_program = ''                 # Program contents
    self.cache_routine_vars = ''            # Routine local variables
    self.cache_routine = ''                 # Routine contents
    self.cache_routines = ''                # All routines
    self.cache_lsv = ''                     # .lvs file (variable file) contents
    
    # Helpers
    self.app = app
    self.uri = uri
    self.program = program
    self.current_routine = None
    self.local_vars = []
    self.global_vars = []
    self.pos_names = []
    self.indentation = '  '
    self.depth = 0
    self.statement_translators = {
      VC_STATEMENT_BREAK:write_break,
      VC_STATEMENT_CALL:write_call,
      VC_STATEMENT_COMMENT:write_comment,
      VC_STATEMENT_CONTINUE:write_continue,
      VC_STATEMENT_DEFINE_BASE:write_define_base,
      VC_STATEMENT_DEFINE_TOOL:write_define_tool,
      VC_STATEMENT_DELAY:write_delay,
      VC_STATEMENT_HALT:write_halt,
      VC_STATEMENT_IF:write_if,
      VC_STATEMENT_LINMOTION:write_lin_motion,
      VC_STATEMENT_PATH:write_path,
      VC_STATEMENT_PRINT:write_print,
      VC_STATEMENT_PROCESS:unknown,
      VC_STATEMENT_PTPMOTION:write_ptp_motion,
      VC_STATEMENT_RETURN:write_return,
      VC_STATEMENT_SETBIN:write_set_bin,
      VC_STATEMENT_SETPROPERTY:write_set_property,
      VC_STATEMENT_WAITBIN:write_wait_bin,
      VC_STATEMENT_WHILE:write_while }
    
    # Robot
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.robot_joint_count = len([x for x in self.controller.Joints if not x.ExternalController])
    self.total_joint_count = len(self.controller.Joints)
    
    # Motion
    self.motiontarget = program.Executor.Controller.createTarget()          #Target with new motion data
    self.motiontarget_cache = program.Executor.Controller.createTarget()    #Target with cached motion data
    self.motion_data_set = False
    self.use_blending = False
  
  def get_prg_name(self):
    head, tail = os.path.split(self.uri)
    return tail[:-4]
  
  def current_indent(self):
    return self.indentation*self.depth
  
  def write_cache(self, line):
    self.cache += self.current_indent() + line + '\n'
  
  def write_var(self, line, override_depth = -1):
    if override_depth >= 0:
      self.cache_vars += override_depth * self.indentation + line + '\n'
    else:
      self.cache_vars += self.current_indent() + line + '\n'
  
  def write_prg(self, line):
    self.cache_program += self.current_indent() + line + '\n'
  
  def write_rou_var(self, line, override_depth = -1):
    if override_depth >= 0:
      self.cache_routine_vars += override_depth * self.indentation + line + '\n'
    else:
      self.cache_routine_vars += self.current_indent() + line + '\n'
  
  def write_rou(self, line):
    self.cache_routine += self.current_indent() + line + '\n'
  
  def write_lsv(self, line):
    self.cache_lsv += line + '\n'
  
  def get_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  
  def get_lsv_filename(self):
    head, tail = os.path.split(self.uri)
    base_name = tail[:tail.rfind('.')]
    return base_name + '.lsv'
  
  def get_lsv_uri(self):
    lsv_uri = '%s\\%s' % (self.get_folder(), self.get_lsv_filename())
    return lsv_uri


class RE(object):
  #---
  # Capsulates RE constants
  #---
  ANY = r'.+'
  SPC = r'\s'
  SPCQ = r'\s*'
  EOL = r'\n'
  INT = r'\s*-?\d+'
  NUM = r'\s*-?\d+\.?\d*[eE]?-?\d*'
  NAME = r'\w+'
  FLAGS = re.I|re.S


class TranslatorException(Exception):
  #---
  # Custom exception type
  #---
  pass


def postProcess(app, program, uri):
  # Entry point
  
  # Create helper object
  helper = TranslatorHelper(app, program, uri)
  
  # Write globals and program
  write_globals(helper)
  write_program(helper)
  
  # Write main and subroutines in reverse order
  try:
    routines = get_routines_in_reverse_order(helper)
    for routine in routines:
      write_routine(routine, routine.Name, helper)
  except TranslatorException as e:
    print 'ERROR: ', e
    return False,[]
  
  #Empty line after global vars
  helper.write_var('')
  
  # Compile file contents
  prog_name = helper.get_prg_name()
  helper.cache = ''
  helper.write_cache('PROGRAM %s' % (prog_name))
  helper.cache += helper.cache_vars
  helper.cache += helper.cache_routines
  helper.cache += helper.cache_program
  helper.write_cache('END %s' % (prog_name))
  
  # Write output file
  files = []
  with open(helper.uri, 'w') as output_file:
    output_file.write(helper.cache)
  files.append(uri)
  
  # Write variable file
  if helper.create_variable_file:
    lsv_uri = helper.get_lsv_uri()
    with open(lsv_uri, 'w') as output_file:
      output_file.write(helper.cache_lsv)
    files.append(lsv_uri)
  
  return True,files


def get_routines_in_reverse_order(helper):
  # Gather all routines in a list and reverse its order (low level sub-routines first, main last)
  
  # First gether rous in call order
  new_rous = [helper.program.MainRoutine]
  rous = []
  while new_rous:
    rou = new_rous.pop(0)
    rous.append(rou)
    stats = get_all_statements(helper, rou)
    for stat in stats:
      if stat.Type != VC_STATEMENT_CALL:
        continue
      if stat.Routine and not (stat.Routine in rous or stat.Routine in new_rous):
        new_rous.append(stat.Routine)
  
  #Add uncalled rous at the back
  for rou in helper.program.Routines:
    if not rou in rous:
      rous.append(rou)
  
  #Reverse order and return
  rous.reverse()
  return rous


def write_globals(helper):
  # Write global variables
  
  #Header
  helper.write_var('VAR', 0)
  
  #Other used globals (component properties) are written by write_set_property
  pass


def write_program(helper):
  # Write program (write frames and call main)
  
  helper.write_prg('BEGIN')
  helper.depth  += 1
  write_frames(helper)
  helper.write_prg('Main')
  helper.depth  -= 1


def write_routine(routine, name, helper):
  # Write a routine
  
  helper.current_routine = routine
  helper.motion_data_set = False
  helper.use_blending = False
  helper.cache_routine = ''
  helper.cache_routine_vars = ''
  
  # Check if motion blending needs to be used in this routine
  stats = get_all_statements(helper, helper.current_routine)
  for stat in stats:
    if stat.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      if stat.AccuracyValue > 0:
        helper.use_blending = True
        break
    elif stat.Type == VC_STATEMENT_PATH:
      acc_prop_found = False
      for p in stat.SchemaProperties:
        if p.Name == 'AccuracyValue':
          acc_prop_found = True
          break
      if acc_prop_found:
        for pos in stat.Positions:
          if pos.AccuracyValue > 0:
            helper.use_blending = True
            break
      if helper.use_blending:
        break
  #endfor
  
  # Routine vars
  helper.write_rou_var('VAR')
  helper.depth  += 1
  helper.local_vars = []
  skip_list = ['Name']
  types = {VC_BOOLEAN : 'BOOLEAN', VC_INTEGER : 'INTEGER', VC_REAL : 'REAL', VC_STRING : 'STRING'}
  for p in routine.Properties:
    if p.Name in skip_list or not p.Type in types.keys():
      continue
    helper.write_rou_var('%s : %s' % (p.Name, types[p.Type]))
    helper.local_vars.append(p.Name)
    #
    #if helper.create_variable_file:
    #  val = str(p.Value)
    #  if p.Type == VC_BOOLEAN:
    #    val = val.upper()
    #  helper.write_lsv('%s %s Priv %s' % (p.Name, types[p.Type][0:3], val))
    #  helper.write_lsv('')
  helper.depth  -= 1
  
  # Routine statements
  helper.write_rou('BEGIN')
  helper.depth  += 1
  for statement in routine.Statements:
    translator = helper.statement_translators.get(statement.Type, unknown)
    translator(statement, helper)
  helper.depth  -= 1
  
  # Routine structure
  helper.cache = ''
  helper.write_cache('ROUTINE %s' % (name))
  helper.cache += helper.cache_routine_vars
  helper.cache += helper.cache_routine
  helper.write_cache('END %s' % (name))
  helper.write_cache('')
  helper.cache_routines += helper.cache


def write_frames(helper):
  bases = []
  tools = []
  all_stats = get_all_statements(helper)
  use_null = False
  for s in all_stats:
    if s.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      for b in bases:
        if b == s.Base:
          break
      else:
        bases.append(s.Base)
        if not s.Base:
          use_null = True
      for t in tools:
        if t == s.Tool:
          break
      else:
        tools.append(s.Tool)
        if not s.Tool:
          use_null = True
  
  #Write null frame
  if use_null:
    pos = write_matrix(vcMatrix.new())
    helper.write_var('%s : POSITION' % ('NULL'))
    helper.write_prg('%s := %s' % ('NULL', pos))
    if helper.create_variable_file:
      lvs_pos = 'NULL POS Priv '
      lvs_pos += write_matrix_lvs(vcMatrix.new())
      helper.write_lsv(lvs_pos)
      helper.write_lsv('')
  for b in bases:
    if not b:
      continue
    
    m = get_base_in_reference(b, helper.motiontarget)
    pos = write_matrix(m)
    helper.write_var('%s : POSITION' % (b.Name))
    helper.write_prg('%s := %s' % (b.Name, pos))
    if helper.create_variable_file:
      lvs_pos = '%s POS Priv ' % (b.Name)
      lvs_pos += write_matrix_lvs(m)
      helper.write_lsv(lvs_pos)
      helper.write_lsv('')
    
    #Check for external tcp configuration
    node = b.Node
    flange_node = helper.controller.FlangeNode
    while node:
      if node == flange_node:
        raise TranslatorException('External TCP not supported.')
      node = node.Parent
    
  for t in tools:
    if not t:
      continue
    
    m = get_tool_in_reference(t, helper.motiontarget, helper.controller)
    pos = write_matrix(m)
    helper.write_var('%s : POSITION' % (t.Name))
    helper.write_prg('%s := %s' % (t.Name, pos))
    
    if helper.create_variable_file:
      lvs_pos = '%s POS Priv ' % (t.Name)
      lvs_pos += write_matrix_lvs(m)
      helper.write_lsv(lvs_pos)
      helper.write_lsv('')
    
  helper.write_prg('')


def write_break(statement, helper):
  helper.write_rou('BREAK')


def write_call(statement, helper):
  rou = statement.getProperty('Routine').Value
  if rou:
    helper.write_rou('%s' % rou.Name)


def write_comment(statement, helper):
  helper.write_rou('--%s' % statement.Comment)


def write_define_base(statement, helper):
  if not statement.Base:
    return
  
  if not statement.IsRelative:
    base_m_in_ref = statement.Position
    if statement.Base.Node:
      ref_m = helper.motiontarget.getSimWorldToRobotWorld()
      ref_m.invert()
      base_m_in_ref = ref_m * statement.Base.Node.WorldPositionMatrix * statement.Position
    pos = write_matrix(base_m_in_ref)
    helper.write_rou('%s := %s' % (statement.Base.Name, pos))
    helper.write_rou('$UFRAME := %s' % (statement.Base.Name))
  else:
    #Just assume that node stays the same
    pos = write_matrix(statement.Position)
    helper.write_rou('%s := %s : %s' % (statement.Base.Name, statement.Base.Name, pos))
    helper.write_rou('$UFRAME := %s' % (statement.Base.Name))


def write_define_tool(statement, helper):
  if not statement.Tool:
    return
  
  if not statement.IsRelative:
    if statement.Node and statement.Node != helper.controller.FlangeNode:
      ref_m = helper.controller.FlangeNode.WorldPositionMatrix
      ref_m.invert()
      tool_m = statement.Node.WorldPositionMatrix * statement.Position
      tool_m_in_ref = ref_m * tool_m
    else:
      tool_m_in_ref = statement.Position
    pos = write_matrix(tool_m_in_ref)
    helper.write_rou('%s := %s' % (statement.Tool.Name, pos))
    helper.write_rou('$TOOL := %s' % (statement.Tool.Name))
  else:
    #Just assume that node stays the same
    pos = write_matrix(statement.Position)
    helper.write_rou('%s := %s : %s' % (statement.Tool.Name, statement.Tool.Name, pos))
    helper.write_rou('$TOOL := %s' % (statement.Tool.Name))


def write_delay(statement, helper):
  helper.write_rou('DELAY %.0f' % (statement.Delay * 1000.0))


def write_halt(statement, helper):
  helper.write_rou('PAUSE')


def write_continue(statement, helper):
  helper.write_rou('CONTINUE')


def write_if(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_rou('IF %s THEN' % condition)
  helper.depth += 1
  for s in statement.ThenScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_rou('ELSE')
  helper.depth += 1
  for s in statement.ElseScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_rou('ENDIF')
  

def write_lin_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  write_motion_data(statement, helper)
  pos = write_target(statement, helper)
  move = 'MOVE'
  advance = ''
  if helper.motiontarget.AccuracyValue > 0:
    move = 'MOVEFLY'
    advance = ' ADVANCE'
  helper.write_rou('%s LINEAR TO %s%s' % (move, pos, advance))


def write_path(statement, helper):
  base_name = ''
  if statement.Base:
    base_name = statement.Base.Name
  tool_name = ''
  if statement.Tool:
    tool_name = statement.Tool.Name
  helper.motiontarget.BaseName = base_name
  helper.motiontarget.ToolName = tool_name
  helper.motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  helper.motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  helper.motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR    
  
  helper.write_rou('--PATH %s' % (statement.Name))
  for i in range(statement.getSchemaSize()):
    target = statement.getSchemaValue(i,'Position')
    helper.motiontarget.Target = target
    jv = helper.motiontarget.JointValues
    helper.motiontarget.JointValues = jv
    helper.motiontarget.CartesianSpeed = statement.getSchemaValue(i,'MaxSpeed')
    helper.motiontarget.CartesianAcceleration = statement.getSchemaValue(i,'Acceleration')
    helper.motiontarget.AccuracyMethod = statement.getSchemaValue(i,'AccuracyMethod')
    helper.motiontarget.AccuracyValue = statement.getSchemaValue(i,'AccuracyValue')
    
    write_motion_data(statement, helper, i)
    pos = write_target(statement, helper, i)
    move = 'MOVE'
    advance = ''
    if helper.motiontarget.AccuracyValue > 0:
      move = 'MOVEFLY'
      advance = ' ADVANCE'
    helper.write_rou('%s LINEAR TO %s%s' % (move, pos, advance))
  helper.write_rou('--ENDPATH')


def write_print(statement, helper):
  helper.write_rou('WRITE (\'%s\', NL)' % statement.Message)


def write_ptp_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  write_motion_data(statement, helper)
  pos = write_target(statement, helper)
  move = 'MOVE'
  advance = ''
  if helper.motiontarget.AccuracyValue > 0:
    move = 'MOVEFLY'
    advance = ' ADVANCE'
  helper.write_rou('%s TO %s%s' % (move, pos, advance))


def write_return(statement, helper):
  helper.write_rou('RETURN')


def write_set_bin(statement, helper):
  val = 'OFF'
  if statement.OutputValue:
    val = 'ON'
  helper.write_rou('$DOUT[%i] := %s' % (statement.OutputPort, val))


def write_set_property(statement, helper):
  #Check if target property is declared in locals or globals
  if not statement.TargetProperty in helper.local_vars and not statement.TargetProperty in helper.global_vars:
    #Not declared, probably a global (component property)
    prop = helper.component.getProperty(statement.TargetProperty)
    if prop:
      #Declare as global
      types = {VC_BOOLEAN : 'BOOLEAN', VC_INTEGER : 'INTEGER', VC_REAL : 'REAL', VC_STRING : 'STRING'}
      if prop.Type in types.keys():
        helper.write_var('%s : %s' % (prop.Name, types[prop.Type]), 1)
        helper.global_vars.append(prop.Name)
        #
        if helper.create_variable_file:
          val = str(prop.Value)
          if prop.Type == VC_BOOLEAN:
            val = val.upper()
          helper.write_lsv('%s %s Priv %s' % (prop.Name, types[prop.Type][0:3], val))
          helper.write_lsv('')
    
  target = check_expression(statement.TargetProperty, helper)
  value_expression = check_expression(statement.ValueExpression, helper)
  helper.write_rou('%s := %s' % (target, value_expression))


def write_wait_bin(statement, helper):
  val = 'OFF'
  if statement.InputValue:
    val = 'ON'
  helper.write_rou('WAIT FOR $DIN[%i] = %s' % (statement.InputPort, val))


def write_while(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_rou('WHILE %s DO' % condition)
  helper.depth += 1
  for s in statement.Scope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_rou('ENDWHILE')


def unknown(statement, helper):
  print '> Unsupported statement type skipped:', statement.Type


def write_target(statement, helper, schema_index = -1):
  conf = ''
  if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
    #Solve conf string
    confs = ['', 'W', 'E', 'E W', 'S', 'S W', 'S E', 'S E W']
    if helper.motiontarget.RobotConfig >= 0 and helper.motiontarget.RobotConfig < 8:
      conf = confs[helper.motiontarget.RobotConfig]
    #Turns
    if helper.robot_joint_count == 6:
      jvs = helper.motiontarget.JointValues
      t1 = ''
      if jvs[3] <= -180:
        t1 = 'T1:-1'
      elif jvs[3] > 180:
        t1 = 'T1:1'
      t2 = ''
      if jvs[5] <= -180:
        t2 = 'T2:-1'
      elif jvs[5] > 180:
        t2 = 'T2:1'
      if conf and t1:
        conf += ' '
      conf += t1
      if conf and t2:
        conf += ' '
      conf += t2
  #endif
  if helper.create_variable_file:
    if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      pos = get_unique_name(statement.Positions[0].Name, helper.pos_names)
    else:# if statement.Type == VC_STATEMENT_PATH:
      pos = get_unique_name(statement.Name + '_' + str(schema_index + 1), helper.pos_names)
    helper.pos_names.append(pos)
    
    helper.write_var('%s : POSITION' % (pos))
    helper.local_vars.append(pos)
    
    lvs_pos = '%s POS Priv ' % (pos)
    lvs_pos += write_matrix_lvs(helper.motiontarget.Target, conf)
    helper.write_lsv(lvs_pos)
    helper.write_lsv('')
  else:
    pos = write_matrix(helper.motiontarget.Target, conf)
  return pos


def write_motion_data(statement, helper, schema_index = -1):
  # Set active motion data from helper.motiontarget (and statement)
  
  if not helper.motion_data_set:
    helper.motiontarget_cache.BaseName = helper.motiontarget.BaseName
    helper.motiontarget_cache.ToolName = helper.motiontarget.ToolName
    helper.motiontarget_cache.JointSpeedFactor = helper.motiontarget.JointSpeedFactor
    helper.motiontarget_cache.CartesianSpeed = helper.motiontarget.CartesianSpeed
    helper.motiontarget_cache.AccuracyValue = helper.motiontarget.AccuracyValue
    
    helper.write_rou('$UFRAME := %s' % (get_base_name(helper.motiontarget_cache)))
    helper.write_rou('$TOOL := %s' % (get_tool_name(helper.motiontarget_cache)))
    helper.write_rou('$ARM_SPD_OVR := %i' % (int(100 * helper.motiontarget_cache.JointSpeedFactor)))
    helper.write_rou('$LIN_SPD := %.1f' % (helper.motiontarget_cache.CartesianSpeed * 0.001))
    if helper.use_blending:
      helper.write_rou('$FLY_TYPE := FLY_CART')
      helper.write_rou('$FLY_TRAJ := FLY_FROM')
      helper.write_rou('$FLY_DIST := %.1f' % (helper.motiontarget_cache.AccuracyValue))
    
    helper.motion_data_set = True
  else:
    if helper.motiontarget.BaseName != helper.motiontarget_cache.BaseName:
      helper.motiontarget_cache.BaseName = helper.motiontarget.BaseName
      helper.write_rou('$UFRAME := %s' % (get_base_name(helper.motiontarget_cache)))
    
    if helper.motiontarget.ToolName != helper.motiontarget_cache.ToolName:
      helper.motiontarget_cache.ToolName = helper.motiontarget.ToolName
      helper.write_rou('$TOOL := %s' % (get_tool_name(helper.motiontarget_cache)))
    
    if helper.motiontarget.JointSpeedFactor != helper.motiontarget_cache.JointSpeedFactor:
      helper.motiontarget_cache.JointSpeedFactor = helper.motiontarget.JointSpeedFactor
      helper.write_rou('$ARM_SPD_OVR := %i' % (int(100 * helper.motiontarget_cache.JointSpeedFactor)))
    
    if helper.motiontarget.CartesianSpeed != helper.motiontarget_cache.CartesianSpeed:
      helper.motiontarget_cache.CartesianSpeed = helper.motiontarget.CartesianSpeed
      helper.write_rou('$LIN_SPD := %.1f' % (helper.motiontarget_cache.CartesianSpeed * 0.001))
    
    if helper.motiontarget.AccuracyValue != helper.motiontarget_cache.AccuracyValue:
      helper.motiontarget_cache.AccuracyValue = helper.motiontarget.AccuracyValue
      helper.write_rou('$FLY_DIST := %.1f' % (helper.motiontarget_cache.AccuracyValue))


def write_matrix(m, conf = '', pos_angles = True):
  p = m.P
  ori = m.getEuler()
  if pos_angles:
    if ori.X < 0:
      ori.X = ori.X + 360.0
    if ori.Y < 0:
      ori.Y = ori.Y + 360.0
    if ori.Z < 0:
      ori.Z = ori.Z + 360.0
  pos = 'POS(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, \'%s\')' % (p.X, p.Y, p.Z, ori.Z, ori.Y, ori.X, conf)
  return pos


def write_matrix_lvs(m, conf = '', pos_angles = True):
  p = m.P
  ori = m.getEuler()
  if pos_angles:
    if ori.X < 0:
      ori.X = ori.X + 360.0
    if ori.Y < 0:
      ori.Y = ori.Y + 360.0
    if ori.Z < 0:
      ori.Z = ori.Z + 360.0
  pos = 'X:%.3f Y:%.3f Z:%.3f A:%.3f E:%.3f R:%.3f CNFG: \'%s\'' % (p.X, p.Y, p.Z, ori.Z, ori.Y, ori.X, conf)
  return pos


def get_unique_name(name, old_names):
  #Return unique variable name that doesn't exist in old_names
  if not name in old_names:
    return name
  
  exp = r'_*(\d+)\Z'
  match = re.search(exp, name)
  if match:
    seed_name = name[:match.start()]
    index = int(match.group(1))
  else:
    seed_name = name
    index = 1
    
  while True:
    if '_' in name:
      new_name = seed_name + '_' + str(index)
    else:
      new_name = seed_name + str(index)
    if new_name in old_names:
      index = index + 1
      continue
    else:
      return new_name


def check_expression(line, helper):
  # Check expression syntax
  
  # Operators
  line = line.replace('==', '=')
  line = line.replace('!=', '<>')
  line = line.replace('!', ' NOT ')
  line = line.replace('&&', ' AND ')
  line = line.replace('&', ' AND ')
  line = line.replace('||', ' OR ')
  line = line.replace('|', ' OR ')
  
  # Boolean
  line = line.replace('True', 'TRUE')
  line = line.replace('False', 'FALSE')
  
  # Signals
  exp = '(?<!D)IN\[(%s)\]' % (RE.INT)
  match = re.search(exp, line, RE.FLAGS)
  while match:
    line = line.replace(match.group(0), '$DIN[%s]' % (match.group(1)))
    match = re.search(exp, line, RE.FLAGS)
  
  # Whitespace trims
  line = line.replace('  ', ' ')
  line = line.strip().rstrip()
  
  return line


def get_all_statements(helper, scope = None):
  # Return all statements in a scope (at any level) as a list
  
  stats = []
  if not scope:
    prog = helper.program
    stats.extend(get_all_statements(helper, prog.MainRoutine))
    for r in prog.Routines:
      stats.extend(get_all_statements(helper, r))
  else:
    for s in scope.Statements:
      stats.append(s)
      if s.Type == VC_STATEMENT_IF:
        stats.extend(get_all_statements(helper, s.ThenScope))
        stats.extend(get_all_statements(helper, s.ElseScope))
      elif s.Type == VC_STATEMENT_WHILE:
        stats.extend(get_all_statements(helper, s.Scope))
    
  return stats


def get_base_name(motiontarget):
  if motiontarget.BaseName == '':
    return 'NULL'
  return motiontarget.BaseName


def get_tool_name(motiontarget):
  if motiontarget.ToolName == '':
    return 'NULL'
  return motiontarget.ToolName


def get_base_in_reference(base, motiontarget):
  #Convert base matrix to robot world
  if not base.Node:
    return base.PositionMatrix
  ref_m = motiontarget.getSimWorldToRobotWorld()
  ref_m.invert()
  return ref_m * base.Node.WorldPositionMatrix * base.PositionMatrix


def get_tool_in_reference(tool, motiontarget, ctr):
  #Convert tool matrix to robot flange
  if tool.Node and tool.Node != ctr.FlangeNode:
    ref_m = ctr.FlangeNode.WorldPositionMatrix
    ref_m.invert()
    tool_m = tool.Node.WorldPositionMatrix * tool.PositionMatrix
    tool_m_in_ref = ref_m * tool_m
  else:
    tool_m_in_ref = tool.PositionMatrix
  return tool_m_in_ref