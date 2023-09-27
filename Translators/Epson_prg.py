# Epson post processor, Version 1.00

from vcCommand import *
import os.path, time, re, math

class TranslatorHelper:
  #---
  # Capsulates commonly used handles and post processor settings
  #---
  def __init__(self, app, program, uri):
  
    # Settings
    self.prg = '' # Program file contents
    self.pts = '' # Point file contents

    
    # Helpers
    self.app = app
    self.uri = uri
    self.program = program
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.motiontarget = program.Executor.Controller.createTarget()
    self.motion_data_set = False
    self.active_speed = 100
    self.active_accel = 100
    self.active_speeds = 1000
    self.active_accels = 1000
    self.active_base = 0
    self.active_tool = 0
    self.current_routine = None
    self.point_count = 0
    self.robot_joint_count = len([x for x in self.controller.Joints if not x.ExternalController])
    self.total_joint_count = len(self.controller.Joints)
    self.string_vars = []
    self.base_map = {}
    self.tool_map = {}
    self.indentation = '  '
    self.depth = 0
    self.statement_translators = {
      VC_STATEMENT_BREAK:write_break,
      VC_STATEMENT_CALL:write_call,
      VC_STATEMENT_COMMENT:write_comment,
      VC_STATEMENT_CONTINUE:unknown,
      VC_STATEMENT_DELAY:write_delay,
      VC_STATEMENT_HALT:write_halt,
      VC_STATEMENT_IF:write_if,
      VC_STATEMENT_LINMOTION:write_lin_motion,
      VC_STATEMENT_PATH:write_path,
      VC_STATEMENT_PRINT:write_print,
      VC_STATEMENT_PTPMOTION:write_ptp_motion,
      VC_STATEMENT_RETURN:write_return,
      VC_STATEMENT_SETBIN:write_set_bin,
      VC_STATEMENT_SETPROPERTY:write_set_property,
      VC_STATEMENT_WAITBIN:write_wait_bin,
      VC_STATEMENT_WHILE:write_while }
      
  def current_indent(self):
    return self.indentation*self.depth

  def write_prg(self, line):
    self.prg += self.current_indent() + line + '\n'
    
  def write_pts(self, line):
    self.pts += line + '\n'
  
  def get_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  
  def get_prg_filename(self):
    head, tail = os.path.split(self.uri)
    return tail
  
  def get_pts_filename(self):
    head, tail = os.path.split(self.uri)
    base_name = tail[:tail.rfind('.')]
    return base_name + '.pts'
  
  def get_pts_uri(self):
    pts_uri = '%s\\%s' % (self.get_folder(), self.get_pts_filename())
    return pts_uri


class RE(object):
  #---
  #Capsulates RE constants
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
  #Custom exception type
  #---
  pass


def postProcess(app, program, uri):
  # Entry point
  
  helper = TranslatorHelper(app, program, uri)
  
  try:
    # Write main and subroutine to helper cache
    write_routine(program.MainRoutine, 'main', helper)
    for routine in program.Routines:
      write_routine(routine, routine.Name, helper)
    
    # Point file header
    write_pts_header(helper)
  except TranslatorException as e:
    print 'ERROR: ', e
    return False,[]
  
  # Write output files
  with open(helper.uri, 'w') as output_file:
    output_file.write(helper.prg)
  pts_uri = helper.get_pts_uri()
  with open(pts_uri, 'w') as output_file:
    output_file.write(helper.pts)  
  
  return True,[uri, pts_uri]


def write_routine(routine, name, helper):
  # Write a routine
  
  helper.current_routine = routine
  helper.motion_data_set = False    #Reset current speed, acc, frame info
  
  helper.write_prg('Function %s' % name)
  helper.depth += 1
  
  #Routine variables
  types = {}
  types[VC_BOOLEAN] = 'Boolean'
  types[VC_INTEGER] = 'Integer'
  types[VC_REAL] = 'Real'
  types[VC_STRING] = 'String'
  for prop in routine.Properties:
    if prop.Name in ['Name']:
      continue
    if prop.Type in types:
      epson_type = types[prop.Type]
      add_dollar = (prop.Type == VC_STRING) * '$'
      helper.write_prg('%s %s%s' % (epson_type, prop.Name, add_dollar))
      if prop.Type == VC_STRING and not prop.Name in helper.string_vars:
        helper.string_vars.append(prop.Name)
  if len(routine.Properties) > 1:
    helper.write_prg('')
  
  if routine == helper.program.MainRoutine:
    # Some init commands
    helper.write_prg('Motor On')
    helper.write_prg('Power High')
    helper.write_prg('ClearPoints')
    helper.write_prg('LoadPoints "%s"' % (helper.get_pts_filename()))
    helper.write_prg('')
  
    # Base/Tool Frames
    write_frames(helper)
  
  # Init motion data
  mot_stat = get_first_motion_statement(routine)
  if mot_stat:
    mot_stat.writeToTarget(helper.motiontarget)
    write_motion_data(mot_stat, helper, 0)
  
  # Statements
  for statement in routine.Statements:
    translator = helper.statement_translators.get(statement.Type, unknown)
    translator(statement, helper)
    
  helper.depth -= 1
  helper.write_prg('Fend')
  helper.write_prg('')


def write_pts_header(helper):
  # Write header to point file
  
  contents = helper.pts
  helper.pts = ''
  helper.write_pts('ENVT0100,LM:%s:' % (time.strftime('%Y/%m/%d %H:%M:%S:000')))
  helper.write_pts('sVersion="2.0.0"')
  helper.write_pts('nDisplayMode=4')
  helper.write_pts('bDisplayR=False')
  helper.write_pts('bDisplayS=False')
  helper.write_pts('bDisplayT=False')
  helper.write_pts('nNumberOfJoints=%i' % (helper.total_joint_count))
  helper.write_pts('nNumberOfPoints=%i' % (helper.point_count))
  helper.write_pts(contents)


def write_frames(helper):
  bases = []
  tools = []
  all_stats = get_all_statements(helper)
  for s in all_stats:
    if s.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      for b in bases:
        if b == s.Base:
          break
      else:
        bases.append(s.Base)
      for t in tools:
        if t == s.Tool:
          break
      else:
        tools.append(s.Tool)
  
  for b in bases:
    if not b:
      continue
    base_number = get_base_number(b.Name, helper)
    pos = get_base_in_reference(b, helper.motiontarget)
    p = pos.P
    wpr = pos.WPR
    helper.write_prg('Local %i, XY(%.3f, %.3f, %.3f, %.4f, %.4f, %.4f)' % (base_number, p.X, p.Y, p.Z, wpr.Z, wpr.Y, wpr.X))
    
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
    tool_number = get_tool_number(t.Name, helper)
    pos = get_tool_in_reference(t, helper.motiontarget, helper.controller)
    p = pos.P
    wpr = pos.WPR
    helper.write_prg('TLSet %i, XY(%.3f, %.3f, %.3f, %.4f, %.4f, %.4f)' % (tool_number, p.X, p.Y, p.Z, wpr.Z, wpr.Y, wpr.X))
    
  helper.write_prg('')


def write_break(statement, helper):
  helper.write_prg('Exit Do')


def write_call( statement, helper):
  rou = statement.getProperty('Routine').Value
  if rou:
    helper.write_prg('Call %s' % rou.Name)


def write_comment(statement, helper):
  helper.write_prg('\'%s' % statement.Comment)


def write_delay(statement, helper):
  helper.write_prg('Wait %.2f' % statement.Delay)


def write_halt(statement, helper):
  helper.write_prg('Pause')


def write_if(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_prg('If %s Then' % condition)
  helper.depth += 1
  for s in statement.ThenScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_prg('Else')
  helper.depth += 1
  for s in statement.ElseScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_prg('EndIf')


def write_lin_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  write_motion_data(statement, helper)
  point_name = write_target(statement, helper)
  cp = ''
  if helper.motiontarget.AccuracyValue > 0:
    cp = ' CP'
  helper.write_prg('Move %s%s' % (point_name, cp))


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
    point_name = write_target(statement, helper, i)
    cp = ''
    if helper.motiontarget.AccuracyValue > 0:
      cp = ' CP'
    helper.write_prg('Move %s%s' % (point_name, cp))


def write_print(statement, helper):
  helper.write_prg('Print "%s"' % statement.Message)


def write_ptp_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  write_motion_data(statement, helper)
  point_name = write_target(statement, helper)
  cp = ''
  if helper.motiontarget.AccuracyValue > 0:
    cp = ' CP'
  helper.write_prg('Go %s%s' % (point_name, cp))


def write_return(statement, helper):
  helper.write_prg('Exit Function')


def write_set_bin(statement, helper):
  val = 'Off'
  if statement.OutputValue:
    val = 'On'
  helper.write_prg('%s %i' % (val, statement.OutputPort))


def write_set_property(statement, helper):
  target = check_expression(statement.TargetProperty, helper)
  value_expression = check_expression(statement.ValueExpression, helper)
  helper.write_prg('%s = %s' % (target, value_expression))


def write_wait_bin(statement, helper):
  val = '0'
  if statement.InputValue:
    val = '1'
  helper.write_prg('Wait Sw(%i) = %s' % (statement.InputPort, val))


def write_while(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_prg('Do While %s' % condition)
  helper.depth += 1
  for s in statement.Scope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_prg('Loop')


def unknown(statement, helper):
  print '> Unsupported statement type skipped:', statement.Type


def write_target(statement, helper, schema_index = -1):
  # Write helper.motiontarget to point file
  
  jvs = helper.motiontarget.JointValues
  point_name = ''
  if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
    point_name = statement.Positions[0].Name
  elif statement.Type == VC_STATEMENT_PATH:
    point_name = '%s_%i' % (statement.Name, schema_index + 1)
  exp = 'p%s' % (RE.INT)
  if re.match(exp, point_name, RE.FLAGS):
    #Change name as p1 is not allowed
    point_name = point_name[0] + point_name
  p = helper.motiontarget.Target.P
  wpr = helper.motiontarget.Target.WPR
  cr = 0 #7th axis
  cs = 0 #E1
  ct = 0 #E2
  for i in range(helper.robot_joint_count, helper.total_joint_count):
    if i == helper.robot_joint_count:
      cs = jvs[i]
    if i == helper.robot_joint_count + 1:
      ct = jvs[i]
  local = get_base_number(helper.motiontarget.BaseName, helper)
  conf = helper.motiontarget.RobotConfig
  hand = 0
  elbow = 0
  wrist = 0
  j1f = 0
  j2f = 0
  j4f = 0
  j6f = 0
  j1a = 0
  j4a = 0
  if helper.robot_joint_count == 4:
    #Scara
    hand = 2 - conf % 2
    j1f = (abs(jvs[0]) > 180) * 1
    j2f = (abs(jvs[1]) > 180) * 1
    if len(helper.component.Name) >= 2 and helper.component.Name[0:2] == 'RS':
      j1a = jvs[0]
  elif helper.robot_joint_count >= 6:
    #6-axis
    hand = (conf in [4, 5, 6, 7]) * 1 + 1
    elbow = (conf in [2, 3, 4, 5]) * 1 + 1
    wrist = (conf in [1, 3, 4, 6]) * 1 + 1
    j4f = (abs(jvs[3]) > 180) * 1
    j6f = (abs(jvs[5]) > 180) * 1
    if len(helper.component.Name) >= 1 and helper.component.Name[0:1] == 'N':
      j4a = jvs[3]  
      
  helper.write_pts('Point%i {' % (helper.point_count + 1))
  helper.write_pts('	nNumber=%i' % (helper.point_count))
  helper.write_pts('  sLabel="%s"' % (point_name))
  helper.write_pts('  sDescription=""')
  helper.write_pts('  nUndefined=0')
  helper.write_pts('  rX=%.3f' % p.X)
  helper.write_pts('  rY=%.3f' % p.Y)
  helper.write_pts('  rZ=%.3f' % p.Z)
  helper.write_pts('  rU=%.4f' % wpr.Z)
  helper.write_pts('  rV=%.4f' % wpr.Y)
  helper.write_pts('  rW=%.4f' % wpr.X)
  helper.write_pts('  rR=%.3f' % cr)
  helper.write_pts('  rS=%.3f' % cs)
  helper.write_pts('  rT=%.3f' % ct)
  helper.write_pts('  nLocal=%i' % local)
  helper.write_pts('  nHand=%i' % hand)
  helper.write_pts('  nElbow=%i' % elbow)
  helper.write_pts('  nWrist=%i' % wrist)
  helper.write_pts('  nJ1Flag=%i' % j1f)
  helper.write_pts('  nJ2Flag=%i' % j2f)
  helper.write_pts('  nJ4Flag=%i' % j4f)
  helper.write_pts('  nJ6Flag=%i' % j6f)
  helper.write_pts('  rJ1Angle=%.3f' % j1a)
  helper.write_pts('  rJ4Angle=%.3f' % j4a)
  helper.write_pts('  bSimVisible=False')
  helper.write_pts('}')
  
  helper.point_count += 1
  return point_name


def write_motion_data(statement, helper, schema_index = -1):
  # Set active motion data from helper.motiontarget (and statement)
  
  if not helper.motion_data_set:
    helper.active_tool = get_tool_number(helper.motiontarget.ToolName, helper)
    helper.active_speed = helper.motiontarget.JointSpeedFactor * 100.0
    if statement.Type == VC_STATEMENT_PTPMOTION:
      helper.active_accel = statement.JointForce * 100.0
    helper.active_speeds = helper.motiontarget.CartesianSpeed
    helper.active_accels = helper.motiontarget.CartesianAcceleration
    
    helper.write_prg('Tool %i' % (helper.active_tool))
    helper.write_prg('Speed %.0f' % (helper.active_speed))
    helper.write_prg('Accel %.0f,%.0f' % (helper.active_accel, helper.active_accel))
    helper.write_prg('SpeedS %.0f' % (helper.active_speeds))
    helper.write_prg('AccelS %.0f' % (helper.active_accels))
    helper.write_prg('')
    helper.motion_data_set = True
  else:
    new_active_tool = get_tool_number(helper.motiontarget.ToolName, helper)
    if new_active_tool != helper.active_tool:
      helper.active_tool = new_active_tool
      helper.write_prg('Tool %i' % (helper.active_tool))
  
    new_active_speed = helper.motiontarget.JointSpeedFactor * 100.0
    if new_active_speed != helper.active_speed:
      helper.active_speed = new_active_speed
      helper.write_prg('Speed %.0f' % (helper.active_speed))
    
    if statement.Type == VC_STATEMENT_PTPMOTION:
      new_active_accel = statement.JointForce * 100.0
      if new_active_accel != helper.active_accel:
        helper.active_accel = new_active_accel
        helper.write_prg('Accel %.0f,%.0f' % (helper.active_accel, helper.active_accel))
    
    new_active_speeds = helper.motiontarget.CartesianSpeed
    if new_active_speeds != helper.active_speeds:
      helper.active_speeds = new_active_speeds
      helper.write_prg('SpeedS %.0f' % (helper.active_speeds))
    
    new_active_accels = helper.motiontarget.CartesianAcceleration
    if new_active_accels != helper.active_accels:
      helper.active_accels = new_active_accels
      helper.write_prg('AccelS %.0f' % (helper.active_accels))
  

def check_expression(line, helper):
  # Check expression syntax
  
  # Format operators
  line = line.replace('==', '=')
  line = line.replace('!=', '<>')
  line = line.replace('!', ' Not ')
  line = line.replace('&&', ' And ')
  line = line.replace('&', ' And ')
  line = line.replace('||', ' Or ')
  line = line.replace('|', ' Or ')
  
  # Signals
  exp = 'IN\[(%s)\]' % (RE.INT)
  match = re.search(exp, line, RE.FLAGS)
  while match:
    line = line.replace(match.group(0), '(Sw(%s) = 1)' % (match.group(1)))
    match = re.search(exp, line, RE.FLAGS)
  
  #String var conversions
  for s in helper.string_vars:
    exp = exp = '(\A|\Z|[ =<>*/+-])(%s)(\A|\Z|[ =<>*/+-])' % s
    match = re.search(exp, line, RE.FLAGS)
    while match:
      line = line.replace(match.group(0), '%s%s$%s' % (match.group(1), match.group(2), match.group(3)))
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


def get_first_motion_statement(scope):
  mot_stat = None
  for stat in scope.Statements:
    if stat.Type == VC_STATEMENT_IF:
      mot_stat = get_first_motion_statement(stat.ThenScope)
      if not mot_stat:
        mot_stat = get_first_motion_statement(stat.ElseScope)
    elif stat.Type == VC_STATEMENT_WHILE:
      mot_stat = get_first_motion_statement(stat.Scope)
    elif stat.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      mot_stat = stat
    #elif stat.Type == VC_STATEMENT_PATH and stat.getSchemaSize() > 0:
    #  mot_stat = stat
    if mot_stat:
      return mot_stat
  return None
  

def get_base_number(base_name, helper):
  #0 = Robot world, 1-15 = bases, -1 = unmapped
  if not base_name:
    return 0
  if base_name in helper.base_map:
    return helper.base_map[base_name]
  number = -1
  exp = 'base(%s)' % RE.INT
  match = re.match(exp, base_name, RE.FLAGS)
  if match:
    try:
      number = int(match.group(1))
    except:
      pass
  if number < 0:
    #Try to map custom named base to one of the latter indices
    for i in range(15, 10, -1):
      if i not in helper.base_map.values():
        number = i
        break
  if number < 0:
    print 'WARNING: Cannot map base %s to local coordinate system.' % (base_name)
  helper.base_map[base_name] = number
  return number


def get_tool_number(tool_name, helper):
  #0 = tool0, 1-15 = tool, -1 = unmapped
  if not tool_name:
    return 0
  if tool_name in helper.tool_map:
    return helper.tool_map[tool_name]
  number = -1
  exp = 'tool(%s)' % RE.INT
  match = re.match(exp, tool_name, RE.FLAGS)
  if match:
    try:
      number = int(match.group(1))
    except:
      pass
  if number < 0:
    #Try to map custom named tool to one of the latter indices
    for i in range(15, 10, -1):
      if i not in helper.tool_map.values():
        number = i
        break
  if number < 0:
    print 'WARNING: Cannot map base %s to local coordinate system.' % (tool_name)
  helper.tool_map[tool_name] = number
  return number


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
