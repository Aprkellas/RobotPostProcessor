# Hyundai post processor, Version 1.00

from vcCommand import *
import os.path, time, re, math
import vcMatrix, vcVector

class TranslatorHelper:
  #---
  # Capsulates commonly used handles and post processor settings
  #---
  def __init__(self, app, program, uri):
    
    self.cmd = getCommand()
    
    # Settings
    self.JOB =    ''  # Program file contents
    self.FRAMES = ''  # Contains tool and user frame info
    self.convert_to_robot_frame = self.cmd.getProperty('Convert positions to Robot frame').Value
    self.assign_frames_in_job = self.cmd.getProperty('Assign Tool/User frames in job').Value
    self.write_frame_file = self.cmd.getProperty('Create Tool/User frame text file').Value
    
    # Helpers
    self.app = app
    self.uri = uri
    self.program = program
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.motiontarget = program.Executor.Controller.createTarget()
    self.robot_joint_count = len([x for x in self.controller.Joints if not x.ExternalController])
    self.total_joint_count = len(self.controller.Joints)
    self.indentation = '     '
    self.depth = 0
    
    self.labels = []
    self.keywords = ['NOT', 'AND', 'OR']
    self.var_types = { VC_BOOLEAN:'Integer', VC_INTEGER:'Integer', VC_REAL:'Double', VC_STRING:'String' }
    self.routine_map = {}
    self.global_vars = {}
    self.current_routine = None
    self.while_stack = []
    self.motion_count = 0
    self.used_tool_indices = []
    self.used_base_indices = []
    
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
      VC_STATEMENT_PROG_SYNC:unknown,
      VC_STATEMENT_PTPMOTION:write_ptp_motion,
      VC_STATEMENT_RETURN:write_return,
      VC_STATEMENT_SETBIN:write_set_bin,
      VC_STATEMENT_SETPROPERTY:write_set_property,
      VC_STATEMENT_WAITBIN:write_wait_bin,
      VC_STATEMENT_WHILE:write_while }
      
  def current_indent(self):
    return self.indentation*self.depth

  def write_job(self, line, SX = ''):
    # SX is the step number at the beginning of the line on motions.
    indent = self.current_indent()
    SX_and_indent = SX + max(0, len(indent)-len(SX)) * ' '
    self.JOB += SX_and_indent + line + '\n'
  
  def write_frames(self, line):
    self.FRAMES += line + '\n'
  
  def get_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  
  def get_job_filename(self):
    head, tail = os.path.split(self.uri)
    return tail
  
  def get_frame_file(self):
    file_path = os.path.join(self.get_folder(), 'FrameInfo.txt')
    return file_path


class RE(object):
  #---
  #Capsulates RE constants
  #---
  ANY = r'.+'
  SPC = r'\s'
  SPCQ = r'\s*'
  EOL = r'\n'
  EOS = r'$'
  INT = r'\s*-?\d+'
  NAME = r'\w+'
  NUM = r'\s*-?\d+\.?\d*[eE]?-?\d*'
  VAR = r'[a-z][a-z0-9]*(?![a-z\[\(])'
  ARRAY = r'[a-z][a-z0-9]*\[[0-9]+\]'
  FUNC = r'[a-z][a-z0-9]*\(.*?\)'
  STRING = r'\".*?\"'
  FLAGS = re.I|re.S


class TranslatorException(Exception):
  #---
  #Custom exception type
  #---
  pass


def postProcess(app, program, uri):
  # Entry point
  
  helper = TranslatorHelper(app, program, uri)
  check_job_name(helper)
  
  # Check ex axes
  if helper.total_joint_count > helper.robot_joint_count:
    print 'WARNING: PP doesn\'t support external axes.'
  
  # Header
  write_header(helper)
  helper.depth += 1
  
  # Write frames
  write_frames(helper)
  
  # Main routine
  write_routine(helper.program.MainRoutine, helper, True)
  
  # Subroutines
  for routine in helper.program.Routines:
    write_routine(routine, helper)
  
  helper.depth -= 1
  
  # Write output files
  files = [uri]
  with open(helper.uri, 'w') as output_file:
    output_file.write(helper.JOB)
  if helper.write_frame_file:
    frame_uri = helper.get_frame_file()
    files.append(frame_uri)
    with open(frame_uri, 'w') as output_file:
      output_file.write(helper.FRAMES)
  
  # Show global variable mapping
  if helper.global_vars:
    print 'Global variable mapping: '
    for key in helper.global_vars:
      print '  %s : %s' % (key, helper.global_vars[key])
  
  return True,files


def getProperties():
  #Properties for action panel
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  props.append((VC_BOOLEAN, 'Convert positions to Robot frame', False, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Assign Tool/User frames in job', False, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Create Tool/User frame text file', False, None, None, 0, 0))
  return props



def getDefaultJobName():
  return '0001.JOB'


def write_frames(helper):
  # Write tool frames to job (if enabled) and tools and user frames to info file (if enabled).
  
  if not helper.assign_frames_in_job and not helper.write_frame_file:
    return
  
  used_bases = []
  used_tools = []
  all_stmts = get_all_statements(helper)
  for stmt in all_stmts:
    try:
      base_name = ''
      tool_name = ''
      if stmt.Base:
        base_name = stmt.Base.Name
      if stmt.Tool:
        tool_name = stmt.Tool.Name
      if not base_name in used_bases:
        used_bases.append(base_name)
      if not tool_name in used_tools:
        used_tools.append(tool_name)
    except:
      pass
  used_bases.sort()
  used_tools.sort()
  
  if helper.assign_frames_in_job:
    if used_tools:
      helper.write_job('\'Tools')
      for tool_name in used_tools:
        tool_index = get_tool_index(tool_name, helper)
        tool = get_tool(tool_name, helper)
        tool_m = get_tool_in_reference(tool, helper)
        tool_value = write_matrix(tool_m)
        helper.write_job('Tool%i=(%s)T' % (tool_index, tool_value))
      helper.write_job('')
    
    if used_bases:
      helper.write_job('\'User frames')
      for base_name in used_bases:
        if not base_name:
          continue
        base_index = get_base_index(base_name, helper)
        base = get_base(base_name, helper)
        base_m = get_base_in_reference(base, helper)
        base_value = write_matrix(base_m)
        helper.write_job('MKUCRD %i,(%s,&H0000)' % (base_index, base_value))
      helper.write_job('')
  
  if helper.write_frame_file:
    helper.write_frames('Tool and user frame info. Copy these values to robot.')
    helper.write_frames('')
    
    helper.write_frames('Tool frames (X,Y,Z,RX,RY,RZ)')
    for tool_name in used_tools:
      tool_index = get_tool_index(tool_name, helper)
      tool = get_tool(tool_name, helper)
      tool_m = get_tool_in_reference(tool, helper)
      tool_value = write_matrix(tool_m)
      helper.write_frames('Tool%i=(%s)' % (tool_index, tool_value))
    
    if not helper.convert_to_robot_frame:
      helper.write_frames('')
      helper.write_frames('User frames (X,Y,Z,RX,RY,RZ)')
      for base_name in used_bases:
        if not base_name:
          continue
        base_index = get_base_index(base_name, helper)
        base = get_base(base_name, helper)
        base_m = get_base_in_reference(base, helper)
        base_value = write_matrix(base_m)
        helper.write_frames('User%i=(%s)' % (base_index, base_value))
  

def write_header(helper):
  header = 'Program File Format Version : 1.0  MechType: _MECHTYPE  TotalAxis: %i  AuxAxis: %i' % (helper.robot_joint_count, helper.total_joint_count - helper.robot_joint_count)
  helper.write_job(header)


def write_routine(routine, helper, is_main = False):
  #Write single routine to JOB
  
  helper.current_routine = routine
  routine_name = check_routine_name(routine.Name, helper)
  
  # Start
  if is_main:
    helper.write_job('\'%s' % (routine_name))
  else:
    helper.write_job('')
    helper.write_job('*%s' % (routine_name))
    helper.labels.append(routine_name)
  
  #Routine variables
  for prop in routine.Properties:
    if prop.Name in ['Name']:
      continue
    if prop.Type in helper.var_types.keys():
      native_type = helper.var_types[prop.Type]
      var_name = check_var_name(prop.Name, helper)
      helper.write_job('DIM %s AS %s' % (var_name, native_type))
  
  # Statements
  for statement in routine.Statements:
    translator = helper.statement_translators.get(statement.Type, unknown)
    translator(statement, helper)
  
  # End
  if is_main:
    helper.write_job('END')
  else:
    helper.write_job('RETURN')


def write_break(statement, helper):
  if helper.while_stack:
    helper.write_job('GOTO *END%s' % (helper.while_stack[-1]))


def write_call(statement, helper):
  rou = statement.getProperty('Routine').Value
  if rou:
    rou_name = check_routine_name(rou.Name, helper)
    helper.write_job('GOSUB *%s' % rou_name)


def write_comment(statement, helper):
  helper.write_job('\'%s' % statement.Comment)


def write_continue(statement, helper):
  if helper.while_stack:
    helper.write_job('GOTO *%s' % (helper.while_stack[-1]))


def write_define_base(statement, helper):
  base = statement.Base
  if not base:
    return
  base_name = base.Name
  base_index = get_base_index(base_name, helper)
  base_m = statement.Position
  if statement.IsRelative:
    base_value = write_matrix(base_m)
    helper.write_job('MKUCRD %i,(%s,&H0000)U%i' % (base_index, base_value, base_index))
  else:
    # Convert to robot coordinates
    if statement.Node:
      base_m = helper.component.InverseWorldPositionMatrix * statement.Node.WorldPositionMatrix * base_m
    base_value = write_matrix(base_m)
    helper.write_job('MKUCRD %i,(%s,&H0000)' % (base_index, base_value))
  helper.write_job('')


def write_define_tool(statement, helper):
  tool = statement.Tool
  if not tool:
    return
  tool_name = tool.Name
  tool_index = get_tool_index(tool_name, helper)
  tool_m = statement.Position
  if statement.IsRelative:
    # Not supported at the moment
    print 'WARNING: Define Tool Statement with relative offset is not supported.'
    return
  else:
    # Convert to flange coordinates
    if statement.Node:
      ref_m = helper.controller.FlangeNode.WorldPositionMatrix
      ref_m.invert()
      tool_m = ref_m * tool.Node.WorldPositionMatrix * tool_m
    tool_value = write_matrix(tool_m)
    helper.write_job('Tool%i=(%s)T' % (tool_index, tool_value))
  helper.write_job('')


def write_delay(statement, helper):
  helper.write_job('DELAY %.2f' % statement.Delay)


def write_halt(statement, helper):
  helper.write_job('STOP')


def write_if(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_job('IF %s THEN' % condition)
  for s in statement.ThenScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.write_job('ELSE')
  for s in statement.ElseScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.write_job('ENDIF')


def write_lin_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  target = write_target(statement, helper)
  helper.write_job('MOVE %s' % (target), 'S%i' % (helper.motion_count))


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
    
    target = write_target(statement, helper)
    helper.write_job('MOVE %s' % (target), 'S%i' % (helper.motion_count))


def write_print(statement, helper):
  helper.write_job('PRINT #0, "%s"' % (statement.Message))


def write_ptp_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  target = write_target(statement, helper)
  helper.write_job('MOVE %s' % (target), 'S%i' % (helper.motion_count))


def write_return(statement, helper):
  helper.write_job('RETURN')


def write_set_bin(statement, helper):
  val = '0'
  if statement.OutputValue:
    val = '1'
  helper.write_job('DO%i=%s' % (statement.OutputPort, val))
  if statement.OutputPort == 0:
    print 'WARNING: Set binary output port 0 is not supported. Consider using non-zero port.'


def write_set_property(statement, helper):
  target = check_var_name(statement.TargetProperty, helper)
  value_expression = check_expression(statement.ValueExpression, helper)
  helper.write_job('%s = %s' % (target, value_expression))


def write_wait_bin(statement, helper):
  val = '0'
  if statement.InputValue:
    val = '1'
  helper.write_job('WAIT DI%i=%s' % (statement.InputPort, val))
  if statement.InputPort == 0:
    print 'WARNING: Wait binary input port 0 is not supported. Consider using non-zero port.'


def write_while(statement, helper):
  #No while in native language, use GOTO - LABEL instead
  condition = check_expression(statement.Condition, helper)
  label = get_unique_label(helper, 'WHL')
  helper.while_stack.append(label)
  helper.write_job('*%s' % (label))
  helper.write_job('IF (%s)=0 THEN *END%s' % (condition, label))
  for s in statement.Scope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.write_job('GOTO *%s' % (label))
  helper.write_job('*END%s' % (label))
  helper.while_stack.pop()


def unknown(statement, helper):
  print 'WARNING: Unsupported statement type skipped:', statement.Type


def write_target(statement, helper, schema_index = -1):
  # Write helper.motiontarget to string (e.g. '(896.778,102.196,955.691,-180.000,0.003,-177.597,&H0008),S=60%,A=3,T=0')
  helper.motion_count += 1
  tgt_string = ''
  
  # Interpolation
  ip = ''
  if helper.motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
    ip = 'P'
  else:
    ip = 'L'
  ext_tcp = is_external_tcp(helper)
  if ext_tcp:
    ip = 'S' + ip
  
  
  # Target
  tgt = helper.motiontarget.Target
  if helper.convert_to_robot_frame:
    base = get_base(helper.motiontarget.BaseName, helper)
    base_m = get_base_in_reference(base, helper)
    tgt = base_m * tgt
  if ext_tcp:
    tgt.invert()
  m = write_matrix(tgt)
  
  # CFG value
  cfg = 0
  if helper.robot_joint_count == 6:
    # Config
    cfg_map = [8, 0, 12, 4, 10, 2, 14, 6]
    if helper.motiontarget.RobotConfig >= 0 and helper.motiontarget.RobotConfig < 8:
      cfg = cfg_map[helper.motiontarget.RobotConfig]
      
    # Turns
    jvs = helper.motiontarget.JointValues
    if abs(jvs[0]) > 180:
      cfg += 16
    if abs(jvs[3]) > 180:
      cfg += 32
    if abs(jvs[5]) > 180:
      cfg += 64
  #endif
  
  # Speed
  if helper.motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
    s = '%.0f%%' % (helper.motiontarget.JointSpeedFactor * 100)
  else:
    s = '%.1fmm/s' % (helper.motiontarget.CartesianSpeed)
  
  # Accuracy value
  a = 7
  a_table = [0.1, 1.0, 5.0, 20.0, 50.0, 100.0, 200.0]
  for i, limit in enumerate(a_table):
    if helper.motiontarget.AccuracyValue <= limit:
      a = i
      break
  
  # Frame
  f = ''
  base_index = get_base_index(helper.motiontarget.BaseName, helper)
  if base_index > 0 and not helper.convert_to_robot_frame:
    f = 'U%i' % (base_index)
  
  # Tool
  t = get_tool_index(helper.motiontarget.ToolName, helper)
  
  # Target string
  tgt_string = '%s,(%s,&H%04X)%s,S=%s,A=%i,T=%i' % (ip, m, cfg, f, s, a, t)
  
  return tgt_string


def write_matrix(m):
  p = m.P
  wpr = m.WPR
  return '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (p.X, p.Y, p.Z, wpr.X, wpr.Y, wpr.Z)


def is_external_tcp(helper):
  # Check if helper.motiontarget is using external tcp (not 100% robust check)
  tool_on_robot = False
  base_on_robot = False
  
  tool = None
  for t in helper.controller.Tools:
    if t.Name == helper.motiontarget.ToolName:
      tool = t
      break
  if not tool or not tool.Node:
    tool_on_robot = True
  else:
    node = tool.Node
    while node != helper.app.Simulation.World:
      if node == helper.controller.FlangeNode:
        tool_on_robot = True
        break
      node = node.Parent
  
  base = None
  for b in helper.controller.Bases:
    if b.Name == helper.motiontarget.BaseName:
      base = b
      break
  if base and base.Node:
    node = base.Node
    while node != helper.app.Simulation.World:
      if node == helper.controller.FlangeNode:
        base_on_robot = True
        break
      node = node.Parent
  
  return ((not tool_on_robot and base_on_robot) ^ (helper.motiontarget.TargetMode == VC_MOTIONTARGET_TM_STATICTOOL))


def check_job_name(helper):
  #Check that job name is following the standard
  name = helper.get_job_filename()[:-4]
  ok = True
  if len(name) != 4:
    ok = False
  try:
    i = int(name)
  except:
    ok = False
  
  if not ok:
    print 'WARNING: Job name is not in allowed range 0001.JOB - 9999.JOB.'


def check_routine_name(rou_name, helper):
  #Routines and calls use labels which can have max 8 characters
  mapped_name = ''
  if not rou_name in helper.routine_map.keys():
    if len(rou_name) <= 8:
      helper.routine_map[rou_name] = rou_name
    else:
      for i in range(1, 999):
        suffix = '_%i' % i
        mapped_name = rou_name[:min(len(rou_name), (8-len(suffix)))] + suffix
        if not mapped_name in helper.routine_map.values():
          break
      helper.routine_map[rou_name] = mapped_name
  mapped_name = helper.routine_map[rou_name]
  return mapped_name


def get_unique_label(helper, seed):
  for i in range(10000):
    label = '%s%i' % (seed, (i+1))
    if label not in helper.labels:
      break
  helper.labels.append(label)
  return label


def check_var_name(var_name, helper):
  # Check that local starts with 'l' and global starts with 'g'
  if not var_name or not helper.current_routine:
    return ''
  is_global = True
  for p in helper.current_routine.Properties:
    if p.Name == var_name:
      is_global = False
      break
  
  if is_global:
    #Global, try to map to a global non-id variable
    if var_name in helper.global_vars.keys():
      var_name = helper.global_vars[var_name]
    else:
      prop = helper.component.getProperty(var_name)
      base_name = 'VX%' #Integer
      if prop and prop.Type in helper.var_types.keys():
        native_type = helper.var_types[prop.Type]
        if native_type == 'Double':
          base_name = 'VX!'
        elif native_type == 'String':
          base_name = 'VX$'
      for i in range(1, 600):
        name = base_name.replace('X', '%i' % (i))
        if not name in helper.global_vars.keys():
          helper.global_vars[var_name] = name
          var_name = name
          break
  else:
    #Local
    first = 'l'
    if var_name[0] != first:
      var_name = first + var_name
  
  return var_name


def check_expression(line, helper):
  # Check expression syntax
  
  # Take out string literals out from the expression and replace them with placeholders "__0__"
  string_literals = []
  line_left = line
  line = ''
  match = re.search(RE.STRING, line_left, RE.FLAGS)
  while match:
    line += line_left[:match.start()]
    line += '"__%i__"' % (len(string_literals))
    string_literals.append(match.group(0))
    line_left = line_left[match.end():]
    match = re.search(RE.VAR, line_left, RE.FLAGS)
  line += line_left
  
  # Format operators
  line = line.replace('==', '=')
  line = line.replace('!=', '<>')
  line = line.replace('!', ' NOT ')
  line = line.replace('&&', ' AND ')
  line = line.replace('&', ' AND ')
  line = line.replace('||', ' OR ')
  line = line.replace('|', ' OR ')
  
  # Format boolean literals
  line = line.replace('False', '0')
  line = line.replace('True', '1')
  
  # Signals
  exp = 'IN\[(%s)\]' % (RE.INT)
  match = re.search(exp, line, RE.FLAGS)
  while match:
    line = line.replace(match.group(0), 'DI[%s]' % (match.group(1)))
    match = re.search(exp, line, RE.FLAGS)
  
  # Check variables
  line_left = line
  line = ''
  match = re.search(RE.VAR, line_left, RE.FLAGS)
  while match:
    line += line_left[:match.start()]
    var_name = match.group(0)
    if not var_name in helper.keywords:
      var_name = check_var_name(var_name, helper)
    line += var_name
    line_left = line_left[match.end():]
    match = re.search(RE.VAR, line_left, RE.FLAGS)
  line += line_left
  
  # Whitespace trims
  line = line.replace('  ', ' ')
  line = line.strip().rstrip()
  
  # Put string literals back to line
  for i, s in enumerate(string_literals):
    line = line.replace('"__%i__"' % (i), s)
  
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


def get_base_index(base_name, helper):
  index = -1
  if not base_name:
    index = 0
  for i,b in enumerate(helper.controller.Bases):
    if b.Name == base_name:
      index = i+1
  if not index in helper.used_base_indices:
    helper.used_base_indices.append(index)
  return index


def get_tool_index(tool_name, helper):
  index = -1
  if not tool_name:
    index = 0
  else:
    for i,t in enumerate(helper.controller.Tools):
      if t.Name == tool_name:
        index = i+1
  if not index in helper.used_tool_indices:
    helper.used_tool_indices.append(index)
  return index


def get_base(base_name, helper):
  for b in helper.controller.Bases:
    if b.Name == base_name:
      return b
  return None


def get_tool(tool_name, helper):
  for t in helper.controller.Tools:
    if t.Name == tool_name:
      return t
  return None


def get_base_in_reference(base, helper):
  #Convert base matrix to robot world
  if not base:
    return vcMatrix.new()
  if not base.Node:
    return base.PositionMatrix
  ref_m = helper.motiontarget.getSimWorldToRobotWorld()
  ref_m.invert()
  return ref_m * base.Node.WorldPositionMatrix * base.PositionMatrix


def get_tool_in_reference(tool, helper):
  #Convert tool matrix to robot flange
  if not tool:
    return vcMatrix.new()
  if tool.Node and tool.Node != helper.controller.FlangeNode:
    ref_m = helper.controller.FlangeNode.WorldPositionMatrix
    ref_m.invert()
    tool_m = tool.Node.WorldPositionMatrix * tool.PositionMatrix
    tool_m_in_ref = ref_m * tool_m
  else:
    tool_m_in_ref = tool.PositionMatrix
  return tool_m_in_ref

