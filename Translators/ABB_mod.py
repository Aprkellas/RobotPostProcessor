#-------------------------------------------------------------------------------
# version 1.12
#-------------------------------------------------------------------------------
from math import e
from vcCommand import *
import os.path
import re
import vcMatrix, vcVector
import locale
locale.setlocale(locale.LC_NUMERIC,'C')


class TranslatorHelper:
  # Capsulates commonly used handles and post processor settings. Also caches translated lines.
  
  def __init__(self, app, program, uri):
    
    # 
    self.data =    ''         # Translated program lines
    self.positions = ''       # Translated robot positions
    self.eio = ''             # Signal configuration file contents
    self.position_names = []
    self.used_inputs = []
    self.used_outputs = []
    
    # Helpers
    self.app = app
    self.uri = uri
    self.program = program
    
    self.app_version = 4.0
    try:
      self.app_version = float(app.ProductVersion[:app.ProductVersion.rfind('.')])
    except:
      pass
    
    # Properties
    self.command = getCommand()
    
    # Robot
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.motiontarget = self.controller.createTarget()
    
    self.robot_joint_count = len([x for x in self.controller.Joints if not x.ExternalController])
    self.total_joint_count = len(self.controller.Joints)
    self.external_joint_count = self.total_joint_count - self.robot_joint_count
    
    self.indentation = '  '
    self.indent_depth = 0
    
    self.statement_writers = {}
  
  def write_line(self, input):
    self.data += ('%s%s\n' % (self.current_indent(), input))
  
  def write_position(self, input):
    self.positions += ('%s%s\n' % (self.indentation, input))
  
  def write_eio(self, input):
    self.eio += ('%s\n' % (input))
  
  def current_indent(self):
    return self.indentation * self.indent_depth
  
  def get_uri_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  
  def get_job_name(self):
    head, tail = os.path.split(self.uri)
    name = tail[:tail.rfind('.')]
    return name
  
  def get_eio_name(self):
    head, tail = os.path.split(self.uri)
    name = os.path.join(head, 'EIO.cfg')
    return name


class RE(object):
  #Capsulates RE constants
  
  ANY = r'.+'
  ANYQ = r'.*'
  SPC = r'\s+'
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


def postProcess(app, program, uri):
  # Entry point
  
  helper = TranslatorHelper(app, program, uri)
  define_statement_writers(helper)
  
  write_job_header(helper)
  write_frames(helper)
  write_global_variables(helper)
  write_positions(helper, True)
  
  write_routine(helper, helper.program.MainRoutine)
  for routine in helper.program.Routines:
    write_routine(helper, routine)
  
  write_positions(helper, False)
  write_job_footer(helper)
  
  write_eio(helper)
  
  files = []
  with open(helper.uri, 'w') as file:
    file.write(helper.data)
    files.append(uri)
  
  if helper.eio:
    eio_uri = helper.get_eio_name()
    with open(eio_uri, 'w') as file:
      file.write(helper.eio)
    files.append(eio_uri)
  
  return True, files


def getDefaultJobName():
  # Default job name for launcher. Remove this if you don't want to specify default name.
  return 'MainModule.mod'


def print_info(line):
  # Wrapper to print info on console
  print 'INFO: %s' % (line)


def print_warning(line):
  # Wrapper to print warning on console
  print 'WARNING: %s' % (line)


def print_error(line):
  # Wrapper to print error on console
  print 'ERROR: %s' % (line)


def write_job_header(helper):
  helper.write_line('MODULE %s' % (helper.get_job_name()))
  helper.write_line('')
  helper.indent_depth += 1


def write_job_footer(helper):
  helper.indent_depth -= 1
  helper.write_line('ENDMODULE')


def write_eio(helper):
  if not helper.used_inputs and not helper.used_outputs:
    helper.eio = ''
    return
  
  helper.used_inputs.sort()
  helper.used_outputs.sort()
  
  # Header
  helper.eio = ''
  helper.write_eio('EIO:CFG_1.0:6:1::')
  helper.write_eio('#')
  helper.write_eio('EIO_SIGNAL:')
  helper.write_eio('')
  
  for input in helper.used_inputs:
    helper.write_eio('      -Name "di%i" -SignalType "DI" -Access "All"' % (input))
    helper.write_eio('')
  
  for output in helper.used_outputs:
    helper.write_eio('      -Name "do%i" -SignalType "DO" -Access "All"' % (output))
    helper.write_eio('')


def write_frames(helper):
  # Find out all base and tool frames which are in use and write them in the job
  statements = get_all_statements_on_program(helper.app_version, helper.program)
  
  bases = []
  tools = []
  
  for statement in statements:
    if not statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      continue
    if statement.Base and not statement.Base in bases:
      bases.append(statement.Base)
    if statement.Tool and not statement.Tool in tools:
      tools.append(statement.Tool)
  
  robot_world_pos = helper.motiontarget.getSimWorldToRobotWorld()
  
  for base in bases:
    name = base.Name
    bm = get_base_matrix(helper.controller, base, robot_world_pos)
    uf = '[%s]' % (matrix_to_string(bm))
    of = '[%s]' % (matrix_to_string(vcMatrix.new()))
    robhold = 'FALSE'
    if base_is_attached_to_robot_flange(helper.controller, base):
      robhold = 'TRUE'
    ufprog = 'TRUE'
    ufmec = '""'
    positioner = get_base_positioner_controller(helper.controller, base)
    if positioner:
      ufprog = 'FALSE'
      ufmec = '"%s"' % (positioner.Component.Name)
      uf, of = of, uf
    line = 'PERS wobjdata %s:=[%s,%s,%s,%s,%s];' % (name, robhold, ufprog, ufmec, uf, of)
    helper.write_line(line)
  if bases:
    helper.write_line('')
  
  for tool in tools:
    name = tool.Name
    tm = get_tool_matrix(helper.controller, tool, robot_world_pos)
    tf = '[%s]' % (matrix_to_string(tm))
    robhold = 'TRUE'
    if not tool_is_attached_to_robot_flange(helper.controller, tool):
      robhold = 'FALSE'
    load = '[5,[0,0,0.001],[1,0,0,0],0,0,0]'
    line = 'PERS tooldata %s:=[%s,%s,%s];' % (name, robhold, tf, load)
    helper.write_line(line)
  if tools:
    helper.write_line('')


def write_global_variables(helper):
  # Find out if there are component properties used in assign statements. Declare them as global variables.
  statements = get_all_statements_on_program(helper.app_version, helper.program)
  
  props = []
  prop_names = []
  var_types = { VC_BOOLEAN:'bool', VC_INTEGER:'num', VC_REAL:'num', VC_STRING:'string'}
  for statement in statements:
    if statement.Type != VC_STATEMENT_SETPROPERTY:
      continue
    var_name = statement.TargetProperty.strip()
    is_local = False
    for prop in statement.ParentRoutine.Properties:
      if prop.Name == var_name:
        is_local = True
        break
    if is_local:
      continue
    prop = helper.component.getProperty(var_name)
    if prop and not prop.Name in prop_names and prop.Type in var_types:
      props.append(prop)
      prop_names.append(prop.Name)
  
  
  for prop in props:
    var_type = var_types[prop.Type]
    value = str(prop.Value)
    if prop.Type == VC_STRING:
      value = '"%s"' % (value)
    helper.write_line('VAR %s %s := %s;' % (var_type, prop.Name, value))
  if props:
    helper.write_line('')


def write_positions(helper, init):
  # Write positions to header. 
  # If init is True only write place holder. Otherwise replace placeholder with actual positions.
  
  if init:
    helper.indent_depth -= 1
    helper.write_line('%POSITIONS%')
    helper.indent_depth += 1
    helper.write_line('')
  else:
    if helper.positions:
      helper.positions = helper.positions[:-1]    # Remove last new line
    helper.data = helper.data.replace('%POSITIONS%', helper.positions)


def write_routine(helper, routine):
  helper.write_line('PROC %s()' % (routine.Name))
  helper.indent_depth += 1
  
  # Routine variables
  vars_exist = False
  var_types = { VC_BOOLEAN:'bool', VC_INTEGER:'num', VC_REAL:'num', VC_STRING:'string'}
  for prop in routine.Properties:
    if prop.Name in ['Name'] or not prop.Type in var_types:
      continue
    var_type = var_types[prop.Type]
    value = str(prop.Value)
    if prop.Type == VC_STRING:
      value = '"%s"' % (value)
    helper.write_line('VAR %s %s := %s;' % (var_type, prop.Name, value))
    vars_exist = True
  if vars_exist:
    helper.write_line('')
  
  # Statements
  for statement in routine.Statements:
    write_statement(helper, statement)
  
  helper.indent_depth -= 1
  helper.write_line('ENDPROC')
  helper.write_line('')


def define_statement_writers(helper):
  # Define statement writers based on app version and store them in helper object
  
  statement_writers = {
    VC_STATEMENT_BREAK:unhandled,
    VC_STATEMENT_CALL:write_call,
    VC_STATEMENT_COMMENT:write_comment,
    VC_STATEMENT_CONTINUE:unhandled,
    VC_STATEMENT_DEFINE_BASE:write_define_base,
    VC_STATEMENT_DEFINE_TOOL:write_define_tool,
    VC_STATEMENT_DELAY:write_delay,
    VC_STATEMENT_HALT:write_halt,
    VC_STATEMENT_IF:write_if,
    VC_STATEMENT_LINMOTION:write_lin,
    VC_STATEMENT_PATH:write_path,
    VC_STATEMENT_PRINT:write_print,
    VC_STATEMENT_PROG_SYNC:unhandled,
    VC_STATEMENT_PTPMOTION:write_ptp,
    VC_STATEMENT_RETURN:write_return,
    VC_STATEMENT_SETBIN:write_set_bin,
    VC_STATEMENT_SETPROPERTY:write_set_property,
    VC_STATEMENT_WAITBIN:write_wait_bin,
    VC_STATEMENT_WHILE:write_while }
  
  if helper.app_version >= 4.4:
    statement_writers[VC_STATEMENT_SETROBOTSTATISTICSSTATE] = unhandled
    statement_writers[VC_STATEMENT_SWITCHCASE] = write_switch_case
  
  helper.statement_writers = statement_writers


def write_statement(helper, statement):
  if not statement.Type in helper.statement_writers:
    return
  helper.statement_writers[statement.Type](helper, statement)


def write_call(helper, statement):
  if statement.Routine:
    helper.write_line('%s;' % (statement.Routine.Name))


def write_comment(helper, statement):
  helper.write_line('!%s;' % (statement.Comment))


def write_define_base(helper, statement):  
  if not statement.Base:
    return
  
  name = statement.Base.Name
  if statement.IsRelative or not statement.Base or not statement.Node:
    m = vcMatrix.new(statement.Position)
  else:
    current_node = statement.Base.Node
    current_pos = statement.Base.PositionMatrix
    statement.Base.Node = statement.Node
    statement.Base.PositionMatrix = statement.Position
    m = get_base_matrix(helper.controller, statement.Base)
    statement.Base.Node = current_node
    statement.Base.PositionMatrix = current_pos
  
  s = matrix_to_string(m)
  if statement.IsRelative:
    helper.write_line('%s.uframe:=PoseMult(%s.uframe, [%s]);' % (name, name, s))
  else:
    helper.write_line('%s.uframe:=[%s];' % (name, s))


def write_define_tool(helper, statement):
  if not statement.Tool:
    return
  
  name = statement.Tool.Name
  if statement.IsRelative or not statement.Tool or not statement.Node:
    m = vcMatrix.new(statement.Position)
  else:
    current_node = statement.Tool.Node
    current_pos = statement.Tool.PositionMatrix
    statement.Tool.Node = statement.Node
    statement.Tool.PositionMatrix = statement.Position
    m = get_tool_matrix(helper.controller, statement.Tool)
    statement.Tool.Node = current_node
    statement.Tool.PositionMatrix = current_pos
  
  s = matrix_to_string(m)
  if statement.IsRelative:
    helper.write_line('%s.tframe:=PoseMult(%s.tframe, [%s]);' % (name, name, s))
  else:
    helper.write_line('%s.tframe:=[%s];' % (name, s))


def write_delay(helper, statement):
  helper.write_line('WaitTime %g;' % (statement.Delay))


def write_halt(helper, statement):
  helper.write_line('Stop;')


def write_if(helper, statement):
  condition = check_expression(statement.Condition)
  helper.write_line('IF %s THEN' % (condition))
  helper.indent_depth += 1
  for child in statement.ThenScope.Statements:
    write_statement(helper, child)
  helper.indent_depth -= 1
  
  if helper.app_version >= 4.4:
    for elseifscope in statement.ElseIfScopes:
      condition = check_expression(elseifscope.Condition)
      helper.write_line('ELSEIF %s THEN' % (condition))
      helper.indent_depth += 1
      for child in elseifscope.Statements:
        write_statement(helper, child)
      helper.indent_depth -= 1
  
  helper.write_line('ELSE')
  helper.indent_depth += 1
  for child in statement.ElseScope.Statements:
    write_statement(helper, child)
  helper.indent_depth -= 1
  
  helper.write_line('ENDIF')


def write_lin(helper, statement):
  statement.writeToTarget(helper.motiontarget)
  
  # Target
  pos_name = write_target(helper, helper.motiontarget, statement.Positions[0].Name)
  
  # Other params
  speed = speed_to_string(helper.motiontarget)
  zone = accuracy_to_string(helper.motiontarget)
  tool = tool_to_string(helper.motiontarget)
  wobj = ''
  if helper.motiontarget.BaseName:
    wobj = r'\WObj:=%s' % (helper.motiontarget.BaseName)
  
  line = 'movel %s,%s,%s,%s%s;' % (pos_name, speed, zone, tool, wobj)
  
  helper.write_line(line)


def write_path(helper, statement):
  
  # Set up motiontarget
  helper.motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  helper.motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  helper.motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR
  if statement.Base == None:
    helper.motiontarget.BaseName = ''
  else:
    helper.motiontarget.BaseName = statement.Base.Name
  if statement.Tool == None:
    helper.motiontarget.ToolName = ''
  else:
    helper.motiontarget.ToolName = statement.Tool.Name
  
  # Iterate schema points
  helper.write_line('!Path start')
  point_count = statement.getSchemaSize()
  ej = statement.ExternalJointCount    
  for i in range(point_count):
    helper.motiontarget.Target = statement.getSchemaValue(i, 'Position')
    helper.motiontarget.CartesianSpeed = statement.getSchemaValue(i, 'MaxSpeed')
    helper.motiontarget.AccuracyMethod = statement.getSchemaValue(i, 'AccuracyMethod')
    helper.motiontarget.AccuracyValue = statement.getSchemaValue(i, 'AccuracyValue')
    
    # Target
    pos_name = '%s_%i' % (statement.Name, (i + 1))
    pos_name = write_target(helper, helper.motiontarget, pos_name)
    
    # Other params
    speed = speed_to_string(helper.motiontarget)
    zone = accuracy_to_string(helper.motiontarget)
    tool = tool_to_string(helper.motiontarget)
    wobj = ''
    if helper.motiontarget.BaseName:
      wobj = r'\WObj:=%s' % (helper.motiontarget.BaseName)
    
    line = 'movel %s,%s,%s,%s%s;' % (pos_name, speed, zone, tool, wobj)
    
    helper.write_line(line)
  #endfor
  helper.write_line('!Path end')


def write_print(helper, statement):
  helper.write_line('TPWrite "%s";' % (statement.Message))


def write_ptp(helper, statement):
  statement.writeToTarget(helper.motiontarget)
  
  # Target
  pos_name = write_target(helper, helper.motiontarget, statement.Positions[0].Name)
  
  # Other params
  speed = speed_to_string(helper.motiontarget)
  zone = accuracy_to_string(helper.motiontarget)
  tool = tool_to_string(helper.motiontarget)
  wobj = ''
  if helper.motiontarget.BaseName:
    wobj = r'\WObj:=%s' % (helper.motiontarget.BaseName)
  
  line = 'movej %s,%s,%s,%s%s;' % (pos_name, speed, zone, tool, wobj)
  
  helper.write_line(line)


def write_return(helper, statement):
  helper.write_line('RETURN;')


def write_set_bin(helper, statement):
  helper.write_line('SetDO do%i,%i;' % (statement.OutputPort,statement.OutputValue))
  if not statement.OutputPort in helper.used_outputs:
    helper.used_outputs.append(statement.OutputPort)


def write_set_property(helper, statement):
  value = check_expression(statement.ValueExpression)
  helper.write_line('%s := %s;' % (statement.TargetProperty, value))


def write_switch_case(helper, statement):
  condition = check_expression(statement.Condition)
  helper.write_line('TEST %s' % (condition))
  
  for case in statement.Cases:
    condition = check_expression(case.CaseCondition)
    if condition.strip().lower() == 'default':
      helper.write_line('DEFAULT:')
    else:
      helper.write_line('CASE %s:' % (condition))
      
    helper.indent_depth += 1
    for child in case.Statements:
      write_statement(helper, child)
    helper.indent_depth -= 1
  
  helper.write_line('ENDTEST')


def write_wait_bin(helper, statement):
  helper.write_line('WaitDI di%i,%i;' % (statement.InputPort,statement.InputValue))
  if not statement.InputPort in helper.used_inputs:
    helper.used_inputs.append(statement.InputPort)


def write_while(helper, statement):
  condition = check_expression(statement.Condition)
  helper.write_line('WHILE %s DO' % (condition))
  helper.indent_depth += 1
  for child in statement.Scope.Statements:
    write_statement(helper, child)
  helper.indent_depth -= 1
  helper.write_line('ENDWHILE')


def unhandled(helper, statement):
  print_warning('Unsupported statement type %s.' % (statement.Type))


def write_target(helper, motiontarget, pos_name):
  # Write given motiontarget to helper.positions and return position name
  target_matrix = matrix_to_string(motiontarget.Target)
  conf = configuration_to_string(helper.motiontarget, helper.robot_joint_count)
  ex_axes = ex_axes_to_string(helper.motiontarget, helper.robot_joint_count, helper.external_joint_count)
  target = '[%s,%s,%s]' % (target_matrix, conf, ex_axes)
  pos_name = get_unique_position_name(helper, pos_name)
  position = 'PERS robtarget %s:=%s;' % (pos_name, target)
  helper.write_position(position)
  helper.position_names.append(pos_name)

  return pos_name


def get_unique_position_name(helper, name):
  # Get unique position name. Use given name if not in use, otherwise increment number on the name.
  if not name in helper.position_names:
    return name
  
  exp = '%s%s' % (RE.INT, RE.EOS)
  match = re.search(exp, name, RE.FLAGS)
  index = 0
  name_base = name
  if match:
    try:
      index = int(match.group(0))
    except:
      pass
    name_base = name_base[:match.start()]
  while index < 100000:
    name = '%s%i' % (name_base, index)
    if not name in helper.position_names:
      return name
    index += 1
  
  return name


def get_all_statements_on_program(app_version, program):
  statements = []
  routines = [program.MainRoutine]
  routines.extend(program.Routines)
  for routine in routines:
    statements.extend(get_all_statements(app_version, routine))
  return statements


def get_all_statements(app_version, scope):
  statements = []
  for s in scope.Statements:
    statements.append(s)
    if s.Type == VC_STATEMENT_IF:
      statements.extend(get_all_statements(app_version, s.ThenScope))
      if app_version >= 4.4:
        for elseifscope in s.ElseIfScopes:
          statements.extend(get_all_statements(app_version, elseifscope))
      statements.extend(get_all_statements(app_version, s.ElseScope))
    elif s.Type == VC_STATEMENT_WHILE:
      statements.extend(get_all_statements(app_version, s.Scope))
    elif app_version >= 4.4 and s.Type == VC_STATEMENT_SWITCHCASE:
      for case in s.Cases:
        statements.extend(get_all_statements(app_version, case))
  return statements


def get_base_matrix(controller, base, robot_world_pos = None):
  # Convert base matrix to reference coordinates. There are 3 cases:
  #   -Default case, base reference is robot world frame.
  #   -External TCP, base reference is robot flange frame.
  #   -Workpiece positioner, base reference is positioner's flange node.
  
  if not base.Node:
    # One of the default default cases. No parent node, already on robot world coordiantes.
    return base.PositionMatrix
  
  is_at_flange = base_is_attached_to_robot_flange(controller, base)
  positioner_ctr = get_base_positioner_controller(controller, base)
  
  if is_at_flange:
    # External TCP
    base_m = controller.FlangeNode.InverseWorldPositionMatrix * base.Node.WorldPositionMatrix * base.PositionMatrix
  elif positioner_ctr:
    # Workpiece positioner
    base_m = positioner_ctr.FlangeNode.InverseWorldPositionMatrix * base.Node.WorldPositionMatrix * base.PositionMatrix
  else:
    # Default
    if not robot_world_pos:
      motiontarget = controller.createTarget()
      robot_world_pos = motiontarget.getSimWorldToRobotWorld()
    robot_world_pos.invert()
    base_m = robot_world_pos * base.Node.WorldPositionMatrix * base.PositionMatrix
  
  return base_m


def base_is_attached_to_robot_flange(controller, base):
  # Check if base frame is attached to robot flange.
  if not base.Node:
    return False
  node = base.Node
  while node and node.Parent:
    if node == controller.FlangeNode:
      return True
    node = node.Parent
  return False


def get_base_positioner_controller(controller, base):
  # Check if base frame is attached to positioner (servo controlled device that is not robot tool).
  # Return None if not and return positioner controller otherwise.
  if not base.Node:
    return None
  if base_is_attached_to_robot_flange(controller, base):
    return None
  comp = base.Node.Component
  while comp and comp.Parent:
    ctrs = comp.findBehavioursByType(VC_SERVOCONTROLLER)
    if not ctrs:
      ctrs = comp.findBehavioursByType(VC_ROBOTCONTROLLER)
    if ctrs and ctrs[0] != controller:
      return ctrs[0]
    comp = comp.Parent.Component
  return None


def get_tool_matrix(controller, tool, robot_world_pos = None):
  # Convert tool matrix to reference coordinates. There are 2 cases:
  #   -Default case, tool reference is robot flange frame.
  #   -External TCP, tool reference is robot world frame.
  
  if not tool.Node:
    # One of the default default cases. No parent node, already on robot flange coordiantes.
    return tool.PositionMatrix
  
  is_at_flange = tool_is_attached_to_robot_flange(controller, tool)
  
  if is_at_flange:
    # Default
    tool_m = controller.FlangeNode.InverseWorldPositionMatrix * tool.Node.WorldPositionMatrix * tool.PositionMatrix
  else:
    # External TCP
    if not robot_world_pos:
      motiontarget = controller.createTarget()
      robot_world_pos = motiontarget.getSimWorldToRobotWorld()
    robot_world_pos.invert()
    tool_m = robot_world_pos * tool.Node.WorldPositionMatrix * tool.PositionMatrix
  
  return tool_m


def tool_is_attached_to_robot_flange(controller, tool):
  # Check if tool frame is attached to robot flange.
  if not tool.Node:
    return True
  node = tool.Node
  while node and node.Parent:
    if node == controller.FlangeNode:
      return True
    node = node.Parent
  return False


def fzero(d):
  # Filter near zero to exact zero
  if abs(d) < 1e-6:
    return 0
  return d


def matrix_to_string(m):
  p = m.P
  q = m.getQuaternion()
  s = '[%g,%g,%g],[%g,%g,%g,%g]' % (fzero(p.X), fzero(p.Y), fzero(p.Z), fzero(q.X), fzero(q.Y), fzero(q.Z), fzero(q.W))
  return s


def configuration_to_string(motiontarget, robot_joint_count):
  joints = motiontarget.JointValues
  confx = motiontarget.RobotConfig
  conf1 = 0
  conf4 = 0
  conf6 = 0
  if robot_joint_count == 6:
    conf1 = int(joints[0] / 90.0)
    if joints[0] < 0.0:
      conf1 -= 1
    conf4 = int(joints[3] / 90.0)
    if joints[0] < 0.0:
      conf4 -= 1
    conf6 = int(joints[5] / 90.0)
    if joints[0] < 0.0:
      conf6 -= 1
  s = '[%g,%g,%g,%g]' % (conf1, conf4, conf6, confx)
  return s


def ex_axes_to_string(motiontarget, robot_joint_count, external_joint_count):
  exs = 6 * ['9e+09']
  for i in range(min(external_joint_count, 6)):
    exs[i] = '%g' % motiontarget.JointValues[robot_joint_count + i]
  s = '[%s,%s,%s,%s,%s,%s]' % (exs[0], exs[1], exs[2], exs[3], exs[4], exs[5])
  return s


def speed_to_string(motiontarget):
  v = motiontarget.CartesianSpeed
  if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
    if motiontarget.JointSpeedFactor < 1.0:
      v = motiontarget.JointSpeedFactor * 5000.0
    else:
      v = 5001.0
  if v > 5000.0:
    s = 'vmax'
  else:
    vrefs = [5000, 4000, 3000, 2500, 2000, 1500, 1000, 800, 600, 500, 400, 300, 200, 150, 100, 80, 60, 50, 40, 30, 20, 10]
    for vref in vrefs:
      if v >= vref:
        v = vref
        break
    else:
      v = 5
    s = 'v%.0f' % (v)
  return s


def accuracy_to_string(motiontarget):
  z = motiontarget.AccuracyValue 
  zrefs = [200, 150, 100, 80, 60, 50, 40, 30, 20, 10, 5, 1, 0.3]
  if z < zrefs[-1]:
    s = 'fine'
  else:
    for zref in zrefs:
      if z >= zref:
        z = zref
        break
    s = 'z%.0f' % (z)
  return s


def tool_to_string(motiontarget):
  if not motiontarget.ToolName:
    return 'tool0'
  return motiontarget.ToolName


def check_expression(exp):
  # Check expression syntax
  exp = exp.strip()
  
  # Format boolean literals
  exp = exp.replace('False', 'FALSE')
  exp = exp.replace('false', 'FALSE')
  exp = exp.replace('True', 'TRUE')
  exp = exp.replace('true', 'TRUE')
  
  # Format operators
  exp = exp.replace('==', '=')
  exp = exp.replace('!=', '<>')
  exp = exp.replace('&&', ' AND ')
  exp = exp.replace('&', ' AND ')
  exp = exp.replace('||', ' OR ')
  exp = exp.replace('|', ' OR ')
  exp = exp.replace('!', ' NOT ')
  
  exp = exp.replace('  ', ' ')
  
  return exp