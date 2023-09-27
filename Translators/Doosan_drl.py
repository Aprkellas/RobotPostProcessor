# Doosan script translator, Version 1.12
# Output file (.drl) is DRL script that you can import into your Doosan robot.

from vcCommand import *
import vcMatrix, os.path, math, re


class TranslatorHelper:
  def __init__(self, program):
  
    #Settings
    self.movel_as_joints = False
    
    #Helpers
    self.app_version = 4.0
    try:
      self.app_version = float(app.ProductVersion[:app.ProductVersion.rfind('.')])
    except:
      pass
    self.program = program
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.motiontarget = program.Executor.Controller.createTarget()
    self.active_frames_set = False
    self.active_base = None
    self.active_tool = None
    self.current_routine = None
    self.indentation = '  '
    self.depth = 1
    self.statement_translators = {
      VC_STATEMENT_BREAK:writeBreak,
      VC_STATEMENT_CALL:writeCall,
      VC_STATEMENT_COMMENT:writeComment,
      VC_STATEMENT_CONTINUE:writeContinue,
      VC_STATEMENT_DEFINE_BASE:writeDefineBase,
      VC_STATEMENT_DEFINE_TOOL:unknown,
      VC_STATEMENT_DELAY:writeDelay,
      VC_STATEMENT_HALT:writeHalt,
      VC_STATEMENT_IF:writeIf,
      VC_STATEMENT_LINMOTION:writeLinMotion,
      VC_STATEMENT_PATH:writePath,
      VC_STATEMENT_PRINT:writePrint,
      VC_STATEMENT_PROG_SYNC:unknown,
      VC_STATEMENT_PTPMOTION:writePtpMotion,
      VC_STATEMENT_RETURN:writeReturn,
      VC_STATEMENT_SETBIN:writeSetBin,
      VC_STATEMENT_SETPROPERTY:writeSetProperty,
      VC_STATEMENT_WAITBIN:writeWaitBin,
      VC_STATEMENT_WHILE:writeWhile }
    if self.app_version >= 4.4:
      self.statement_translators[VC_STATEMENT_SETROBOTSTATISTICSSTATE] = unknown
      self.statement_translators[VC_STATEMENT_SWITCHCASE] = writeSwitchCase
    
  def current_indent(self):
    return self.indentation*self.depth


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


def postProcess(appx, programx, uri):
  #Entry point
  global app, cmd, program
  
  app = appx
  cmd = getCommand()
  program = programx
  
  head, tail = os.path.split(uri)
  mainName = 'Main' # tail[:tail.rfind('.')]
  helper = TranslatorHelper(program)
  helper.movel_as_joints = cmd.getProperty('movel as joint values').Value
  
  # Check warnings
  checkWarnings(helper)
  
  #Write output file
  with open(uri,'w') as output_file:
    #Write header
    output_file.write('from DRCF import *\n')
    output_file.write('\n')
    
    #Write frames and globals
    writeFrames(output_file, helper)
    writeGlobals(output_file, helper)
    
    #Translate main and subroutine
    translateRoutine(program.MainRoutine, mainName, output_file, helper)
    for routine in program.Routines:
      translateRoutine(routine, routine.Name, output_file, helper)
    output_file.write('%s()\n' % mainName)
  
  return True,[uri]


def getProperties():
  #Properties for action panel
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  props.append((VC_BOOLEAN, 'movel as joint values', False, None, None, 0, 0))
  return props


def checkWarnings(helper):
  for j in helper.controller.Joints:
    if j.ExternalController:
      print 'Warning: External axes are not supported.'
      break
  
  statements = getAllStatementsOnProgram(helper.app_version, helper.program)
  ext_tcp = False
  for s in statements:
    if s.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      if s.ExternalTCP or s.Base and baseIsAttachedToRobotFlange(helper.controller, s.Base):
        ext_tcp = True
        break
  if ext_tcp:
    print 'Warning: External TCP motions are not supported.'


def translateRoutine(routine, name, output_file, helper):
  #Translate routine
  helper.current_routine = routine
  helper.active_frames_set = False
  output_file.write('def %s():\n' % name)
  
  #Routine globals
  statements = getAllStatements(helper.app_version, routine)
  for statement in statements:
    if statement.Type == VC_STATEMENT_DEFINE_BASE and statement.Base:
      output_file.write(helper.current_indent() + 'global %s\n' % statement.Base.Name)
  
  #Routine variables
  if len(routine.Properties) > 1:
    for p in routine.Properties:
      if p.Name in ['Name']:
        continue
      if p.Type in [VC_REAL, VC_INTEGER, VC_BOOLEAN]:
        output_file.write('%s%s = %s\n' % (helper.current_indent(), p.Name, str(p.Value)))
      if p.Type in [VC_STRING]:
        output_file.write('%s%s = "%s"\n' % (helper.current_indent(), p.Name, p.Value))
    output_file.write('\n')
  
  #Statements
  point_count = 0
  statement_count = 0
  for statement in routine.Statements:
    translator = helper.statement_translators.get(statement.Type, unknown)
    translator(output_file, statement, helper)
  output_file.write('#enddef %s\n\n' % name)
  output_file.write('\n')


def writeFrames(output_file, helper):
  bases = []
  tools = []
  all_stats = getAllStatementsOnProgram(helper.app_version, helper.program)
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
  
  robot_world_pos = helper.motiontarget.getSimWorldToRobotWorld()
  
  if not  helper.movel_as_joints:
    output_file.write('\n')
    output_file.write('# User frames (bases)\n')
    for b in bases:
      if not b:
        #No need to write down null base
        continue
      pos = getBaseMatrix(helper.controller, b, robot_world_pos)
      p = pos.P
      euler = pos.getEuler()
      posx = 'posx(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X)
      output_file.write('%s = set_user_cart_coord(%s, ref=DR_BASE)\n' % (b.Name, posx))
    
  output_file.write('\n')
  output_file.write('# Tool frames, make sure tcp coordinates (XYZABC) match the ones in controller!\n')
  for t in tools:
    name = 'TCP_0'
    pos = vcMatrix.new()
    if t:
      name = t.Name
      pos = getToolMatrix(helper.controller, t, robot_world_pos)
    p = pos.P
    euler = pos.getEuler()
    output_file.write('# %s = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n' % (name, p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X))
  output_file.write('\n')


def writeGlobals(output_file, helper):
  output_file.write('# Globals\n')
  output_file.write('set_accj(100)\n')
  output_file.write('\n')


def writeBreak(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'break\n')


def writeCall(output_file, statement, helper):
  if statement.getProperty('Routine').Value:
    output_file.write(helper.current_indent() + '%s()\n' % statement.getProperty('Routine').Value.Name )


def writeComment(output_file, statement, helper):
  output_file.write(helper.current_indent() + '# %s\n' % statement.Comment)


def writeContinue(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'continue\n')


def writeDefineBase(output_file, statement, helper):
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
    m = getBaseMatrix(helper.controller, statement.Base)
    statement.Base.Node = current_node
    statement.Base.PositionMatrix = current_pos
  
  if statement.IsRelative:
    p = m.P
    euler = m.getEuler()
    delta = 'posx(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X)
    pos0 = 'posx(0, 0, 0, 0, 0, 0)'
    output_file.write(helper.current_indent() + 'pos0 = %s\n' % (pos0))
    output_file.write(helper.current_indent() + 'delta = %s\n' % (delta))
    output_file.write(helper.current_indent() + 'pos1 = trans(pos0, delta, %s, DR_BASE)\n' % (name))
    output_file.write(helper.current_indent() + '%s = set_user_cart_coord(pos1, ref=DR_BASE)\n' % (name))
  else:
    p = m.P
    euler = m.getEuler()
    posx = 'posx(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X)
    output_file.write(helper.current_indent() + '%s = set_user_cart_coord(%s, ref=DR_BASE)\n' % (name, posx))
  output_file.write(helper.current_indent() + 'set_ref_coord(%s)\n' % name)


def writeDelay(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'wait(%.3f)\n' % statement.Delay)


def writeHalt(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'exit()\n')


def writeIf(output_file, statement, helper):
  condition = checkExpression(statement.Condition, helper)
  output_file.write(helper.current_indent() + 'if %s:\n' % condition)
  helper.depth += 1
  if not statement.ThenScope.Statements:
    output_file.write(helper.current_indent() + 'pass\n' )
  for s in statement.ThenScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(output_file, s, helper)
  helper.depth -= 1
  
  if helper.app_version >= 4.4:
    for elseifscope in statement.ElseIfScopes:
      condition = checkExpression(elseifscope.Condition, helper)
      output_file.write(helper.current_indent() + 'elif %s:\n' % condition)
      helper.depth += 1
      for s in elseifscope.Statements:
        translator = helper.statement_translators.get(s.Type, unknown)
        translator(output_file, s, helper)
      helper.depth -= 1
  
  output_file.write(helper.current_indent() + 'else:\n')
  helper.depth += 1
  if not statement.ElseScope.Statements:
    output_file.write(helper.current_indent() + 'pass\n' )
  for s in statement.ElseScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(output_file, s, helper)
  helper.depth -= 1
  output_file.write(helper.current_indent() + '#endif\n')


def writeLinMotion(output_file, statement, helper):
  writeActiveFrames(output_file, statement, helper)
  statement.writeToTarget(helper.motiontarget)
  if not helper.movel_as_joints:
    #Write as posx
    p = helper.motiontarget.Target.P
    euler = helper.motiontarget.Target.getEuler()
    pos = '[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]' % (p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X)
  else:
    #Write as fkin(posj)
    js = helper.motiontarget.JointValues
    pos = 'fkin(posj(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f), DR_BASE)' % (js[0], js[1], js[2], js[3], js[4], js[5])
  v = ', v=%.1f' % helper.motiontarget.CartesianSpeed
  a = ', a=%.1f' % helper.motiontarget.CartesianAcceleration
  r = ''
  if statement.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and statement.AccuracyValue > 0:
    r = ', r=%.1f' % statement.AccuracyValue
  output_file.write(helper.current_indent() + 'movel(%s%s%s%s)\n' % (pos, v, a, r))


def writePath(output_file, statement, helper):
  writeActiveFrames(output_file, statement, helper)
  output_file.write(helper.current_indent() + '#Start path: %s\n' % (statement.Name))
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
  for i in range(statement.getSchemaSize()):
    target = statement.getSchemaValue(i,'Position')
    helper.motiontarget.Target = target
    jv = helper.motiontarget.JointValues
    helper.motiontarget.JointValues = jv
    helper.motiontarget.CartesianSpeed = statement.getSchemaValue(i,'MaxSpeed')
    helper.motiontarget.CartesianAcceleration = statement.getSchemaValue(i,'Acceleration')
    helper.motiontarget.AccuracyMethod = statement.getSchemaValue(i,'AccuracyMethod')
    helper.motiontarget.AccuracyValue = statement.getSchemaValue(i,'AccuracyValue')
    
    if not helper.movel_as_joints:
      #Write as posx
      p = helper.motiontarget.Target.P
      euler = helper.motiontarget.Target.getEuler()
      pos = '[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]' % (p.X, p.Y, p.Z, euler.Z, euler.Y, euler.X)
    else:
      #Write as fkin(posj)
      js = helper.motiontarget.JointValues
      pos = 'fkin(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (js[0], js[1], js[2], js[3], js[4], js[5])
    v = ', v=%.1f' % helper.motiontarget.CartesianSpeed
    a = ', a=%.1f' % helper.motiontarget.CartesianAcceleration
    r = ''
    if helper.motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and helper.motiontarget.AccuracyValue > 0:
      r = ', r=%.1f' % helper.motiontarget.AccuracyValue
    output_file.write(helper.current_indent() + 'movel(%s%s%s%s)\n' % (pos, v, a, r))
    
  output_file.write(helper.current_indent() + '#End path: %s\n' % (statement.Name))


def writePrint(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'tp_log("%s")\n' % statement.Message)


def writePtpMotion(output_file, statement, helper):
  writeActiveFrames(output_file, statement, helper)
  statement.writeToTarget(helper.motiontarget)
  js = helper.motiontarget.JointValues
  pos = '[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]' % (js[0], js[1], js[2], js[3], js[4], js[5])
  v = ', v=%.1f' % (helper.motiontarget.JointSpeedFactor * helper.controller.Joints[0].MaxSpeed)  #Use slowest joint to determine speed
  r = ''
  if statement.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and statement.AccuracyValue > 0:
    r = ', r=%.1f' % statement.AccuracyValue
  output_file.write(helper.current_indent() + 'movej(%s%s%s)\n' % (pos, v, r))


def writeReturn(output_file, statement, helper):
  output_file.write(helper.current_indent() + 'return\n')


def writeSetBin(output_file, statement, helper):
  val = 'OFF'
  if statement.OutputValue:
    val = 'ON'
  output_file.write(helper.current_indent() + 'set_digital_output(%i, %s)\n' %(statement.OutputPort, val))


def writeSetProperty(output_file, statement, helper):
  value_expression = checkExpression(statement.ValueExpression, helper)
  output_file.write(helper.current_indent() + '%s = %s\n' % (statement.TargetProperty, value_expression))


def writeSwitchCase(output_file, statement, helper):
  test_condition = checkExpression(statement.Condition, helper)
  first = True
  
  for case in statement.Cases:
    case_condition = checkExpression(case.CaseCondition, helper)
    condition = '%s == %s' % (test_condition, case_condition)
    if case_condition.strip().lower() == 'default':
      output_file.write(helper.current_indent() + 'else:\n')
    elif first:
      output_file.write(helper.current_indent() + 'if %s:\n' % (condition))
      first = False
    else:
      output_file.write(helper.current_indent() + 'elif %s:\n' % (condition))
    
    helper.depth += 1
    for s in case.Statements:
      translator = helper.statement_translators.get(s.Type, unknown)
      translator(output_file, s, helper)
    helper.depth -= 1
  output_file.write(helper.current_indent() + '#endif\n')


def writeWaitBin(output_file, statement, helper):
  val = 'OFF'
  if statement.InputValue:
    val = 'ON'
  output_file.write(helper.current_indent() + 'wait_digital_input(%i, %s)\n' %(statement.InputPort, val))


def writeWhile(output_file, statement, helper):
  condition = checkExpression(statement.Condition, helper)
  output_file.write(helper.current_indent() + 'while %s:\n' % condition )
  helper.depth += 1
  if not statement.Scope.Statements:
    output_file.write(helper.current_indent() + 'pass\n' )
  for s in statement.Scope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(output_file, s, helper)
  helper.depth -= 1
  output_file.write(helper.current_indent() + '#endwhile\n')


def unknown(output_file, statement, helper):
  print '> Unsupported statement type skipped:', statement.Type


def getAllStatementsOnProgram(app_version, program):
  statements = []
  routines = [program.MainRoutine]
  routines.extend(program.Routines)
  for routine in routines:
    statements.extend(getAllStatements(app_version, routine))
  return statements


def getAllStatements(app_version, scope):
  #Return all statements in a scope (at any level) as a list
  statements = []
  for s in scope.Statements:
    statements.append(s)
    if s.Type == VC_STATEMENT_IF:
      statements.extend(getAllStatements(app_version, s.ThenScope))
      if app_version >= 4.4:
        for elseifscope in s.ElseIfScopes:
          statements.extend(getAllStatements(app_version, elseifscope))
      statements.extend(getAllStatements(app_version, s.ElseScope))
    elif s.Type == VC_STATEMENT_WHILE:
      statements.extend(getAllStatements(app_version, s.Scope))
    elif app_version >= 4.4 and s.Type == VC_STATEMENT_SWITCHCASE:
      for case in s.Cases:
        statements.extend(getAllStatements(app_version, case))
  return statements


def checkExpression(line, helper):
  #Check expression for formatting
  
  #Format operators
  line = line.replace('!', ' not ')
  line = line.replace('&&', ' and ')
  line = line.replace('&', ' and ')
  line = line.replace('||', ' or ')
  line = line.replace('|', ' or ')
  
  #Signals
  exp = 'IN\[(%s)\]' % (RE.INT)
  match = re.search(exp, line, RE.FLAGS)
  while match:
    line = line.replace(match.group(0), 'get_digital_input(%s)' % (match.group(1)))
    match = re.search(exp, line, RE.FLAGS)
  
  #Whitespace trims
  line = line.replace('  ', ' ')
  line = line.strip().rstrip()
  
  return line


def writeActiveFrames(output_file, statement, helper):
  
  if not  helper.movel_as_joints and (not helper.active_frames_set or statement.Base != helper.active_base):
    name = 'DR_BASE'
    if statement.Base:
      name = statement.Base.Name
    output_file.write(helper.current_indent() + 'set_ref_coord(%s)\n' % name )
    helper.active_base = statement.Base
  if not helper.active_frames_set or statement.Tool != helper.active_tool:
    name = 'TCP_0'
    if statement.Tool:
      name = statement.Tool.Name
    output_file.write(helper.current_indent() + 'set_tcp("%s")\n' % name )
    helper.active_tool = statement.Tool
    
  helper.active_frames_set = True


def getBaseMatrix(controller, base, robot_world_pos = None):
  # Convert base matrix to reference coordinates. There are 3 cases:
  #   -Default case, base reference is robot world frame.
  #   -External TCP, base reference is robot flange frame.
  #   -Workpiece positioner, base reference is positioner's flange node.
  
  if not base.Node:
    # One of the default default cases. No parent node, already on robot world coordiantes.
    return base.PositionMatrix
  
  is_at_flange = baseIsAttachedToRobotFlange(controller, base)
  positioner_ctr = getBasePositionerController(controller, base)
  
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


def baseIsAttachedToRobotFlange(controller, base):
  # Check if base frame is attached to robot flange.
  if not base.Node:
    return False
  node = base.Node
  while node and node.Parent:
    if node == controller.FlangeNode:
      return True
    node = node.Parent
  return False


def getBasePositionerController(controller, base):
  # Check if base frame is attached to positioner (servo controlled device that is not robot tool).
  # Return None if not and return positioner controller otherwise.
  if not base.Node:
    return None
  if baseIsAttachedToRobotFlange(controller, base):
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


def getToolMatrix(controller, tool, robot_world_pos = None):
  # Convert tool matrix to reference coordinates. There are 2 cases:
  #   -Default case, tool reference is robot flange frame.
  #   -External TCP, tool reference is robot world frame.
  
  if not tool.Node:
    # One of the default default cases. No parent node, already on robot flange coordiantes.
    return tool.PositionMatrix
  
  is_at_flange = toolIsAttachedToRobotFlange(controller, tool)
  
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


def toolIsAttachedToRobotFlange(controller, tool):
  # Check if tool frame is attached to robot flange.
  if not tool.Node:
    return True
  node = tool.Node
  while node and node.Parent:
    if node == controller.FlangeNode:
      return True
    node = node.Parent
  return False

