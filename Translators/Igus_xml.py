# Template post processor

from vcCommand import *
import os.path
import sys
import vcMatrix, vcVector, math
import locale
locale.setlocale(locale.LC_NUMERIC,'C')
import xml.etree.ElementTree as ET

class TranslatorHelper:
  # Capsulates commonly used handles and post processor settings. Also caches translated lines.
  
  def __init__(self, app, program, uri):
    
    # 
    self.data =    '' # Translated program lines
    
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
    #self.MyBoolean = self.command.getProperty('MyBoolean').Value
    #self.MyReal = self.command.getProperty('MyReal').Value
    
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
    self.xml_root_name = ET.Element('Program')
    self.line_number = 1
    self.routine_variables = []
  
  
  def get_uri_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  

def prettify(element, indent='  '):
    queue = [(0, element)]  # (level, element)
    while queue:
        level, element = queue.pop(0)
        children = [(level + 1, child) for child in list(element)]
        if children:
            element.text = '\n' + indent * (level+1)  # for child open
        if queue:
            element.tail = '\n' + indent * queue[0][0]  # for sibling open
        else:
            element.tail = '\n' + indent * (level-1)  # for parent close
        queue[0:0] = children  # prepend so children come before siblings

def postProcess(app, program, uri):
  # Entry point
  helper = TranslatorHelper(app, program, uri)
  define_statement_writers(helper)
  
  write_frames(helper)
  
  write_routine(helper, helper.program.MainRoutine)
  prettify(helper.xml_root_name)
  tree = ET.ElementTree(helper.xml_root_name)
  tree.write(uri,encoding='UTF-8',xml_declaration=True)
  helper.xml_root_name.clear()
  for routine in helper.program.Routines:
    helper.line_number = 1
    write_routine(helper, routine)
    sub_routine_uri = get_subroutine_path(helper, routine.Name)
    prettify(helper.xml_root_name)
    tree = ET.ElementTree(helper.xml_root_name)
    tree.write(sub_routine_uri,encoding='UTF-8',xml_declaration=True)
    helper.xml_root_name.clear()
    helper.line_number = 1
  
  
  
  return True, [uri]


def getProperties():
  # Properties for action panel. Remove this if you don't want any settings.
  
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  #props.append((VC_BOOLEAN, 'MyBoolean', False, None, None, 0, 0))
  #props.append((VC_REAL, 'MyReal', 100.0, None, None, 0, 0))
  return props


def print_warning(line):
  # Wrapper to print warning on console
  print 'WARNING: %s' % (line)


def print_error(line):
  # Wrapper to print error on console
  print 'ERROR: %s' % (line)


def write_frames(helper):
  # Find out all base and tool frames which are in use and write them in the job
  statements = get_all_statements_on_program(helper, helper.program)
  
  bases = []
  tools = []
  
  for statement in statements:
    if not statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      continue
    if statement.Base and not statement.Base in bases:
      bases.append(statement.Base)
    if statement.Tool and not statement.Tool in tools:
      tools.append(statement.Tool)
  



def write_routine(helper, routine):

  
  # Routine variables
  var_types = { VC_REAL:'number' }
  if routine.Properties:
    for prop in routine.Properties:
      if prop.Name in ['Name'] or not prop.Type in var_types:
        continue
      print_warning('All the routine variables on Igus side are global variables. This post processor does not support global variables, '
                  'nor does it support any component properties if being used in the statements. '
                  'Consider using variables with different names and try again.')
    
      print('*************************************************************************************')
      print_warning('The only supported variable on Igus side that VC has is VC_REAL; Boolean, Integer, and String are not supported.')
      print('*************************************************************************************')
      if '_' in prop.Name:
        print_error("Consider removing the '_' from the variables' names and try again!")
        sys.exit()
      ET.SubElement(helper.xml_root_name,'Store',Nr=str(helper.line_number),Type='number',Value='0',Name=str(prop.Name),Descr="")
      helper.line_number += 1
  
  # Statements
  for statement in routine.Statements:
    write_statement(helper, statement)


def define_statement_writers(helper):
  # Define statement writers based on app version and store them in helper object
  statement_writers = {
    VC_STATEMENT_BREAK:unhandled,
    VC_STATEMENT_CALL:write_call_sub_routine,
    VC_STATEMENT_COMMENT:write_comment,
    VC_STATEMENT_CONTINUE:unhandled,
    VC_STATEMENT_DEFINE_BASE:unhandled,
    VC_STATEMENT_DEFINE_TOOL:unhandled,
    VC_STATEMENT_DELAY:write_delay,
    VC_STATEMENT_HALT:write_halt,
    VC_STATEMENT_IF:write_if,
    VC_STATEMENT_LINMOTION:write_lin,
    VC_STATEMENT_PATH:write_path,
    VC_STATEMENT_PRINT:unhandled,
    VC_STATEMENT_PROG_SYNC:unhandled,
    VC_STATEMENT_PTPMOTION:write_ptp,
    VC_STATEMENT_RETURN:unhandled, #write_return
    VC_STATEMENT_SETBIN:write_set_bin,
    VC_STATEMENT_SETPROPERTY:write_set_property,
    VC_STATEMENT_WAITBIN:write_wait_bin,
    VC_STATEMENT_WHILE:write_while }
  
  if helper.app_version >= 4.4:
    statement_writers[VC_STATEMENT_SETROBOTSTATISTICSSTATE] = unhandled
    statement_writers[VC_STATEMENT_SWITCHCASE] = unhandled
  
  helper.statement_writers = statement_writers


def write_statement(helper, statement):
  if not statement.Type in helper.statement_writers:
    return
  helper.statement_writers[statement.Type](helper, statement)


def write_call_sub_routine(helper, statement):
  if not statement.Routine:
    print_error("Select the routine name to be called and try again!")
    sys.exit()
  else:
    sub_routine_uri = get_subroutine_path(helper, statement.Routine.Name)
    ET.SubElement(helper.xml_root_name,'Sub',Nr=str(helper.line_number),File=sub_routine_uri,Descr="")
    helper.line_number += 1


def write_comment(helper, statement):
  ET.SubElement(helper.xml_root_name,'Comment',Nr=str(helper.line_number),Descr=str(statement.Comment))
  helper.line_number += 1


def write_delay(helper, statement):
  ET.SubElement(helper.xml_root_name,'Wait',Nr=str(helper.line_number),Type='Time',Seconds=str(statement.Delay),Descr="")
  helper.line_number += 1


def write_halt(helper, statement):
  if statement.getProperty('HaltSimulation').Value == True:
    ET.SubElement(helper.xml_root_name,'Pause',Nr=str(helper.line_number),Descr="")
  elif statement.getProperty('ResetSimulation').Value == True:
    ET.SubElement(helper.xml_root_name,'Stop',Nr=str(helper.line_number),Descr="")
  elif statement.getProperty('ResetSimulation').Value == True and statement.getProperty('HaltSimulation').Value == True:
    ET.SubElement(helper.xml_root_name,'Stop',Nr=str(helper.line_number),Descr="")
  helper.line_number += 1

def write_if(helper, statement):
  ET.SubElement(helper.xml_root_name,'If',Nr=str(helper.line_number),Condition=str(statement.Condition),Descr="")
  helper.line_number += 1
  for child in statement.ThenScope.Statements:
    write_statement(helper, child)
  if helper.app_version >= 4.4:
    if statement.ElseIfScopes:
      #continue
      #for elseifscope in statement.ElseIfScopes:
      #  ET.SubElement(helper.xml_root_name,'If',Nr=str(helper.line_number),Condition=str(statement.Condition),Descr="")
      #  helper.line_number += 1
      #  for child in elseifscope.Statements:
      #    write_statement(helper, child)
      print_warning('On Igus side, the ElseIf is not supported!')


  ET.SubElement(helper.xml_root_name,'Else',Nr=str(helper.line_number),Descr="")
  helper.line_number += 1
  for child in statement.ElseScope.Statements:
    write_statement(helper, child)
  if helper.app_version >= 4.4:
    if statement.ElseIfScopes:
      #continue
      #for elseifscope in statement.ElseIfScopes:
      #  ET.SubElement(helper.xml_root_name,'If',Nr=str(helper.line_number),Condition=str(statement.Condition),Descr="")
      #  helper.line_number += 1
      #  for child in elseifscope.Statements:
      #    write_statement(helper, child)
      print_warning('On Igus side, the ElseIf is not supported!')
  ET.SubElement(helper.xml_root_name,'EndIf',Nr=str(helper.line_number),Descr="")



def write_lin(helper, statement):
  statement.writeToTarget(helper.motiontarget)
  
  # Target
  #target_position,target_orientation = get_target_values(statement.Positions[0].PositionInReference)
  target_matrix = statement.Positions[0].PositionInReference
  #q = target_matrix.getQuaternion()
  
  # Other params
  speed = str(statement.MaxSpeed)
  acceleration = str(statement.Acceleration)
  smoothing = str(helper.motiontarget.AccuracyValue)

  if helper.motiontarget.BaseName:
    target_matrix = helper.motiontarget.BaseMatrix*statement.Positions[0].PositionInReference
  ET.SubElement(helper.xml_root_name,'Linear',AbortCondition='False',\
                Nr=str(helper.line_number),Source='Numerical',vel=speed,\
                acc=acceleration,smooth=smoothing,x=str(target_matrix.P.X),y=str(target_matrix.P.Y),z=str(target_matrix.P.Z),\
                a=str(target_matrix.O.X),b=str(target_matrix.O.Y),c=str(target_matrix.O.Z),e1="0",e2="0",e3="0",Descr="")
  helper.line_number += 1


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
  point_count = statement.getSchemaSize()
  ej = statement.ExternalJointCount 
  for i in range(point_count):
    helper.motiontarget.Target = statement.getSchemaValue(i, 'Position')
    helper.motiontarget.CartesianSpeed = statement.getSchemaValue(i, 'MaxSpeed')
    helper.motiontarget.AccuracyMethod = statement.getSchemaValue(i, 'AccuracyMethod')
    helper.motiontarget.AccuracyValue = statement.getSchemaValue(i, 'AccuracyValue')
    
    # Target
    #target_position = get_target_postion_vector(helper.motiontarget.Target)
    target_matrix = helper.motiontarget.Target
    
    # Other params
    speed = str(helper.motiontarget.CartesianSpeed)
    acceleration = str(statement.getSchemaValue(i, 'Acceleration'))
    smoothing = str(helper.motiontarget.AccuracyValue)
    if helper.motiontarget.BaseName:
      #target_position = get_target_postion_vector(helper.motiontarget.BaseMatrix*statement.Positions[0].PositionInReference)
      target_matrix = helper.motiontarget.BaseMatrix*statement.Positions[0].PositionInReference
    ET.SubElement(helper.xml_root_name,'Linear',AbortCondition='False',\
                  Nr=str(helper.line_number),Source='Numerical',vel=speed,\
                  acc=acceleration,smooth=smoothing,x=str(target_matrix.P.X),y=str(target_matrix.P.Y),z=str(target_matrix.P.Z),\
                  a=str(target_matrix.O.X),b=str(target_matrix.O.Y),c=str(target_matrix.O.Z),e1="0",e2="0",e3="0",Descr="")
    helper.line_number += 1


def write_ptp(helper, statement):
  statement.writeToTarget(helper.motiontarget)
  joint_values = []
  for joint_index in range(0,len(helper.motiontarget.JointValues)):
    joint_values.append(helper.motiontarget.JointValues[joint_index])
  while joint_index + 1 <= 5:
    joint_values.append(0)
    joint_index += 1
  for joint_index in range(0,len(joint_values)):
    joint_values[joint_index] = str(joint_values[joint_index])
  # Target
  #target_position = get_target_postion_vector(statement.Positions[0].PositionInReference)
  #target_matrix = statement.Positions[0].PositionInReference

  # Other params
  velocity_percentage = str(statement.JointSpeed*100.0)
  acceleration = str(statement.JointForce*100.0)
  smoothing = str(helper.motiontarget.AccuracyValue)

  #if helper.motiontarget.BaseName:
  #  #target_position = get_target_postion_vector(helper.motiontarget.BaseMatrix*statement.Positions[0].PositionInReference)
  #  target_matrix = helper.motiontarget.BaseMatrix*statement.Positions[0].PositionInReference
  ET.SubElement(helper.xml_root_name,'Joint',AbortCondition='False',\
                Nr=str(helper.line_number),Source='Numerical',velPercent=velocity_percentage,\
                acc=acceleration,smooth=smoothing,a1=joint_values[0],a2=joint_values[1],a3=joint_values[2],\
                a4=joint_values[3],a5=joint_values[4],a6=joint_values[5],e1="0",e2="0",e3="0",Descr="")
  helper.line_number += 1


def write_set_bin(helper, statement):
  ET.SubElement(helper.xml_root_name,'Output',Nr=str(helper.line_number),Channel="DOut"+str(statement.OutputPort),\
                State=str(statement.OutputValue),Descr="")
  helper.line_number += 1


def write_set_property(helper, statement):
  #if statement.TargetProperty.Type ==
  function,second_operand = get_math_operator_elements(statement.ValueExpression)
  ET.SubElement(helper.xml_root_name,'MathOperator',Nr=str(helper.line_number),Function=function,\
                FirstOperand=str(statement.TargetProperty),SecondOperand=second_operand,Descr="")
  helper.line_number += 1

def write_wait_bin(helper, statement):
  ET.SubElement(helper.xml_root_name,'Wait',Nr=str(helper.line_number),Type="Conditional",\
                Condition="Din"+str(statement.InputPort),Descr="")
  helper.line_number += 1


def write_while(helper, statement):
  condition = check_expression(statement.Condition)
  ET.SubElement(helper.xml_root_name,'Loop',Nr=str(helper.line_number),Mode='DIn',\
                Condition=condition,Descr="")
  helper.line_number += 1
  for child in statement.Scope.Statements:
    write_statement(helper, child)
  ET.SubElement(helper.xml_root_name,'EndLoop',Nr=str(helper.line_number),Descr="")


def unhandled(helper, statement):
  print_warning('Unsupported statement type %s.' % (statement.Type))


def get_all_statements_on_program(helper, program):
  statements = []
  routines = [program.MainRoutine]
  routines.extend(program.Routines)
  for routine in routines:
    statements.extend(get_all_statements(helper, routine))
  return statements


def get_all_statements(helper, scope):
  statements = []
  for s in scope.Statements:
    statements.append(s)
    if s.Type == VC_STATEMENT_IF:
      statements.extend(get_all_statements(helper, s.ThenScope))
      if helper.app_version >= 4.4:
        for elseifscope in s.ElseIfScopes:
          statements.extend(get_all_statements(helper, elseifscope))
      statements.extend(get_all_statements(helper, s.ElseScope))
    elif s.Type == VC_STATEMENT_WHILE:
      statements.extend(get_all_statements(helper, s.Scope))
    elif helper.app_version >= 4.4 and s.Type == VC_STATEMENT_SWITCHCASE:
      for case in s.Cases:
        statements.extend(get_all_statements(helper, case))
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
  
  
def get_target_values(m):
  position_vector = [str(m.P.X),str(m.P.Y),str(m.P.Z)]
  #q = m.getQuaternion()
  #orientation_values = [str(q.X),str(q.Y),str(q.Z),str(q.W)]
  orientation_values = [str(m.O.X),str(m.O.Y),str(m.O.Z)]
  return position_vector,orientation_values


def check_expression(exp):
  # Check expression syntax
  exp = exp.strip()
  
  # Format boolean literals
  #exp = exp.replace('False', 'FALSE')
  #exp = exp.replace('false', 'FALSE')
  #exp = exp.replace('True', 'TRUE')
  #exp = exp.replace('true', 'TRUE')
  
  # Format operators
  #exp = exp.replace('!=', '<>')
  #exp = exp.replace('&&', ' AND ')
  #exp = exp.replace('&', ' AND ')
  #exp = exp.replace('||', ' OR ')
  #exp = exp.replace('|', ' OR ')
  #exp = exp.replace('!', ' NOT ')
  
  #exp = exp.replace('  ', ' ')
  #myNrVar&gt;0
  #myNrVar&lt;0
  #myNrVar=0
  #myNrVar&gt;=0
  #myNrVar&lt;=0
  #exp = exp.replace('>', '&gt;')
  #exp = exp.replace('>', '&lt;')
  exp = exp.replace('=', '=')
  exp = exp.replace('==', '=')
  #exp = exp.replace('>=', '&gt;=')
  #exp = exp.replace('<=', '&lt;=')
  
  return exp
  

def get_subroutine_path(helper, subroutine_name):
  path = helper.uri.split(os.sep)
  path[-1] = subroutine_name + '.xml'
  path = '/'.join(path)
  return path


def get_math_operator_elements(value_expression):
  #value_expression = value_expression.strip()
  second_operand_string = ''
  if '+' in value_expression:
    function_string = 'add'
    sub_strings = value_expression.split('+')
    second_operand_string = sub_strings[1]
  elif '-' in value_expression:
    function_string = 'subtract'
    sub_strings = value_expression.split('-')
    second_operand_string = sub_strings[1]
  elif '*' in value_expression:
    function_string = 'multiply'
    sub_strings = value_expression.split('*')
    second_operand_string = sub_strings[1]
  elif '/' in value_expression:
    function_string = 'divide'
    sub_strings = value_expression.split('/')
    second_operand_string = sub_strings[1]
  else:
    function_string = 'set'
    second_operand_string = value_expression
  return function_string,second_operand_string