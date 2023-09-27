#Version 2.1 (12.01.2022)
# Convert to *.urp and *.script

#Current
  #DH parameters for e-series

# URP settings
  #1. Use Active TCP        - True  - Uses the Default TCP defined in Polyscope for all the motion statements
  #                         - False - Uses the motion statement TCP.
  #2. Hide Sub Program Tree - True  - Hide the statement tree in polyscope for the sub program
  #                         - False - Shows the statement tree in polyscope for the sub program
  #3. Generate URP          - True  - Create URP from VC

#TO DO IN POLYSCOPE
  #TOOL DEFINITION
  #  The TCP name used in VC should be defined in Polyscope under INSTALLATION > TCP 
  #  VC Tool > X, Y, Z    > TCP Position X, Y, Z 
  #  VC Tool > Rx, Ry, Rz > TCP Orientation Unit (RPY in degree)
  #  *NULL* TCP in VC will create 'Tool0' in MoveL and MoveJ. DEFINE THE TCP NAME as 'Tool0' IN POLYSCOPE UNDER INSTALLATION > TCP with position and orientation as '0'

  #SEQUENCE (routines):
  #During PostProcess, each subroutine in VC is created as *.urp file
  #Loading the *.urp(Main) file in polyscope, Call statement and SubProg with the (sequence)routine name is created
  #User should manually navigate to the *.urp(sequence) file from their file system and select it. Also assign it to Call statement.

#VC to URP Supported statements
  #1.MOVE (MoveL,MoveJ,MoveP)
  #2.Wait
  #3.Set and Type-"DigitalOutput"
  #4.Halt
  #5.Comment
  #6.Loop
  #7.SubProg
  #8.Assignment
  #9.If

#VC to URP Limitations
  #Assign statement:
    #Routine variable are NOT created in runtime. Should be assigned in Routine Properties
    #value expresions format should be: VariableName-Operator-Value, Eg:a+5
    #If routine variable name are similar, polyscope rename them. Therefore in VC have unique variable names in robot program
    #string concatenate not supported
  
  #SubRoutines
    #All the subroutines must be called from the program main routine using 'Call Sequence Statement'
    #SubRoutines SHOULD NOT have 'Call Sequence Statement'
 

#VC to URP NOT Supported statements - 
#1.Move - (MoveP with CircleMove)
#2.Return, Continue, Break


from vcCommand import *
import vcMatrix, os.path, math
from collections import OrderedDict
import xml.etree.ElementTree as ET
import gzip, shutil
import re

R2D = 180.0 / math.pi
D2R = math.pi / 180.0
EPSILON = 1e-6

#VC Statement Index
stmt_index = 0

#URP Waypoint numbering
current_way_point = 0

#URP loop check
loop_count = 1
prefix = '../../'
loop_children = None
Is_In_Loop = False


def postProcess(app,program,uri):
  global controller, motiontarget, tcp, pp_type, set_tcp, movel_as_joints, path_motion_type, use_acc, in_type, out_type
  global ur_config, CONTROLLER_VERSION
  global routine_variables
  global urp_use_active_tcp, sub_prog_tree_visibility
  
  cmd = getCommand()
  
  pp_type = cmd.getProperty('PP Type').Value
  urp_use_active_tcp =  cmd.getProperty('Use Active TCP').Value
  sub_prog_tree_visibility =  cmd.getProperty('Hide Sub Program Tree').Value
  set_tcp = cmd.getProperty('Use set_tcp').Value
  use_acc = cmd.getProperty('Use acceleration values').Value
  movel_as_joints = cmd.getProperty('movel as joint values').Value
  path_motion_type = cmd.getProperty('Path motion type').Value
  in_type = cmd.getProperty('Input mapping').Value
  out_type = cmd.getProperty('Output mapping').Value
  
  
  # Write script in uri
  uri = uri[:uri.rfind('.')] + '.script'
  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-7]
  
  
  #URP-Create temp txt file with format and convert to gzip
  controller = program.Executor.Controller
  motiontarget = controller.createTarget()
  tcp = -1
  urp_file_name = mainName + ".urp"
  temp_txt_file = mainName + "_temp.txt"
  uri_urp = (os.path.join(head, urp_file_name))
  uri_temp_txt = (os.path.join(head, temp_txt_file))
  if controller.Name == "e-Series":
    CONTROLLER_VERSION = "5.10.0"
  else:#if controller.Name == "CB3":
    CONTROLLER_VERSION = "3.15.0"
  comp = controller.Component
  comp_id = get_robot_id(comp)
  
  # Get the kinematic configuration for UR
  ur_config = get_ur_config(comp_id)
  if not ur_config:
    if pp_type == 'URP':
      print 'WARNING: URP Post-Processor only supports e-Series models.'
    ur_config = '<Kinematics/>'
  
  
  # Generate both urp and script and delete other at the end
  urps = []
  with open(uri,"w") as output_file:
    urp_generate_root(mainName)
    # main routine
    get_routine_variables(program.MainRoutine)
    translateRoutine(program.MainRoutine, mainName, output_file)
    tree = ET.ElementTree(root)
    tree.write(uri_temp_txt)
    encode_urp_file(uri_temp_txt, uri_urp) #Compress txt file to gzip
    urps.append(uri_urp)
    
    # subroutines
    for routine in program.Routines:
      urp_file_name = routine.Name + '.urp'
      uri_temp_txt = routine.Name + '_temp.txt'
      uri_urp = (os.path.join(head, urp_file_name))
      uri_temp_txt = (os.path.join(head,temp_txt_file))
      urp_generate_root(routine.Name)
      get_routine_variables(routine)
      translateRoutine(routine, routine.Name, output_file)
      tree = ET.ElementTree(root)
      tree.write(uri_temp_txt)
      encode_urp_file(uri_temp_txt, uri_urp) #Compress txt file to gzip
      urps.append(uri_urp)
    
    output_file.write("%s()\n" % mainName)
    output_file.close()
    #tree = ET.ElementTree(root)
    #tree.write(uri_temp_txt)
    #encode_urp_file(uri_temp_txt, uri_urp) #Compress txt file to gzip
    os.remove(uri_temp_txt)
    
    if pp_type == 'URP':
      #Remove script
      os.remove(uri)
      files = urps
    else:
      #Remove urps
      for uri_urp in urps:
        os.remove(uri_urp)
      #os.remove(uri_temp_txt)
      files = [uri]
    
    
    return True,files
  return False,files


def getProperties():
  #Properties for action panel
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  props.append((VC_STRING, 'PP Type', 'URP', VC_PROPERTY_STEP, ['URP', 'Script'], 0, 0))
  
  #URP only
  props.append((VC_BOOLEAN, 'Use Active TCP', False, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Hide Sub Program Tree', False, None, None, 0, 0))
  
  #script only
  props.append((VC_BOOLEAN, 'Use set_tcp', True, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Use acceleration values', False, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'movel as joint values', False, None, None, 0, 0))
  
  #URP and script
  props.append((VC_STRING, 'Path motion type', 'movel', VC_PROPERTY_STEP, ['movel', 'movep'], 0, 0))
  props.append((VC_STRING, 'Input mapping', 'get_configurable_digital_in', VC_PROPERTY_STEP, ['get_configurable_digital_in', 'get_standard_digital_in', 'read_input_boolean_register', 'get_tool_digital_in', 'get_euromap_input'], 0, 0))
  props.append((VC_STRING, 'Output mapping', 'set_configurable_digital_out', VC_PROPERTY_STEP, ['set_configurable_digital_out', 'set_standard_digital_out', 'write_output_boolean_register', 'set_tool_digital_out', 'set_euromap_output'], 0, 0))
  
  return props


def getFileFilter():
  #Override file filter
  cmd = getCommand()
  pp_type_prop = cmd.getProperty('PP Type')
  if not pp_type_prop or pp_type_prop.Value == 'URP':
    file_filter = "Universal Robot Program file (*.URP)|*.URP"
  else:
    file_filter = "Universal Robot script file (*.script)|*.script"
  return file_filter


def updateActionPanel():
  #Update properties on action panel
  cmd = getCommand()
  pp_type_prop = cmd.getProperty('PP Type')
  output_prop = cmd.getProperty('Output')
  urp_type = pp_type_prop.Value == 'URP'
  
  #URP only
  cmd.getProperty('Use Active TCP').IsVisible = urp_type
  cmd.getProperty('Hide Sub Program Tree').IsVisible = urp_type
  
  #script only
  cmd.getProperty('Use set_tcp').IsVisible = not urp_type
  cmd.getProperty('Use acceleration values').IsVisible = not urp_type
  cmd.getProperty('movel as joint values').IsVisible = not urp_type
  cmd.getProperty('Path motion type').IsVisible = not urp_type
  
  #URP and script
  cmd.getProperty('Path motion type').IsVisible = True
  cmd.getProperty('Input mapping').IsVisible = True
  cmd.getProperty('Output mapping').IsVisible = True
  
  
  uri = output_prop.Value
  
  if urp_type:
    uri = uri[0:uri.rfind('.')] + '.URP'
  else:
    uri = uri[0:uri.rfind('.')] + '.script'
  output_prop.Value = uri


def get_robot_id(comp):
  prop = comp.getProperty('RobotModelID')
  if prop:
    words = prop.Value.replace(' ','').split('|')
    if words and len(words) >= 2:
      model = words[1]
      return model


def get_ur_config(robotname):
  #UR configs
  global kin_default
  '''
  DH parameters for e-Series robots
  '''
  if robotname=="UR3e":
    kin_default = '''<Kinematics status="NOT_INITIALIZED" validChecksum="false">
                    <deltaTheta value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
                    <a value="0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0"/>
                    <d value="0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921"/>
                    <alpha value="1.570796327, 0.0, 0.0, 1.570796327, -1.570796327, 0.0"/>
                    <jointChecksum value="-1, -1, -1, -1, -1, -1"/>
                  </Kinematics>
                  '''
  elif robotname=="UR5e":
    kin_default = '''<Kinematics status="NOT_INITIALIZED" validChecksum="false">
                    <deltaTheta value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
                    <a value="0.0, -0.425, -0.3922, 0.0, 0.0, 0.0"/>
                    <d value="0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996"/>
                    <alpha value="1.570796327, 0.0, 0.0, 1.570796327, -1.570796327, 0.0"/>
                    <jointChecksum value="-1, -1, -1, -1, -1, -1"/>
                  </Kinematics>
                  '''
  elif robotname=="UR10e":
    kin_default = '''<Kinematics status="NOT_INITIALIZED" validChecksum="false">
                      <deltaTheta value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
                      <a value="0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0"/>
                      <d value="0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655"/>
                      <alpha value="1.570796327, 0.0, 0.0, 1.570796327, -1.570796327, 0.0"/>
                      <jointChecksum value="-1, -1, -1, -1, -1, -1"/>
                    </Kinematics>
                  '''
  elif robotname=="UR16e":
    kin_default = '''  <Kinematics status="NOT_INITIALIZED" validChecksum="false">
                      <deltaTheta value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
                      <a value="0.0, -0.4784, -0.36, 0.0, 0.0, 0.0"/>
                      <d value="0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655"/>
                      <alpha value="1.570796327, 0.0, 0.0, 1.570796327, -1.570796327, 0.0"/>
                      <jointChecksum value="-1, -1, -1, -1, -1, -1"/>
                    </Kinematics>
                  '''
  else:
    kin_default = ''
  
  return kin_default


def get_port_value(val):
  if val:
    return '1'
  else:
    return '0'


def get_routine_variable_type_index(target_property):
  '''
  Get the routine variable index
  '''
  if routine_variables.has_key(target_property):
    var_type = routine_variables[target_property]
    all_routine_var = list(routine_variables)
    index = all_routine_var.index(target_property)
    #URP indexing starts from 1 NOT from 0
    return var_type, str(index+1)
  else:
    return None, None


def write_urp_call_subprogram(routine_name):
  '''
  Call SubProg statement
  '''
  if sub_prog_tree_visibility:
    sub_prog_keep_hidden = "true"
  else:
    sub_prog_keep_hidden = "false"

  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  call_sub_prog = ET.SubElement(main_program_children, "CallSubProgram")
  ref_sub_prog = ET.SubElement(root_childern, "SubProgram", name=routine_name, keepHidden=sub_prog_keep_hidden, keepSynchronizedWithDisk="false")
  ref_sub_prog_children = ET.SubElement(ref_sub_prog, "children")
  ET.SubElement(ref_sub_prog_children,"Placeholder")


def write_urp_routine_variable(var_dict):
  '''
  Get routine variable and it's type.
  Based on the variable type create subelements with initial values
  '''
  global list_routine_var, list_routine_var_type

  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  list_routine_var = list(var_dict.keys())
  list_routine_var_type = list(var_dict.values())
  #var_dict have Name:Type as key value pair
  for var in var_dict:
    assign = ET.SubElement(main_program_children,"Assignment", valueSource="Expression")
    assign_var = ET.SubElement(assign,"variable", name=var, prefersPersistentValue="")
    ET.SubElement(assign_var, "initializeExpression")
    assign_expression = ET.SubElement(assign,"expression")
    if var_dict[var] == "Integer":
      #Initial VC routines int values is 0
      ET.SubElement(assign_expression,"ExpressionChar", character="0")
    if var_dict[var] == "Boolean":
      #Initial VC routines bool value is false
      ET.SubElement(assign_expression,"ExpressionToken", token=" False ")
    if var_dict[var] == "Real":
      #Initial VC routines real value is 0.0
      ET.SubElement(assign_expression,"ExpressionChar", character="0")
      ET.SubElement(assign_expression,"ExpressionChar", character=".")
      ET.SubElement(assign_expression,"ExpressionChar", character="0")
    if var_dict[var] == "String":
      #Initial VC routines srting value is ""
      ET.SubElement(assign_expression,"ExpressionChar", character="\"")
      ET.SubElement(assign_expression,"ExpressionChar", character="\"")


def write_urp_assignment(var_name, index, var_type, has_program_var, expression_chars):
  global loop_count
  
  '''
  Assignment statement
  '''
  if Is_In_Loop:
    main_program_children = loop_children
    loop_count += 1
  else:
    main_program_children = main_program.find("children")

  assign = ET.SubElement(main_program_children,"Assignment", valueSource="Expression")
  #Variable should be initialized as RoutineVariables. (TEMP)In case if they are not, a variable is created.
  if index is None:
    assign_var = ET.SubElement(assign,"variable", name=var_name, prefersPersistentValue="")
    ET.SubElement(assign_var, "initializeExpression")
    assign_expression = ET.SubElement(assign,"expression")
    for char in expression_chars:
      #assignment expression is splited to individual element and used as character for expression
      ET.SubElement(assign_expression,"ExpressionChar", character=char)
  else:  
    #URP assignment refer to the variable index.First variable in routines will have index 1.   
    if index == "1" and Is_In_Loop:
      ref_val = '{}Assignment/variable'.format(prefix+loop_count*prefix)
    elif index == "1":
      #ref_val = '../../Assignment/variable'
      ref_val = '{}Assignment/variable'.format(prefix)
    else:
      if Is_In_Loop:
        ref_val = '{}Assignment[{}]/variable'.format(prefix+loop_count*prefix, index)
      else:
        ref_val = '{}Assignment[{}]/variable'.format(prefix,index)
    ET.SubElement(assign,"variable", reference=ref_val)
    assign_expression = ET.SubElement(assign,"expression")

  if var_type == "Integer" or var_type == "Real":
    if has_program_var:
      #If main routine variable is used in the expression, programvariable element is created
      if index == "1" and Is_In_Loop:
        loop_count += 1
        ref_val = '{}Assignment/variable'.format(prefix+loop_count*prefix)
        loop_count -= 1
      elif index == "1":
        ref_val = '../../../../Assignment/variable'
      else:
        if Is_In_Loop:
          loop_count += 1
          #ref_val = '../../../../../../Assignment[{}]/variable'.format(index)
          ref_val = '{}Assignment[{}]/variable'.format(prefix+loop_count*prefix, index)
          loop_count -= 1
        else:
          ref_val = '../../../../Assignment[{}]/variable'.format(index)
          express_var = ET.SubElement(assign_expression,"ExpressionVariable")
          ET.SubElement(express_var,"ProgramVariable", reference=ref_val)
    for char in expression_chars:
      #assignment expression is splited to individual element and used as character for expression
      ET.SubElement(assign_expression,"ExpressionChar", character=char)
  elif var_type == "Boolean":
    value = ''.join(expression_chars)
    ET.SubElement(assign_expression,"ExpressionToken", token=value)
  elif var_type == "String":
    for char in expression_chars:
       #assignment expression is splited to individual element and used as character for expression
      ET.SubElement(assign_expression,"ExpressionChar", character=char)
    #ET.SubElement(assign_expression,"ExpressionChar", character="\"")
    #ET.SubElement(assign_expression,"ExpressionChar", character="\"")
  pass


def write_urp_move(motion_type, speed, acc, blend_radius, IsActiveTCP, j_speed, j_acc, cartesian_speed, cartesian_acc, joint_value, tcp_offset, base_offset):
  '''
  MoveX statement
  '''
  global kin_default, current_way_point
  
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")
  
  current_way_point += 1
  if motion_type=="MoveP":
    statement_type = ET.SubElement(main_program_children, "Move", motionType = motion_type, speed = speed, acceleration = acc, blendRadius = blend_radius, useActiveTCP = IsActiveTCP, positionType = "CartesianPose")
  else:
    statement_type = ET.SubElement(main_program_children, "Move", motionType = motion_type, speed = speed, acceleration = acc, useActiveTCP = IsActiveTCP, positionType = "CartesianPose")
  if IsActiveTCP == "false":
    ET.SubElement(statement_type, "tcp", referencedName=tcp_name)
  way_point_children = ET.SubElement(statement_type, "children")
  way_point = ET.SubElement(way_point_children, "Waypoint", type="Fixed", name="Waypoint_"+str(current_way_point), kinematicsFlags="")
  if motion_type=="MoveJ":
    ET.SubElement(way_point,"motionParameters", blendRadius = blend_radius)
  if motion_type=="MoveL":
    ET.SubElement(way_point,"motionParameters", blendRadius = blend_radius)

  position = ET.SubElement(way_point,"position")
  ET.SubElement(position, "JointAngles", angles=joint_value)
  ET.SubElement(position, "TCPOffset", pose=tcp_offset)
  if kin_default:
    position.append(ET.fromstring(kin_default))
  position = ET.SubElement(way_point, "BaseToFeature", pose=base_offset)


def write_urp_wait(t, in_type=None, port=None, port_value=None):
  '''
  Wait statement. Delay and input signal 
  '''
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  if not t:
    digi_signal_value = get_port_value(port_value)
    wait = ET.SubElement(main_program_children,"Wait", type="DigitalInput")
    if in_type == "get_configurable_digital_in":
      reference_name = 'config_in[{}]'.format(port)
    if in_type == "get_standard_digital_in" or "get_euromap_input":
      reference_name = 'digital_in[{}]'.format(port)
    if in_type == "read_input_boolean_register":
      reference_name = 'GP_bool_in[{}]'.format(port) 
    if in_type == "get_tool_digital_in":
      reference_name = 'tool_in[{}]'.format(port)
    ET.SubElement(wait, "pin" , referencedName=reference_name)
    digi_value = ET.SubElement(wait,"digitalValue")
    digi_value.text = digi_signal_value
  else:
    wait = ET.SubElement(main_program_children,"Wait", type="Sleep")
    wait_time = ET.SubElement(wait,"waitTime")
    wait_time.text = str(t)


def write_urp_set(out_type=None, port=None, port_value=None):
  '''
  Set output statement. Set signal out
  '''
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  set_stat = ET.SubElement(main_program_children,"Set", type="DigitalOutput")
  digi_signal_value = get_port_value(port_value)
  if out_type == "set_configurable_digital_out":
    reference_name = 'config_out[{}]'.format(port)
  if out_type == "set_standard_digital_out" or "set_euromap_output":
    reference_name = 'digital_out[{}]'.format(port)
  if out_type == "write_output_boolean_register":
    reference_name = 'GP_bool_out[{}]'.format(port)
  if out_type == "set_tool_digital_out":
    reference_name = 'tool_out[{}]'.format(port)
  ET.SubElement(set_stat, "pin" , referencedName=reference_name)
  digi_value = ET.SubElement(set_stat,"digitalValue")
  digi_value.text = digi_signal_value


def write_urp_loop(loop_type, has_program_var, index=None, variable_name=None, operator=None, values=None):
  '''
  Loop - Forever or some condition
   '''
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")
 
  if loop_type == "Forever":
    loop = ET.SubElement(main_program_children, "Loop", type=loop_type)
  else:
    loop = ET.SubElement(main_program_children, "Loop", type=loop_type, checkContinuously="")
    expression = ET.SubElement(loop, "expression")
    expression_var = ET.SubElement(expression, "ExpressionVariable")
    if has_program_var:
      if index == "1":
        #ref_val = '../../../../Assignment/variable'
        ref_val = '{}Assignment/variable'.format(prefix+loop_count*prefix)
      else:
        #ref_val = '../../../../Assignment[{}]/variable'.format(index)
        ref_val = '{}Assignment[{}]/variable'.format(prefix+loop_count*prefix, index)
      ET.SubElement(expression_var,"ProgramVariable", reference=ref_val)
    else:  
      prog_var = ET.SubElement(expression_var, "ProgramVariable", name=variable_name, prefersPersistentValue="")
      ET.SubElement(prog_var, "initializeExpression")
    ET.SubElement(expression, "ExpressionChar", character=operator)
    for char in values:
      #assignment expression is splited to individual element and used as character for expression
      ET.SubElement(expression,"ExpressionChar", character=char)
  return loop


def write_urp_if_else_variable(condition, loop_type, has_program_var, index=None, variable_name=None, operator=None, values=None):


  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")
  
  if_loop = ET.SubElement(main_program_children, "If", type=loop_type, checkContinuously="false")

  if loop_type == "If":
    if_expression = ET.SubElement(if_loop,"expression")
    #if condition == '1' or 'True':
    #  ET.SubElement(if_expression,"ExpressionToken", token="True")
    if condition is None:
      expression_var = ET.SubElement(if_expression, "ExpressionVariable")
      if has_program_var:
        if index == "1":
          #ref_val = '../../../../Assignment/variable'
          ref_val = '{}Assignment/variable'.format(prefix+loop_count*prefix)
        else:
          #ref_val = '../../../../Assignment[{}]/variable'.format(index)
          ref_val = '{}Assignment[{}]/variable'.format(prefix+loop_count*prefix, index)
        ET.SubElement(expression_var,"ProgramVariable", reference=ref_val)
      else:  
        prog_var = ET.SubElement(expression_var, "ProgramVariable", name=variable_name, prefersPersistentValue="")
        ET.SubElement(prog_var, "initializeExpression")
      for char in operator:
        ET.SubElement(if_expression, "ExpressionChar", character=char)
      for char in values:
        #assignment expression is splited to individual element and used as character for expression
        ET.SubElement(if_expression,"ExpressionChar", character=char)
  
  else_loop = ET.SubElement(main_program_children, "If", type ="Else", checkContinuously="false")
  ET.SubElement(else_loop,"expression")
  
  return if_loop, else_loop


def writeSetProperty(output_file,statement):
  '''
  Assignment statement
  '''
  target_property = statement.TargetProperty
  value_expression = statement.ValueExpression.strip()
  output_file.write(indentation*depth + "%s = %s\n" % (target_property, value_expression))
  var_type, var_index = get_routine_variable_type_index(target_property)
  values = re.split(r'([+\-*^/()])',value_expression)
  if var_index is not None:
    if target_property in values:
      expression_program_var = filter(lambda x: x == target_property, values)
      #print 'expression_program_var', expression_program_var
      if expression_program_var:
        for i in expression_program_var:
          if not Is_In_Loop:
            if i in values:values.remove(i)
        has_prgm_var = True
    else:
      has_prgm_var = False
      #If the variable is not used in the assignment expression
      expression_program_var = target_property
  else:
    has_prgm_var = True
    #If the variable is not initialized in the routine
    expression_program_var = target_property
  #print 'values', values
  values = map(str,''.join(values))  
  write_urp_assignment(expression_program_var, var_index, var_type, has_prgm_var, values)


def writePrint(output_file,statement):

  output_file.write(indentation*depth + 'textmsg("%s")\n' % statement.Message)
  
  #URP
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  ET.SubElement(main_program_children,"Popup", type="Message", haltProgram="false", message=statement.Message, inputType="Text")


def writeIf(output_file,statement):
  global depth, Is_In_Loop, loop_children, loop_count
  
  _condition = statement.Condition.strip()
  output_file.write(indentation*depth + "if %s:\n" % _condition)
  depth += 1

  #URP
  temp_flag = False #Check if this is inside a loop (while/If/else)
  temp_loop_childern = None
  if Is_In_Loop:
    loop_count += 1
    temp_flag = True #if 
    temp_loop_childern = loop_children #Take the loop (while/If/else) as temp children element
  loop_type = "If"
  if _condition in ['1','True']:
    has_program_var = False
    if_loop_element, else_loop_element = write_urp_if_else_variable(_condition, loop_type, has_program_var)
  else:
    condition_variable_expression = re.split(r'(<[=>]?|==|=|>=?|!=|\&\&|\|\|)',_condition)
    condition_variable = condition_variable_expression[0]      
    condition_operator = condition_variable_expression[1]    
    condition_value = condition_variable_expression[2]   
    var_type, var_index = get_routine_variable_type_index(condition_variable)
    if var_index:  
      has_program_var = True
      index = var_index
    else:
      has_program_var = False
    if_loop_element, else_loop_element = write_urp_if_else_variable(None, loop_type, has_program_var, index, condition_variable, condition_operator, condition_value)
  if_loop_children = ET.SubElement(if_loop_element, "children")

  if not statement.ThenScope.Statements:
    # not sure if UR script supports "pass" statement => test
    output_file.write(indentation*depth + "#pass\n" )
    #URP
    ET.SubElement(if_loop_children, "Placeholder")
  loop_children = if_loop_children
  for s in statement.ThenScope.Statements:
    translator = statement_translators.get(s.Type, unknown)
    #URP
    Is_In_Loop = True
    if s.Type == "IfElse":
      loop_count += 1
    translator(output_file,s)
  depth -= 1

  #URP
  loop_count -= 1

  output_file.write(indentation*depth + "else:\n")
  depth += 1
  
  #URP
  else_loop_children = ET.SubElement(else_loop_element, "children")

  if not statement.ElseScope.Statements:
    # not sure if UR script supports "pass" statement => test
    output_file.write(indentation*depth + "#pass\n" )
    #URP
    ET.SubElement(else_loop_children, "Placeholder")
  loop_children = else_loop_children 
  for s in statement.ElseScope.Statements:
    translator = statement_translators.get(s.Type, unknown)
    #URP
    Is_In_Loop = True
    if s.Type == "IfElse":
      loop_count += 1
    translator(output_file,s)
  #URP
  loop_count -= 1
  Is_In_Loop = False

  #If this was called from (while/If/else) set the children back to the original loop
  if temp_flag:
    loop_children = temp_loop_childern
  
  depth -= 1
  output_file.write(indentation*depth + "end #if\n")


def writeReturn(output_file,statement):
  output_file.write(indentation*depth + "return\n")
  print 'URP > Unsupported statement type skipped:', statement.Type


def writeHalt(output_file,statement):
  output_file.write(indentation*depth + "halt\n")
  
  #URP
  if Is_In_Loop:
    main_program_children = loop_children
  else:
    main_program_children = main_program.find("children")

  ET.SubElement(main_program_children,"Halt")


def writeContinue(output_file,statement):
  output_file.write(indentation*depth + "continue\n")
  print 'URP > Unsupported statement type skipped:', statement.Type


def writeBreak(output_file,statement):
  output_file.write(indentation*depth + "break\n")
  print 'URP > Unsupported statement type skipped:', statement.Type


def writeWhile(output_file,statement):
  global depth, Is_In_Loop, loop_children, loop_count

  _condition = statement.Condition.strip()
  output_file.write(indentation*depth + "while %s:\n" % _condition )
  depth += 1

  #URP
  if Is_In_Loop:
    loop_count += 1
  index = None
  if _condition in ['1','True']:
    has_program_var = False
    loop_element = write_urp_loop("Forever",has_program_var)
  else: 
    condition_variable_expression = re.split(r'(<[=>]?|==|=|>=?|!=|\&\&|\|\|)',_condition)
    condition_variable = condition_variable_expression[0]      
    condition_operator = condition_variable_expression[1]    
    condition_value = condition_variable_expression[2:]
    values = map(str,''.join(condition_value))    
    var_type, var_index = get_routine_variable_type_index(condition_variable)
    if var_index:  
      has_program_var = True
      index = var_index
    else:
      has_program_var = False
    loop_element = write_urp_loop("While", has_program_var, index, condition_variable, condition_operator, values)
  loop_children = ET.SubElement(loop_element, "children")

  if not statement.Scope.Statements:
    # not sure if UR script supports "pass" statement => test
    output_file.write(indentation*depth + "pass\n" )
    ET.SubElement(loop_children, "Placeholder")
  for s in statement.Scope.Statements:
    Is_In_Loop = True
    if s.Type == "While":
      loop_count += 1
    translator = statement_translators.get(s.Type, unknown)
    translator(output_file,s)
  depth -= 1
  output_file.write(indentation*depth + "end #while\n")
  loop_count -= 1
  Is_In_Loop = False


def writeWaitBin(output_file, statement):
  #optionally
  # - get_euromap_input(n) # n: An integer specifying one of the available Euromap67 input signals. URP don't have this, so using the standard digital input
  # - get_standard_digital_in(n) # n:The number (id) of the input, integer: [0:7]
  # - get_tool_digital_in(n) # n: The number (id) of the input, integer: [0:1]
  # - read_input_boolean_register(address) # address: Address of the register (0:63)
  in_port = statement.InputPort
  in_value = statement.InputValue
  if in_value:
    output_file.write(indentation*depth + "while not %s(%i):\n" %(in_type, in_port ))
  else:
    output_file.write(indentation*depth + "while %s(%i):\n" % (in_type, in_port))
  output_file.write(indentation*depth + "  sleep(0.05)\n") #20hz polling
  output_file.write(indentation*depth + "end\n")
  #URP 
  write_urp_wait(None, in_type, in_port, in_value)


def writeSetBin(output_file, statement):
  out_port = statement.OutputPort
  out_value = statement.OutputValue
  output_file.write(indentation*depth + "%s(%i,%s)\n" % (out_type, out_port, out_value))
  #URP 
  write_urp_set(out_type, out_port, out_value)


def writeDelay(output_file, statement):
  delay_time = statement.Delay
  output_file.write(indentation*depth + "sleep(%.3f)\n" %delay_time)
  #URP
  write_urp_wait('%.3f' % delay_time)


def writeComment(output_file, statement):
  output_file.write(indentation*depth + "# %s\n" % statement.Comment)
  #URP
  ET.SubElement(main_program_children,"Comment", comment=str(statement.Comment))


def writeCall(output_file, statement):
  if statement.getProperty("Routine").Value:
    routine_name = statement.getProperty("Routine").Value.Name
    output_file.write(indentation*depth + "%s() #subroutine call\n" % routine_name)
    write_urp_call_subprogram(routine_name)


def writeLinMotion(output_file, statement):
  global motiontarget, movel_as_joints

  motion_type = "MoveL"
  setTcp(statement, output_file)
  tcp_pose = tcpToArray(statement)
  if tcp_pose:
    tcp_pose = [ '%f' % pos for pos in tcp_pose ]
  blend_radius = '0'
  if urp_use_active_tcp:
    active_tcp = "true" 
    tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  else:
    active_tcp = "false" #This will use the statement TCP
  r_string = ''
  if statement.AccuracyValue > 0:
    r_string = ',r=%.3f' % (statement.AccuracyValue/1000.0)
    #motion_type = "MoveP"
    blend_radius = '%.2f' % (statement.AccuracyValue/1000.0)
  v_cartesian = statement.MaxSpeed/1000.0
  a_cartesian = statement.Acceleration/1000.0
  a_string = ''
  if use_acc:
    a_string = ',a=%.3f' % (statement.Acceleration/1000.0)
  if pp_type != 'URP' and not movel_as_joints:
    #Write as pose (X,Y,Z,RX,RY,RZ)
    output_file.write(indentation*depth + "movel(")
    statement.writeToTarget(motiontarget)
    pose = poseTrans(statement.Base, motiontarget.Target)
    p = pose.P
    ori = pose.getAxisAngle()
    x,y,z       = p.X/1000.0, p.Y/1000.0, p.Z/1000.0
    ax, ay, az  = math.radians(ori.W)*ori.X, math.radians(ori.W)*ori.Y, math.radians(ori.W)*ori.Z
    urp_j_values = (x, y, z, ax, ay, az)
    output_file.write("p[%f, %f, %f, %f, %f, %f]" % (x,y,z, ax, ay, az) )
    output_file.write("%s,v=%.3f%s)\n" % (a_string, statement.MaxSpeed/1000.0, r_string))
  else:
    #Write as joints
    output_file.write(indentation*depth + "movel([")
    urp_j_values = []
    for j in range(6):
      val = statement.Positions[0].JointValues[j]
      urp_j_values.append("%.16f" % math.radians(val))
      val = "%f" % math.radians(val)
      output_file.write(val)
      if j<5:
        output_file.write(",")
    output_file.write("],v=%.3f%s)\n" % (statement.MaxSpeed/1000.0, r_string) )

  #URP 
  base_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #VERIFY THIS
  v = '%.3f' % v_cartesian
  a = '%.3f' % a_cartesian
  '''
  #We use shared parameters. No need to override the speed, acceleration for the waypoints
  j_speed = '%.16f' % (controller.Joints[0].MaxSpeed*D2R)
  j_acc = '%.16f' % (controller.Joints[0].MaxAcceleration*D2R)
  cartesian_speed = '%.3f' % v_cartesian
  cartesian_acc = '%.3f' % a_cartesian
  '''
  j_angles = ', '.join(map(str,urp_j_values))
  t_offset = ', '.join(map(str,tcp_pose))
  b_offset = ', '.join(map(str,base_pose)) 
  write_urp_move(motion_type, v, a, blend_radius, active_tcp, None, None, None, None, j_angles, t_offset, b_offset)


def writePtpMotion(output_file, statement):
  motion_type = "MoveJ"
  setTcp(statement, output_file)
  tcp_pose = tcpToArray(statement)
  tcp_pose = [ '%f' % pos for pos in tcp_pose ]
  blend_radius = '0'
  if urp_use_active_tcp:
    active_tcp = "true" 
    tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  else:
    active_tcp = "false" #This will use the statement TCP
  r_string = ''
  if statement.AccuracyValue > 0:
    r_string = ',r=%.3f' % (statement.AccuracyValue/1000.0)
    blend_radius = '%.2f' % (statement.AccuracyValue/1000.0)
  v = statement.JointSpeed*controller.Joints[0].MaxSpeed*D2R
  a_string = ''
  max_acc = statement.JointForce*controller.Joints[0].MaxAcceleration*D2R
  if use_acc:
    a_string = ',a=%.3f' % (max_acc)
  #script
  output_file.write(indentation*depth + "movej([")
  urp_j_values = []
  for j in range(6):
    val = statement.Positions[0].JointValues[j]
    urp_j_values.append("%.16f"%(math.radians(val)))
    val = "%f" % math.radians(val)
    output_file.write(val)
    if j<5:
      output_file.write(",")
  output_file.write("]%s,v=%.3f%s)\n" % (a_string,v, r_string))

  #URP 
  v = '%.3f' % v
  a = '%.3f' % max_acc
  j_angles = ', '.join(map(str,urp_j_values))
  t_offset = ', '.join(map(str,tcp_pose))
  b_offset = "0.0, 0.0, 0.0, 0.0, 0.0, 0.0"
  write_urp_move(motion_type, v, a, blend_radius, active_tcp, None, None, None, None, j_angles, t_offset, b_offset)
  

def writePath(output_file,statement):
  global motiontarget, movel_as_joints, tcp_name

  motion_type = "MoveL"
  if path_motion_type == "movep":
    motion_type = "MoveP"
  
  setTcp(statement, output_file)
  tcp_pose = tcpToArray(statement)
  #URP
  tcp_pose = [ '%f' % pos for pos in tcp_pose ]
  blend_radius = '0'
  if urp_use_active_tcp:
    active_tcp = "true" 
    tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  else:
    active_tcp = "false" #This will use the statement TCP
  #
  output_file.write(indentation*depth + "# <START PATH: %s>\n" % (statement.Name))
  motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR    
  if statement.Base == None:
    motiontarget.BaseName = ""
  else:
    motiontarget.BaseName = statement.Base.Name
  if statement.Tool == None:
    motiontarget.ToolName = ""
    tcp_name = 'Tool0'
  else:
    motiontarget.ToolName = statement.Tool.Name
    tcp_name = motiontarget.ToolName
  for i in range( statement.getSchemaSize()):
    target = statement.getSchemaValue(i,"Position")
    motiontarget.Target = target
    jv = motiontarget.JointValues
    motiontarget.JointValues = jv
    motiontarget.AccuracyMethod = statement.getSchemaValue(i,"AccuracyMethod")
    motiontarget.AccuracyValue = statement.getSchemaValue(i,"AccuracyValue")
    motiontarget.CartesianAcceleration = statement.getSchemaValue(i,"Acceleration")
    motiontarget.CartesianDeceleration = statement.getSchemaValue(i,"Deceleration")
    speed = statement.getSchemaValue(i,"MaxSpeed")
    v_cartesian = speed/1000.0
    a_cartesian = motiontarget.CartesianAcceleration/1000.0
    r_string = ''
    if motiontarget.AccuracyValue > 0:
      r_string = ',r=%.3f' % (motiontarget.AccuracyValue/1000.0)
      blend_radius = '%.3f' % (motiontarget.AccuracyValue/1000.0)
    a_string = ''
    if use_acc:
      a_string = ',a=%.3f' % (motiontarget.CartesianAcceleration)
    if pp_type != 'URP' and not movel_as_joints:
      #Write as pose (X,Y,Z,RX,RY,RZ)
      pose = poseTrans(statement.Base, motiontarget.Target)
      p = pose.P
      ori = pose.getAxisAngle()
      x,y,z       = p.X/1000.0, p.Y/1000.0, p.Z/1000.0
      ax, ay, az  = math.radians(ori.W)*ori.X, math.radians(ori.W)*ori.Y, math.radians(ori.W)*ori.Z
      urp_j_values = (x, y, z, ax, ay, az)
      output_file.write(indentation*depth + "%s(" % (path_motion_type))
      output_file.write("p[%f, %f, %f, %f, %f, %f]" % (x,y,z, ax, ay, az) )
      output_file.write("%s,v=%.3f%s)\n" % (a_string, speed/1000.0, r_string))
    else:
      #Write as joints
      urp_j_values = []
      output_file.write(indentation*depth + "%s([" % (path_motion_type))
      for j in range(6):
        val = jv[j]
        urp_j_values.append("%.16f" % math.radians(val))
        output_file.write("%f" % math.radians(val))
        if j<5:
          output_file.write(",")
      output_file.write("],v=%.3f%s)\n" % (speed/1000.0, r_string) )
      
    #URP 
    base_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #VERIFY THIS
    v = '%.3f' % v_cartesian
    a = '%.3f' % a_cartesian
    j_speed = '%.16f' % (controller.Joints[0].MaxSpeed*D2R)
    j_acc = '%.16f' % (controller.Joints[0].MaxAcceleration*D2R)
    cartesian_speed = '%.3f' % v_cartesian
    cartesian_acc = '%.3f' % a_cartesian
    j_angles = ', '.join(map(str,urp_j_values))
    t_offset = ', '.join(map(str,tcp_pose))
    b_offset = ', '.join(map(str,base_pose)) 
    write_urp_move(motion_type, v, a, blend_radius, active_tcp, j_speed, j_acc, cartesian_speed, cartesian_acc, j_angles, t_offset, b_offset)
      
  output_file.write(indentation*depth + "# <END PATH: %s>\n" % (statement.Name))


def unknown(output_file, statement):
  print '> Unsupported statement type skipped:', statement.Type


def translateRoutine( routine, name, output_file):
  global depth
  
  depth = 1
  output_file.write("def %s():\n" % name)
  for statement in routine.Statements:
    translator = statement_translators.get(statement.Type, unknown)
    translator(output_file,statement)
    #WriteStatement(output_file,statement)
  output_file.write("end #%s\n\n" % name)
  output_file.write("\n")


def encode_urp_file(temp_output_file, urp_output_file):
  try:
    f_in = open(temp_output_file, 'rb')
    with gzip.open(urp_output_file, 'wb') as f_out:
      shutil.copyfileobj(f_in, f_out)
  except:
    pass


def get_routine_variables(routine):
  #VC routine variables and types
  global routine_variables

  routine_variables = {}
  order_var = []
  for x in routine.Properties:
    if x.Name != "Name":
      order_var.append(x.Name)
      routine_variables[x.Name]=x.Type
  list_of_var = [(key, routine_variables[key]) for key in order_var]
  routine_variables = OrderedDict(list_of_var)
  if routine_variables:
    write_urp_routine_variable(routine_variables)


def urp_generate_root(mainName):
  global root, root_childern, main_program, main_program_children
  root = ET.Element("URProgram", name=mainName, createdIn=CONTROLLER_VERSION, lastSavedIn=CONTROLLER_VERSION)
  root.append(ET.fromstring(ur_config))
  root_childern = ET.SubElement(root,"children")
  main_program = ET.SubElement(root_childern,"MainProgram", runOnlyOnce="false", InitVariablesNode="false")
  main_program_children = ET.SubElement(main_program, "children")


def poseTrans(base, pose):
  #Translate pose into robot world coordinates if some base is used
  global controller
  
  if not base or not base.Node:
    return pose
  iworld = controller.Component.WorldPositionMatrix * controller.WorldTransformMatrix
  iworld.invert()
  base_world = base.Node.WorldPositionMatrix * base.PositionMatrix
  pose = iworld * base_world * pose
  return pose


def setTcp(statement, output_file):
  #Set active TCP coordinates
  global tcp, set_tcp
  
  if not set_tcp:
    return
  if not statement:
    output_file.write(indentation*depth + "set_tcp(p[%f, %f, %f, %f, %f, %f])\n" % (0, 0, 0, 0, 0, 0) )
  if statement.Tool != tcp:
    tcp = statement.Tool
    pose = vcMatrix.new()
    if tcp and tcp.Node:
      pose = controller.FlangeNode.InverseWorldPositionMatrix * tcp.Node.WorldPositionMatrix * tcp.PositionMatrix
    elif tcp:
      pose = tcp.PositionMatrix
    
    p = pose.P
    ori = pose.getAxisAngle()
    x,y,z       = p.X/1000.0, p.Y/1000.0, p.Z/1000.0
    ax, ay, az  = math.radians(ori.W)*ori.X, math.radians(ori.W)*ori.Y, math.radians(ori.W)*ori.Z
    output_file.write(indentation*depth + "set_tcp(p[%f, %f, %f, %f, %f, %f])\n" % (x,y,z, ax, ay, az) )


def tcpToArray(statement):
  #Write tcp to an array and also set active tcp_name
  global tcp_name
  
  tcp = statement.Tool
  if statement.Type is not 'Path':
    if tcp:
      tcp_name = tcp.Name
      if '[' in tcp_name:
        tcp_name = tcp_name.replace('[','').replace(']','')
    else:
      tcp_name = "Tool0" #Null tool in Visual Components.Tool_0 is created for the URP statement. Define this tool in Polyscope
  
  pose = vcMatrix.new()
  if tcp and tcp.Node:
    pose = controller.FlangeNode.InverseWorldPositionMatrix * tcp.Node.WorldPositionMatrix * tcp.PositionMatrix
  elif tcp:
    pose = tcp.PositionMatrix
  
  p = pose.P
  ori = pose.getAxisAngle()
  x,y,z       = p.X/1000.0, p.Y/1000.0, p.Z/1000.0
  ax, ay, az  = math.radians(ori.W)*ori.X, math.radians(ori.W)*ori.Y, math.radians(ori.W)*ori.Z
  tcp_val = [x, y, z, ax, ay, az]
  return tcp_val


indentation = '  '
statement_translators = {
VC_STATEMENT_SETPROPERTY:writeSetProperty,
VC_STATEMENT_PRINT:writePrint,
VC_STATEMENT_IF:writeIf,
VC_STATEMENT_RETURN:writeReturn,
VC_STATEMENT_HALT:writeHalt,
VC_STATEMENT_CONTINUE:writeContinue,
VC_STATEMENT_BREAK:writeBreak,
VC_STATEMENT_WHILE:writeWhile,
VC_STATEMENT_WAITBIN:writeWaitBin,
VC_STATEMENT_SETBIN:writeSetBin,
VC_STATEMENT_DELAY:writeDelay,
VC_STATEMENT_COMMENT:writeComment,
VC_STATEMENT_CALL:writeCall,
VC_STATEMENT_LINMOTION:writeLinMotion,
VC_STATEMENT_PTPMOTION:writePtpMotion,
VC_STATEMENT_PATH:writePath}
