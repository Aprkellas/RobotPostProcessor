# Kuka Sunrise post-processor, 0.10

from vcCommand import *
import os.path, time, re, math
import xml.etree.ElementTree as ET
import vcMatrix

class TranslatorHelper:
  #---
  # Capsulates commonly used handles and post processor settings
  #---
  def __init__(self, app, program, uri):
    
    # String cache
    self.cache_java = ''                                   # Cache java program code
    self.cache_globals = ''                                # Global variable declarations
    
    # RoboticsAPI.data.xml as ElementTree object
    self.data_xml = None
    self.base_elements = {}
    self.tool_elements = {}
    
    # Helpers
    self.app = app
    self.uri = uri
    self.program = program
    self.current_routine = None
    self.local_vars = []
    self.global_vars = []
    self.pos_names = []
    self.indentation = '    '
    self.depth = 0
    self.statement_translators = {
      VC_STATEMENT_BREAK:write_break,
      VC_STATEMENT_CALL:write_call,
      VC_STATEMENT_COMMENT:write_comment,
      VC_STATEMENT_CONTINUE:write_continue,
      VC_STATEMENT_DEFINE_BASE:unknown,
      VC_STATEMENT_DEFINE_TOOL:unknown,
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
  
  def current_indent(self):
    return self.indentation*self.depth
  
  def get_job_name(self):
    head, tail = os.path.split(self.uri)
    return tail[:-5].replace(' ', '_')
  
  def get_folder(self):
    head, tail = os.path.split(self.uri)
    return head
  
  def get_xml_filename(self):
    return 'RoboticsAPI.data.xml'
  
  def get_xml_uri(self):
    xml_uri = '%s\\%s' % (self.get_folder(), self.get_xml_filename())
    return xml_uri
  
  def write_java(self, line):
    self.cache_java += self.current_indent() + line + '\n'
  
  def write_globals(self, line, indent = 1):
    self.cache_globals += indent * self.indentation + line + '\n'


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
  
  # RoboticsAPI.data.xml
  init_data_xml(helper)
  write_frames(helper)
  
  # Write header
  write_java_header(helper)
  
  # Write routines
  helper.depth += 1
  routines = [helper.program.MainRoutine]
  routines.extend(helper.program.Routines)
  for routine in routines:
    write_routine(routine, routine.Name, helper)
  helper.depth -= 1
  
  # Write footer
  write_java_footer(helper)
  
  # Add globals to java code
  helper.cache_java = helper.cache_java.replace('%GLOBALS%', helper.cache_globals + 1 * helper.indentation)
  
  # Write output file
  files = []
  with open(helper.uri, 'w') as output_file:
    output_file.write(helper.cache_java)
  files.append(uri)
  
  # Write frame xml file
  indent_etree(helper.data_xml)
  xml_uri = helper.get_xml_uri()
  with open(xml_uri, 'w') as output_file:
    output_file.write(ET.tostring(helper.data_xml, 'UTF-8'))
  files.append(xml_uri)
  
  return True, [uri, xml_uri]


def write_java_header(helper):
  # Write jave "header"
  
  header = '''\
package application;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.ioModel.*;
import com.kuka.roboticsAPI.conditionModel.*;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;

public class %JOB% extends RoboticsAPIApplication {
    
    @Inject
    private ITaskLogger logger;
    @Inject
    private LBR robot;
    @Inject
    private IApplicationData data;
    
%GLOBALS%
    
    @Override
    public void initialize() {
    }
  
    @Override
    public void run() {
        %MAIN%;
    }
    
    private Output GetDO(int port) {
        //Map you outputs here
        return null;
    }
    
    private Input GetDI(int port) {
        //Map you inputs here
        return null;
    }
    
'''
  
  header = header.replace('%JOB%', helper.get_job_name())
  header = header.replace('%MAIN%', '%s()' % (helper.program.MainRoutine.Name))
  
  helper.cache_java += header


def write_java_footer(helper):
  # Write jave "footer"
  
  footer = '}'
  
  helper.cache_java += footer


def init_data_xml(helper):
  # Initialize RoboticsAPI.data.xml

  xml = '''\
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<RoboticsAPIData version="3">
    <world>
        <gravitation x="0.0" y="0.0" z="9.81"/>
    </world>
    <objectTemplates>
        <toolTemplate class="" name="SimulationTool">
            <frames>
            </frames>
            <loadData cogA="0.0" cogB="0.0" cogC="0.0" cogX="0.0" cogY="0.0" cogZ="0.0" inertiaX="0.0" inertiaY="0.0" inertiaZ="0.0" mass="0.0"/>
        </toolTemplate>
    </objectTemplates>
</RoboticsAPIData>
'''

  helper.data_xml = ET.fromstring(xml)


def write_frames(helper):
  # Write all used base and tool frames to data xml
  
  bases = []
  tools = []
  all_stats = get_all_statements(helper)
  use_null_base = False
  use_null_tool = False
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
  bases.sort(key = lambda x: x.Name if x else '')
  tools.sort(key = lambda x: x.Name if x else '')
  
  # Bases
  world_el = get_data_xml_element(helper, 'world')
  for b in bases:
    if b:
      base_name = b.Name
      m = get_base_in_reference(b, helper.motiontarget)
    else:
      base_name = ''
      m = vcMatrix.new()
    name = get_base_name(base_name)
    
    base_el = create_frame_element(helper, world_el, name, m)
    add_info_el = ET.SubElement(base_el, 'additionalInformation')
    create_last_modified_element(helper, add_info_el)
    info_el = ET.SubElement(add_info_el, 'info')
    info_el.set('key', 'UseAsBaseForJogging')
    info_el.set('type', 'java.lang.Boolean')
    info_el.set('value', 'true')
    helper.base_elements[name] = base_el
    
    if not b:
      continue
      
    #Check for external tcp configuration
    node = b.Node
    flange_node = helper.controller.FlangeNode
    while node:
      if node == flange_node:
        raise TranslatorException('External TCP not supported.')
      node = node.Parent
  #endfor
  
  # Tools
  sim_tool_el = get_data_xml_element(helper, 'objectTemplates/toolTemplate/frames')
  for t in tools:
    if t:
      tool_name = t.Name
      m = get_tool_in_reference(t, helper.motiontarget, helper.controller)
    else:
      tool_name = ''
      m = vcMatrix.new()
    name = get_tool_name(tool_name)
    
    tool_el = create_frame_element(helper, sim_tool_el, name, m)
    helper.tool_elements[name] = tool_el
  #endfor
  pass


def write_routine(routine, name, helper):
  # Write a routine
  
  helper.write_java('private void %s() {' % (routine.Name))
  helper.depth += 1
  
  # Write routine variables
  helper.local_vars = []
  vars_exist = False
  for p in routine.Properties:
    if p.Name in ['Name', 'ProjectDirectory', 'ProjectName', 'JobName', 'Author', 'Company', 'Division', 'Comment']:
      continue
    types = {VC_BOOLEAN : 'boolean', VC_INTEGER : 'int', VC_REAL : 'float', VC_STRING : 'String'}
    if p.Type in types:
      vars_exist = True
      type = types[p.Type]
      value = str(p.Value)
      if p.Type == VC_BOOLEAN:
        value = value.lower()
      elif p.Type == VC_STRING:
        value = '"%s"' % (value)
      elif p.Type == VC_REAL:
        value += 'f'
      helper.write_java('%s %s = %s;' % (type, p.Name, value))
      helper.local_vars.append(p.Name)
  if vars_exist:
    helper.write_java('')
  
  # Write statements
  for statement in routine.Statements:
    translator = helper.statement_translators.get(statement.Type, unknown)
    translator(statement, helper)
  
  helper.depth += -1
  helper.write_java('}')
  helper.write_java('')


def get_data_xml_element(helper, element_path):
  # Find data xml element and return it. If it doesn't exist it yet create it.
  
  if helper.data_xml is None:
    raise TranslatorException('RoboticsAPI.data.xml error.')
  
  args = element_path.split(r'/')
  parent_el = helper.data_xml
  while args:
    arg = args.pop(0)
    el = parent_el.find(arg)
    if el is None:
      el = ET.SubElement(parent_el, arg)
    parent_el = el
  
  return parent_el


def create_frame_element(helper, parent_el, name, m):
  # Create frame xml element to parent_el with name and transformation matrix m
  
  frame_el = ET.SubElement(parent_el, 'frame')
  frame_el.set('name', name)
  
  p = m.P
  wpr = m.WPR
  tf_el = ET.SubElement(frame_el, 'transformation')
  tf_el.set('x', '%.3f' % (p.X))
  tf_el.set('y', '%.3f' % (p.Y))
  tf_el.set('z', '%.3f' % (p.Z))
  tf_el.set('a', '%.6f' % (wpr.Z * math.pi / 180.0))
  tf_el.set('b', '%.6f' % (wpr.Y * math.pi / 180.0))
  tf_el.set('c', '%.6f' % (wpr.X * math.pi / 180.0))
  
  return frame_el


def create_last_modified_element(helper, parent_el):
  info_el = ET.SubElement(parent_el, 'info')
  info_el.set('key', 'LastModificationTime')
  info_el.set('type', 'java.lang.Long')
  info_el.set('value', str(int(round(time.time() * 1000))))


def indent_etree(elem, level = 0):
  # Indent element tree object
  one_indent = '    '
  i = '\n' + level * one_indent
  if len(elem):
    if not elem.text or not elem.text.strip():
      elem.text = i + one_indent
    if not elem.tail or not elem.tail.strip():
      elem.tail = i
    for elem in elem:
      indent_etree(elem, level+1)
    if not elem.tail or not elem.tail.strip():
      elem.tail = i
  else:
    if level and (not elem.tail or not elem.tail.strip()):
      elem.tail = i


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


def get_base_in_reference(base, motiontarget):
  # Convert base matrix to robot world
  if not base.Node:
    return base.PositionMatrix
  ref_m = motiontarget.getSimWorldToRobotWorld()
  ref_m.invert()
  return ref_m * base.Node.WorldPositionMatrix * base.PositionMatrix


def get_tool_in_reference(tool, motiontarget, ctr):
  # Convert tool matrix to robot flange
  if tool.Node and tool.Node != ctr.FlangeNode:
    ref_m = ctr.FlangeNode.WorldPositionMatrix
    ref_m.invert()
    tool_m = tool.Node.WorldPositionMatrix * tool.PositionMatrix
    tool_m_in_ref = ref_m * tool_m
  else:
    tool_m_in_ref = tool.PositionMatrix
  return tool_m_in_ref


def get_status(helper, motiontarget):
  #Get status from motiontarget
  map = {
    0:2,
    1:6,
    2:0,
    3:4,
    4:3,
    5:7,
    6:1,
    7:5
  }
  conf = motiontarget.RobotConfig
  if conf in map:
    return map[conf]
  else:
    return 0


def get_turn(helper, motiontarget):
  #Get turn from motiontarget. Note joint order A1, A2, A4, A5, A6, A7, A3!
  if helper.robot_joint_count == 7:
    indexing = [0, 1, 6, 2, 3, 4, 5]
  elif helper.robot_joint_count == 8:
    indexing = [0, 1, 2, 3, 4, 5, 6]
  else:
    return 0
    
  turn = 0
  joints = motiontarget.JointValues
  if joints[indexing[0]] < 0:
    turn += 1
  if joints[indexing[1]] < 0:
    turn += 2
  if joints[indexing[2]] < 0:
    turn += 4
  if joints[indexing[3]] < 0:
    turn += 8
  if joints[indexing[4]] < 0:
    turn += 16
  if joints[indexing[5]] < 0:
    turn += 32
  if joints[indexing[6]] < 0:
    turn += 64
  return turn


def get_e1(helper, motiontarget):
  #Get E1 value from motion target
  
  joints = motiontarget.JointValues
  if helper.robot_joint_count == 7:
    return (joints[6] * math.pi / 180.0)
  elif helper.robot_joint_count == 8:
    return (joints[2] * math.pi / 180.0)
  else:
    return 0


def get_base_name(name):
  # Format base name so that BASE_DATA[1] => BASE_1
  if name:
    exp = 'BASE_DATA\[(%s)\]' % (RE.INT)
    match = re.match(exp, name, RE.FLAGS)
    if match:
      name = 'BASE_%s' % (match.group(1))
  else:
    name = 'BASE_0'
  return name


def get_tool_name(name):
  # Format tool name so that TOOL_DATA[1] => TCP_1
  if name:
    exp = 'TOOL_DATA\[(%s)\]' % (RE.INT)
    match = re.match(exp, name, RE.FLAGS)
    if match:
      name = 'TCP_%s' % (match.group(1))
  else:
    name = 'TCP_0'
  return name


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
  
  # Boolean
  line = line.replace('True', 'true')
  line = line.replace('False', 'false')
  
  # Signals
  exp = 'IN\[(%s)\]' % (RE.INT)
  match = re.search(exp, line, RE.FLAGS)
  while match:
    line = line.replace(match.group(0), 'GetDI(%s).getBooleanIOValue()' % (match.group(1)))
    match = re.search(exp, line, RE.FLAGS)
  
  # Whitespace trims
  line = line.replace('  ', ' ')
  line = line.strip().rstrip()
  
  return line


def write_target(statement, helper, schema_index = -1):
  # Write motion statement position into data xml. Return position name.
  
  # Form position name
  if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
    pos = get_unique_name(statement.Positions[0].Name, helper.pos_names)
  elif statement.Type in [VC_STATEMENT_PATH]:
    pos = get_unique_name(statement.Name + '_P' + str(schema_index + 1), helper.pos_names)
  else:
    raise TranslatorException('Error in write_target.')
  helper.pos_names.append(pos)
  
  # Base
  base_name = get_base_name(helper.motiontarget.BaseName)
  if not base_name in helper.base_elements.keys():
    raise TranslatorException('Error in write_target.')
  base_el = helper.base_elements[base_name]
  
  # Tool
  tool_name = get_tool_name(helper.motiontarget.ToolName)
  if not tool_name in helper.tool_elements.keys():
    raise TranslatorException('Error in write_target.')
  tool_el = helper.tool_elements[tool_name]
  tool_tf_el = tool_el.find('transformation')
  if tool_tf_el is None:
    raise TranslatorException('Error in write_target.')
  
  # Create position frame element in data xml
  pos_el = create_frame_element(helper, base_el, pos, helper.motiontarget.Target)
  
  # Additional info
  add_info_el = ET.SubElement(pos_el, 'additionalInformation')
  create_last_modified_element(helper, add_info_el)
  info_el = ET.SubElement(add_info_el, 'info')
  info_el.set('base', 'Robot')
  info_el.set('controller', 'Controller')
  info_el.set('device', 'Robot')
  info_el.set('tcpName', 'SimulationTool/%s' % (tool_name))
  info_el.set('tool', 'SimulationTool')
  info_el.set('type', 'com.kuka.roboticsAPI.deviceModel.TeachInformation')
  tcp_tf_el = ET.SubElement(info_el, 'tcpTransformation')
  tcp_tf_el.set('x', tool_tf_el.get('x', '0.0'))
  tcp_tf_el.set('y', tool_tf_el.get('y', '0.0'))
  tcp_tf_el.set('z', tool_tf_el.get('z', '0.0'))
  tcp_tf_el.set('a', tool_tf_el.get('a', '0.0'))
  tcp_tf_el.set('b', tool_tf_el.get('b', '0.0'))
  tcp_tf_el.set('c', tool_tf_el.get('c', '0.0'))
  
  # Redundancy info
  if helper.robot_joint_count in [7, 8]:
    red_data_el = ET.SubElement(pos_el, 'redundancyData')
    red_el = ET.SubElement(red_data_el, 'redundancy')
    red_el.set('transformationProviderName', 'Controller/Robot')
    red_el.set('type', 'com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy')
    info_el = ET.SubElement(red_el, 'info')
    info_el.set('type', 'com.kuka.roboticsAPI.deviceModel.StatusTurnRedundancy$StatusParameter')
    info_el.set('value', '%i' % (get_status(helper, helper.motiontarget)))
    info_el.set('valueType', 'java.lang.Integer')
    info_el = ET.SubElement(red_el, 'info')
    info_el.set('type', 'com.kuka.roboticsAPI.deviceModel.StatusTurnRedundancy$TurnParameter')
    info_el.set('value', '%i' % (get_turn(helper, helper.motiontarget)))
    info_el.set('valueType', 'java.lang.Integer')
    info_el = ET.SubElement(red_el, 'info')
    info_el.set('type', 'com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy$E1Parameter')
    info_el.set('value', '%.3f' % (get_e1(helper, helper.motiontarget)))
    info_el.set('valueType', 'java.lang.Double')
  
  return pos


def write_break(statement, helper):
  helper.write_java('break;')


def write_call(statement, helper):
  rou = statement.getProperty('Routine').Value
  if rou:
    helper.write_java('%s();' % rou.Name)


def write_comment(statement, helper):
  helper.write_java('//%s' % statement.Comment)


def write_continue(statement, helper):
  helper.write_java('continue;')


def write_delay(statement, helper):
  helper.write_java('ThreadUtil.milliSleep(%.0f);' % (statement.Delay * 1000.0))


def write_halt(statement, helper):
  helper.write_java('getApplicationControl().halt();')


def write_if(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_java('if (%s) {' % condition)
  helper.depth += 1
  for s in statement.ThenScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_java('}')
  helper.write_java('else {')
  helper.depth += 1
  for s in statement.ElseScope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_java('}')


def write_lin_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  pos = write_target(statement, helper)
  base_name = get_base_name(helper.motiontarget.BaseName)
  motion = 'lin(data.getFrame("/%s/%s"))' % (base_name, pos)
  speed = '.setCartVelocity(%.0f)' % (helper.motiontarget.CartesianSpeed)
  blend = ''
  if helper.motiontarget.AccuracyValue > 0:
    blend = '.setBlendingCart(%.0f)' % (helper.motiontarget.AccuracyValue)
  move = 'robot.move(%s%s%s);' % (motion, speed, blend)
  helper.write_java(move)


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
  
  helper.write_java('//Path %s' % (statement.Name))
  for i in range(statement.getSchemaSize()):
    target = statement.getSchemaValue(i,'Position')
    helper.motiontarget.Target = target
    jv = helper.motiontarget.JointValues
    helper.motiontarget.JointValues = jv
    helper.motiontarget.CartesianSpeed = statement.getSchemaValue(i,'MaxSpeed')
    helper.motiontarget.CartesianAcceleration = statement.getSchemaValue(i,'Acceleration')
    helper.motiontarget.AccuracyMethod = statement.getSchemaValue(i,'AccuracyMethod')
    helper.motiontarget.AccuracyValue = statement.getSchemaValue(i,'AccuracyValue')
    
    pos = write_target(statement, helper, i)
    base_name = get_base_name(helper.motiontarget.BaseName)
    motion = 'lin(data.getFrame("/%s/%s"))' % (base_name, pos)
    speed = '.setCartVelocity(%.0f)' % (helper.motiontarget.CartesianSpeed)
    blend = ''
    if helper.motiontarget.AccuracyValue > 0:
      blend = '.setBlendingCart(%.0f)' % (helper.motiontarget.AccuracyValue)
    move = 'robot.move(%s%s%s);' % (motion, speed, blend)
    helper.write_java(move)
  #endfor
  helper.write_java('//Endpath')


def write_print(statement, helper):
  helper.write_java('logger.info("%s");' % statement.Message)


def write_ptp_motion(statement, helper):
  statement.writeToTarget(helper.motiontarget)
  pos = write_target(statement, helper)
  base_name = get_base_name(helper.motiontarget.BaseName)
  motion = 'ptp(data.getFrame("/%s/%s"))' % (base_name, pos)
  speed = '.setJointAccelerationRel(%.2f)' % (helper.motiontarget.JointSpeedFactor)
  blend = ''
  if helper.motiontarget.AccuracyValue > 0:
    blend = '.setBlendingCart(%.0f)' % (helper.motiontarget.AccuracyValue)
  move = 'robot.move(%s%s%s);' % (motion, speed, blend)
  helper.write_java(move)


def write_return(statement, helper):
  helper.write_java('return;')


def write_set_bin(statement, helper):
  val = 'false'
  if statement.OutputValue:
    val = 'true'
  helper.write_java('GetDO(%i).setBooleanValue(%s);' % (statement.OutputPort, val))


def write_set_property(statement, helper):
  #Check if target property is declared in locals or globals
  if not statement.TargetProperty in helper.local_vars and not statement.TargetProperty in helper.global_vars:
    #Not declared, probably a global (component property)
    p = helper.component.getProperty(statement.TargetProperty)
    if p:
      #Declare as global
      types = {VC_BOOLEAN : 'boolean', VC_INTEGER : 'int', VC_REAL : 'float', VC_STRING : 'String'}
      if p.Type in types:
        type = types[p.Type]
        value = str(p.Value)
        if p.Type == VC_BOOLEAN:
          value = value.lower()
        elif p.Type == VC_STRING:
          value = '"%s"' % (value)
        elif p.Type == VC_REAL:
          value += 'f'
        helper.write_globals('%s %s = %s;' % (type, p.Name, value))
        helper.global_vars.append(p.Name)
  
  target = check_expression(statement.TargetProperty, helper)
  value_expression = check_expression(statement.ValueExpression, helper)
  helper.write_java('%s = %s;' % (target, value_expression))


def write_wait_bin(statement, helper):
  val = 'false'
  if statement.InputValue:
    val = 'true'
  helper.write_java('getObserverManager().waitFor(new BooleanIOCondition(GetDI(%i), %s));' % (statement.InputPort, val))


def write_while(statement, helper):
  condition = check_expression(statement.Condition, helper)
  helper.write_java('while (%s) {' % condition)
  helper.depth += 1
  for s in statement.Scope.Statements:
    translator = helper.statement_translators.get(s.Type, unknown)
    translator(s, helper)
  helper.depth -= 1
  helper.write_java('}')


def unknown(statement, helper):
  print '> Unsupported statement type skipped:', statement.Type
