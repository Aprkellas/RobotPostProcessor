#Version 1.03

from vcCommand import *
import vcMatrix
import time, os.path
import re
import locale

locale.setlocale(locale.LC_NUMERIC,'C') # This is to keep all numeric conversions consistent across platform locales.


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
  VAR = r'[a-z][a-z0-9]*\[[0-9]+\]|[a-z][a-z0-9]*'
  ARRAY = r'[a-z][a-z0-9]*\[[0-9]+\]'
  FLAGS = re.I|re.S


class TranslatorHelper:
  #---
  # Capsulates commonly used handles and post processor settings
  #---
  def __init__(self, app, program, uri):
    
    self.cmd = getCommand()
    
    # 
    self.data =    '' # Program module lines
    
    # Helpers
    self.app = app
    self.app_version = GetAppVersion(self.app)
    self.uri = uri
    self.program = program
    self.controller = program.Executor.Controller
    self.component = program.Executor.Component
    self.motiontarget = self.controller.createTarget()
    
    self.robot_joint_count = len([x for x in self.controller.Joints if not x.ExternalController])
    self.total_joint_count = len(self.controller.Joints)
    self.extaxisunit = []
    extjc = 0
    for j in self.controller.Joints:
      if j.ExternalController:
        if j.Type==VC_JOINT_TRANSLATIONAL:
          self.extaxisunit.append("mm")
        else:
          self.extaxisunit.append("deg")
        extjc += 1
    self.num_of_groups = 1
    self.groupmask = ''
    self.ext_axis_groups = {'ExtAxisGroup1' : 1,
       'ExtAxisGroup2' : 1,
       'ExtAxisGroup3' : 1,
       'ExtAxisGroup4' : 1,
       'ExtAxisGroup5' : 1,
       'ExtAxisGroup6' : 1,
       'ExtAxisGroup7' : 1,
       'ExtAxisGroup8' : 1,
       'ExtAxisGroup9' : 1,
       'ExtAxisGroup10' : 1,
       'ExtAxisGroup11' : 1,
       'ExtAxisGroup12' : 1,
       'ExtAxisGroup13' : 1,
       'ExtAxisGroup14' : 1,
       'ExtAxisGroup15' : 1,
       'ExtAxisGroup16' : 1,
       'ExtAxisGroup17' : 1,
       'ExtAxisGroup18' : 1,
       'ExtAxisGroup19' : 1,
       'ExtAxisGroup20' : 1 }
    
    self.indentation = '  '
    self.depth = 0
    
    self.keywords = ['NOT', 'AND', 'OR', 'ON', 'OFF', 'DI', 'DO', 'DIV', 'MOD']
    self.variables = {}
    self.current_routine = None
    self.current_uf = -1
    self.current_ut = -1
    self.point_count = 0
    self.statement_count = 0
    self.label_index = 1
    self.while_label_stack = []
    
    self.statement_translators = {
      VC_STATEMENT_BREAK:WriteBreak,
      VC_STATEMENT_CALL:WriteCall,
      VC_STATEMENT_COMMENT:WriteComment,
      VC_STATEMENT_CONTINUE:WriteContinue,
      VC_STATEMENT_DEFINE_BASE:WriteDefineBase,
      VC_STATEMENT_DEFINE_TOOL:WriteDefineTool,
      VC_STATEMENT_DELAY:WriteDelay,
      VC_STATEMENT_HALT:WriteHalt,
      VC_STATEMENT_IF:WriteIf,
      VC_STATEMENT_LINMOTION:WriteLinMotion,
      VC_STATEMENT_PATH:WritePath,
      VC_STATEMENT_PRINT:WritePrint,
      VC_STATEMENT_PROG_SYNC:Unhandled,
      VC_STATEMENT_PTPMOTION:WritePtpMotion,
      VC_STATEMENT_RETURN:WriteReturn,
      VC_STATEMENT_SETBIN:WriteSetBin,
      VC_STATEMENT_SETPROPERTY:WriteSetProperty,
      VC_STATEMENT_WAITBIN:WriteWaitBin,
      VC_STATEMENT_WHILE:WriteWhile }
    if self.app_version[0] > 4 or (self.app_version[0] == 4 and self.app_version[1] >= 4):
      #4.4 or newer
      self.statement_translators[VC_STATEMENT_STATISTICS] = Unhandled
      self.statement_translators[VC_STATEMENT_SWITCHCASE] = WriteSwitchCase
    
  
  
  def init_data(self, routine = None):
    self.data = ''
    self.current_routine = routine
    self.current_uf = -1
    self.current_ut = -1
    self.point_count = 0
    self.statement_count = 0
  
  def write_line(self, input):
    self.data += input + '\n'
  
  def write_statement(self, input, motype = ''):
    self.statement_count += 1
    if not motype:
      self.data += ('%4i:  %s%s\n' % (self.statement_count, self.current_indent(), input))
    else:
      self.data += ('%4i:%s %s%s\n' % (self.statement_count, motype, self.current_indent(), input))
  
  def replace_data(self, old_text, new_text):
    self.data = self.data.replace(old_text, new_text)
  
  def current_indent(self):
    return self.indentation * self.depth


def postProcess(app, program, uri):
  
  helper = TranslatorHelper(app, program, uri)
  head, tail = os.path.split(helper.uri)
  mainName = tail[:len(tail)-3]
  
  filenamelist=[]
  
  # Main routine
  filenamelist.append(uri)
  if not WriteProgramBody(helper, program.MainRoutine, mainName, uri):
    return False, filenamelist
    
  # Subroutines
  for routine in program.Routines:
    filename = head + "\\" + routine.Name + ".ls"
    filenamelist.append(filename)
    if not WriteProgramBody(helper, routine, routine.Name, filename):
      return False, filenamelist
  
  # Frames subroutine
  name = 'SET_FRAMES'
  filename = head + "\\%s.ls" % name
  filenamelist.append(filename)
  if not WriteFrames(helper, name, filename, program):
    return False,filenamelist
  
  CheckProgram(helper)
  
  return True, filenamelist


def getDefaultJobName():
  return 'MAIN.LS'


def CheckProgram(helper):
  # Check program and print some warnings
  
  stmts = GetAllStatements(helper.program)
  relative_frame_found = False
  else_if_found = False
  
  for stmt in stmts:
    if not relative_frame_found:
      # Check for relative SetBase/SetTool
      if stmt.Type in [VC_STATEMENT_DEFINE_BASE, VC_STATEMENT_DEFINE_BASE]:
        if stmt.IsRelative:
          relative_frame_found = True
          print 'INFO: Relative SetBase/SetTool needs matrix mutliplication. Include "Vision support tools" option and set $KAREL_ENB to 1.'


def WriteProgramBody(helper, routine, name, filename):
  
  helper.init_data(routine)
  
  WriteJobHeader(helper, name)
  
  if routine == routine.Program.MainRoutine:
    # Call SET_FRAMES at the start of main
    helper.write_statement('!SET UTOOLS AND UFRAMES ;')
    helper.write_statement('CALL SET_FRAMES ;')
    helper.write_statement(';')
  
  for statement in routine.Statements:
    WriteStatement(helper, statement)
    
  # Collect all position definitions
  helper.write_line('/POS')
  helper.point_count = 0
  for statement in routine.Statements:
    WriteTargetDefinition(helper, statement)
  
  WriteJobFooter(helper)
  
  
  try:
    file = open(filename, 'w')
    file.write(helper.data)
    file.close()
  except:
    print "Cannot open file \'%s\' for writing" % filename
    return False
  
  return True


def WriteFrames(helper, name, filename, program):
  
  helper.init_data()
  
  WriteJobHeader(helper, name)
  
  all_statements = GetAllStatements(helper.program)
  used_tools = []
  used_bases = []
  swap_base_tool = {}
  swap_tool_base = {}
  for statement in all_statements:
    if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      if BaseToolSwapped(helper, statement) and statement.Base and statement.Tool:
        # External TCP by frame swapping
        swap_base_tool[statement.Base.Name] = statement.Tool.Name
        swap_tool_base[statement.Tool.Name] = statement.Base.Name
        if not statement.Base in used_tools:
          used_tools.append(statement.Base)
        if not statement.Tool in used_bases:
          used_bases.append(statement.Tool)
      else:
        # Rgeular TCP
        if not statement.Base in used_bases:
          used_bases.append(statement.Base)
        if not statement.Tool in used_tools:
          used_tools.append(statement.Tool)
  
  
  if None in used_bases:
    used_bases.remove(None)
  used_bases = sorted(used_bases, key=lambda base: base.Name)
  
  for base in used_bases:
    base_name = 'Null'
    if base:
      base_name = base.Name
    if base_name in swap_tool_base:
      base_name = swap_tool_base[base_name]
    helper.motiontarget.BaseName = base_name
    uf = GetBaseIndex(helper.controller, helper.motiontarget)
    if uf < 0 or uf > 9:
      print 'WARNING: Base %s not supported. Use NULL or UFRAME1-9.' % (base_name)
    
    if not base:
      continue
    
    if base.Node:
      irobot_world_pos = helper.motiontarget.getSimWorldToRobotWorld()
      irobot_world_pos.invert()
      base_world_pos = base.Node.WorldPositionMatrix * base.PositionMatrix
      base_pos = irobot_world_pos * base_world_pos
    else:
      base_pos = base.PositionMatrix
    
    frame_name = 'UFRAME[%i]' % uf
    
    WriteFrame(helper, frame_name, base_pos)
  
  
  if None in used_tools:
    used_tools.remove(None)
    print 'WARNING: Tool Null not supported. Use UTOOL1-10.'
  used_tools = sorted(used_tools, key=lambda tool: tool.Name)
  
  for tool in used_tools:
    tool_name = 'Null'
    if tool:
      tool_name = tool.Name
    if tool_name in swap_base_tool:
      tool_name = swap_base_tool[tool_name]
    helper.motiontarget.ToolName = tool_name
    ut = GetToolIndex(helper.controller, helper.motiontarget)
    if ut <= 0 or ut > 10:
      print 'WARNING: Tool %s not supported. Use UTOOL1-10.' % (tool_name)
    
    if not tool:
      continue
    
    if tool.Node and tool.Node != helper.controller.FlangeNode:
      iflange_world_pos = helper.controller.FlangeNode.WorldPositionMatrix
      tool_world_pos = tool.Node.WorldPositionMatrix * tool.PositionMatrix
      tool_pos = iflange_world_pos * tool_world_pos
    else:
      tool_pos = tool.PositionMatrix
    
    frame_name = 'UTOOL[%i]' % ut
    
    WriteFrame(helper, frame_name, tool_pos)
  
  
  WriteJobFooter(helper)
  
  try:
    file = open(filename,"w")
    file.write(helper.data)
    file.close()
  except:
    print "Cannot open file \'%s\' for writing" % filename
    return False
  return True


def WriteFrame(helper, frame_name, pos, is_relative = False):
  pr = 1
  if is_relative:
    pr = 2
  p = pos.P
  wpr = pos.WPR
  
  helper.write_statement('PR[%i] = LPOS;' % (pr))
  helper.write_statement('PR[%i,1] = %.3f;' % (pr, p.X))
  helper.write_statement('PR[%i,2] = %.3f;' % (pr, p.Y))
  helper.write_statement('PR[%i,3] = %.3f;' % (pr, p.Z))
  helper.write_statement('PR[%i,4] = %.4f;' % (pr, wpr.X))
  helper.write_statement('PR[%i,5] = %.4f;' % (pr, wpr.Y))
  helper.write_statement('PR[%i,6] = %.4f;' % (pr, wpr.Z))
  
  if is_relative:
    helper.write_statement('PR[1]=%s ;' % (frame_name))
    helper.write_statement('CALL MATRIX(1,2,1);')
  
  helper.write_statement('%s=PR[1] ;' % (frame_name))


def WriteJobHeader(helper, name):
  # Headers
  td = time.strftime('DATE %y-%m-%d  TIME %H:%M:%S')
  helper.write_line('/PROG %s' % name )
  helper.write_line('/ATTR')
  helper.write_line('OWNER = MNEDITOR;')
  helper.write_line('COMMENT = \"VC generated Program\";')
  helper.write_line('PROG_SIZE = 64000;')
  helper.write_line('CREATE = %s;' % td)
  helper.write_line('MODIFIED = %s;' % td)
  helper.write_line('FILE_NAME = %s;' % name )
  helper.write_line('VERSION = 0;')
  helper.write_line('LINE_COUNT = __LINE_COUNT__;' )  # Placeholder __LINE_COUNT__ replaced later
  helper.write_line('MEMORY_SIZE = 64000;')
  helper.write_line('PROTECT = READ_WRITE;')
  helper.write_line('TCD: STACK_SIZE = 0,')
  helper.write_line('     TASK_PRIORITY = 50,')
  helper.write_line('     TIME_SLICE = 0,')
  helper.write_line('     BUSY_LAMP_OFF = 0,')
  helper.write_line('     ABORT_REQUEST = 0,')
  helper.write_line('     PAUSE_REQUEST = 0;')
  
  # Group definitions
  line = 'DEFAULT_GROUP = 1'
  helper.groupmask=['*', '*', '*', '*', '*']
  # Loop through all external axis group definitions and set the the group mask
  for axisindex in range(1, 21):
    groupindex = helper.ext_axis_groups['ExtAxisGroup' + str(axisindex)]
    if groupindex > 0:
      helper.groupmask[groupindex - 1] = '1'
  for i in range(1, 5):
    line += ',' + helper.groupmask[i]
  line += ';'
  helper.write_line(line)
  helper.num_of_groups = helper.groupmask.count('1')
  helper.write_line('CONTROL_CODE = 00000000 00000000;')
  
  # Main
  helper.write_line('/MN')


def WriteJobFooter(helper):
  # Footers
  helper.write_line('/END')
  helper.replace_data('__LINE_COUNT__', str(helper.statement_count))


def WriteTargetDefinition(helper, statement):

  if statement.Type == VC_STATEMENT_LINMOTION or statement.Type == VC_STATEMENT_PTPMOTION:
    statement.writeToTarget(helper.motiontarget)
    DoWriteTargetDefinition(helper, statement)
  elif statement.Type == VC_STATEMENT_PATH:
    
    base_name = 'Null'
    if statement.Base:
      base_name = statement.Base.Name
    tool_name = 'Null'
    if statement.Tool:
      tool_name = statement.Tool.Name
    helper.motiontarget.BaseName = base_name
    helper.motiontarget.ToolName = tool_name
    size = statement.getSchemaSize()
    for i in range(size):
      pos = statement.getSchemaValue(i, 'Position')
      helper.motiontarget.Target = pos
      DoWriteTargetDefinition(helper, statement)
    
  elif statement.Type == VC_STATEMENT_WHILE:
    for child in statement.Scope.Statements:
      WriteTargetDefinition(helper, child)
  elif statement.Type == VC_STATEMENT_IF:
    for child in statement.ThenScope.Statements:
      WriteTargetDefinition(helper, child)
    if helper.app_version[0] > 4 or (helper.app_version[0] == 4 and helper.app_version[1] >= 4):
      for else_if_scope in statement.ElseIfScopes:
        for child in else_if_scope.Statements:
          WriteTargetDefinition(helper, child)
    for child in statement.ElseScope.Statements:
      WriteTargetDefinition(helper, child)
  elif helper.app_version[0] > 4 or (helper.app_version[0] == 4 and helper.app_version[1] >= 4):
    if statement.Type == VC_STATEMENT_SWITCHCASE:
      for case_scope in statement.Cases:
        for child in case_scope.Statements:
          WriteTargetDefinition(helper, child)


def DoWriteTargetDefinition(helper, statement):
  
  helper.point_count += 1
  sn = statement.Name
  c = GetConfigs(helper.motiontarget)
  uf = GetBaseIndex(helper.controller, helper.motiontarget)
  ut = GetToolIndex(helper.controller, helper.motiontarget)
  helper.write_line('P[%i]{ ' % helper.point_count )
  helper.write_line('    GP1:' )
  line = '         UF : %i, UT : %i,' % (uf, ut)
  if helper.motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT and uf == 0 and helper.total_joint_count == helper.robot_joint_count:
    helper.write_line(line)
    j = helper.motiontarget.JointValues
    line = '         '
    for (i, jx) in enumerate(j):
      line = line + 'J%i= %8.2f deg' % ((i + 1), jx)
      if i < len(j) - 1:
        line = line + ','
        if (i + 1) % 3 == 0:
          helper.write_line(line)
          line = '         '
        else:
          line += '      '
    helper.write_line(line)
    helper.write_line('};')
  else:
    tgt = helper.motiontarget.Target
    if statement.ExternalTCP or BaseToolSwapped(helper, statement):
      tgt.invert()
    p = tgt.P
    a = tgt.getWPR()
    line += '              CONFIG : %s' % c 
    helper.write_line(line)
    helper.write_line('         X = %8.2f  mm,     Y = %8.2f  mm,     Z = %8.2f  mm,' % (p.X,p.Y,p.Z))
    line = '         W = %8.2f deg,     P = %8.2f deg,     R = %8.2f deg' % (a.X,a.Y,a.Z)
    
    externalAxes = helper.total_joint_count - helper.robot_joint_count
    if externalAxes > 0 and helper.num_of_groups > 0:
      if helper.num_of_groups == 1:
        line += ','
        helper.write_line(line)
        line = '    '
        for j in range(externalAxes):
          if j > 0:
            line += ','
            if j % 3 == 0:
              helper.write_line(line)
              line = '    '
          line += '     E%i = %8.2f %s' % (j + 1, helper.motiontarget.JointValues[helper.robot_joint_count + j], helper.extaxisunit[j])
      else:
        line += ','
        helper.write_line(line)
        line = '    '
        for groupnum in range(2,6):
          if helper.groupmask[groupnum - 1] == '1':
            helper.write_line(line)
            helper.write_line('    GP%i:' % groupnum)
            helper.write_line('         UF : %i, UT : %i,' % (uf, ut) )
            line = '    '
            
            jcount = 0
            for j in range(externalAxes):
              if groupnum == helper.ext_axis_groups['ExtAxisGroup' + str(j + 1)]:
                if jcount > 0:
                  line += ','
                  if jcount % 3 == 0:
                    line += ','
                    helper.write_line(line)
                    line = '      '
                line += '     J%i = %8.2f %s' % (jcount + 1, helper.motiontarget.JointValues[helper.robot_joint_count + j], helper.extaxisunit[j])
                jcount += 1
    helper.write_line(line)
    helper.write_line('};')


def WriteCurrentFrames(helper, statement):
  # Check frames
  if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
    base_name = 'Null'
    if statement.Base:
      base_name = statement.Base.Name
    helper.motiontarget.BaseName = base_name
    uf = GetBaseIndex(helper.controller, helper.motiontarget)
    if uf != helper.current_uf:
      helper.write_statement('UFRAME_NUM=%i;' % (uf))
      helper.current_uf = uf
  
    tool_name = 'Null'
    if statement.Tool:
      tool_name = statement.Tool.Name
    helper.motiontarget.ToolName = tool_name
    ut = GetToolIndex(helper.controller, helper.motiontarget)
    if ut != helper.current_ut:
      helper.write_statement('UTOOL_NUM=%i;' % (ut))
      helper.current_ut = ut


def WriteStatement(helper, statement):
  if statement.Type in helper.statement_translators:
    helper.statement_translators[statement.Type](helper, statement)
  else:
    Unhandled(helper, statement)


def WriteBreak(helper, statement):
  if helper.while_label_stack:
    helper.write_statement('JMP LBL[%i] ;' % (helper.while_label_stack[-1] + 1))


def WriteCall(helper, statement):
  helper.write_statement('CALL %s ;' % (statement.getProperty("Routine").Value.Name) )


def WriteComment(helper, statement):
  c = statement.Comment
  helper.write_statement('!%s;' % (c))


def WriteContinue(helper, statement):
  if helper.while_label_stack:
    helper.write_statement('JMP LBL[%i] ;' % (helper.while_label_stack[-1]))


def WriteDefineBase(helper, statement):
  base = statement.Base
  base_name = 'Null'
  if base:
    base_name = base.Name
  helper.motiontarget.BaseName = base_name
  uf = GetBaseIndex(helper.controller, helper.motiontarget)
  if base:
    if not statement.IsRelative:
      if statement.Node:
        irobot_world_pos = helper.motiontarget.getSimWorldToRobotWorld()
        irobot_world_pos.invert()
        base_world_pos = statement.Node.WorldPositionMatrix * statement.Position
        base_pos = irobot_world_pos * base_world_pos
      else:
        base_pos = statement.Position
    else:
      base_pos = statement.Position
    frame_name = 'UFRAME[%i]' % uf
    WriteFrame(helper, frame_name, base_pos, statement.IsRelative)


def WriteDefineTool(helper, statement):
  tool = statement.Tool
  tool_name = 'Null'
  if tool:
    tool_name = tool.Name
  helper.motiontarget.ToolName = tool_name
  ut = GetToolIndex(helper.controller, helper.motiontarget)
  if tool:
    if statement.Node and statement.Node != helper.controller.FlangeNode:
      iflange_world_pos = helper.controller.FlangeNode.WorldPositionMatrix
      tool_world_pos = statement.Node.WorldPositionMatrix * statement.Position
      tool_pos = iflange_world_pos * tool_world_pos
    else:
      tool_pos = statement.Position
    frame_name = 'UTOOL[%i]' % ut
    statementCount = WriteFrame(helper, frame_name, tool_pos, statement.IsRelative)


def WriteDelay(helper, statement):
  d = statement.Delay
  helper.write_statement('WAIT %6.2f(sec) ;' % (d))


def WriteHalt(helper, statement):
  helper.write_statement('PAUSE ;')


def WriteIf(helper, statement):
  condition = CheckExpression(helper, statement.Condition)
  helper.write_statement('IF (%s) THEN ;' % (condition))
  helper.depth = helper.depth + 1
  for child in statement.ThenScope.Statements:
    WriteStatement(helper, child)
  helper.depth = helper.depth - 1
  
  end_label_index = -1
  helper.write_statement('ELSE ;')
  helper.depth = helper.depth + 1
  
  if helper.app_version[0] > 4 or (helper.app_version[0] == 4 and helper.app_version[1] >= 4):
    #4.4 or newer, elseif scopes. Use Jump to label with ifs to form same logic.
    if statement.ElseIfScopes:
      end_label_index = helper.label_index
      helper.label_index += 1
      for else_if_scope in statement.ElseIfScopes:
        condition = CheckExpression(helper, else_if_scope.Condition)
        helper.write_statement('IF (%s) THEN ;' % (condition))
        helper.depth = helper.depth + 1
        for child in else_if_scope.Statements:
          WriteStatement(helper, child)
        helper.write_statement('JMP LBL[%i] ;' % (end_label_index))
        helper.depth = helper.depth - 1
        helper.write_statement('ENDIF ;')
  
  for child in statement.ElseScope.Statements:
    WriteStatement(helper, child)
  helper.depth = helper.depth - 1
  helper.write_statement('ENDIF ;')
  if end_label_index >= 0:
    helper.write_statement('LBL[%i] ;' % (end_label_index))


def WriteLinMotion(helper, statement):
  WriteCurrentFrames(helper, statement)
  statement.writeToTarget(helper.motiontarget)
  helper.point_count += 1
  pos_lv = 'FINE'
  if statement.AccuracyValue > 0:
    pos_lv = 'CNT%.0f' % (statement.AccuracyValue)
  rtcp = ''
  if statement.ExternalTCP or BaseToolSwapped(helper, statement):
    rtcp = ' RTCP'
  helper.write_statement('P[%i: %s]  %gmm/sec %s%s    ;' % (helper.point_count, statement.Positions[0].Name, statement.MaxSpeed, pos_lv, rtcp), 'L')


def WritePath(helper, statement):
  WriteCurrentFrames(helper, statement)
  rtcp = ''
  if statement.ExternalTCP or BaseToolSwapped(helper, statement):
    rtcp = ' RTCP'
  size = statement.getSchemaSize()
  get_speed = False
  get_pos_lv = False
  props = statement.SchemaProperties
  for prop in props:
    if prop.Name == 'AccuracyValue':
      get_pos_lv = True
    elif prop.Name == 'MaxSpeed':
      get_speed = True
  for i in range(size):
    helper.point_count += 1
    name = statement.Name + '_' + str(i+1)
    speed = 100.0
    pos_lv = 'FINE'
    if get_speed:
      speed = statement.getSchemaValue(i, 'MaxSpeed')
    if get_pos_lv:
      accuracy = statement.getSchemaValue(i, 'AccuracyValue')
      if accuracy > 0:
        pos_lv = 'CNT%.0f' % (accuracy)
    helper.write_statement('P[%i: %s]  %gmm/sec %s%s    ;' % (helper.point_count, name, speed, pos_lv, rtcp), 'L')


def WritePrint(helper, statement):
  helper.write_statement('Message[%s] ;' % statement.Message)


def WritePtpMotion(helper, statement):
  WriteCurrentFrames(helper, statement)
  statement.writeToTarget(helper.motiontarget)
  helper.point_count += 1
  pos_lv = 'FINE'
  if statement.AccuracyValue > 0:
    pos_lv = 'CNT%.0f' % (statement.AccuracyValue)
  helper.write_statement('P[%i: %s]  %g%% %s    ;' % (helper.point_count, statement.Positions[0].Name, statement.JointSpeed * 100.0, pos_lv), 'J')


def WriteReturn(helper, statement):
  helper.write_statement('END ;')


def WriteSetBin(helper, statement):
  value = 'OFF'
  if statement.OutputValue:
    value = 'ON'
  helper.write_statement('DO[%i]= %s ;' % (statement.OutputPort, value))


def WriteSetProperty(helper, statement):
  helper.write_statement('%s=%s ;' % (CheckExpression(helper, statement.TargetProperty), CheckExpression(helper, statement.ValueExpression)))


def WriteSwitchCase(helper, statement):
  
  start_label_index = helper.label_index
  end_label_index = start_label_index + len(statement.Cases)
  helper.label_index += len(statement.Cases) + 1
  
  label_index = start_label_index
  for i, case in enumerate(statement.Cases):
    if i == 0:
      condition = CheckExpression(helper, statement.Condition)
      case_condition = CheckExpression(helper, case.CaseCondition)
      helper.write_statement('SELECT %s=%s,JMP LBL[%i] ;' % (condition, case_condition, label_index))
    elif case.CaseCondition.lower() != 'default':
      case_condition = CheckExpression(helper, case.CaseCondition)
      helper.write_statement('       =%s,JMP LBL[%i] ;' % (case_condition, label_index))
    else:
      helper.write_statement('       ELSE,JMP LBL[%i] ;' % (label_index))
    label_index += 1
  
  label_index = start_label_index
  for case in statement.Cases:
    helper.write_statement('LBL[%i] ;' % (label_index))
    for child in case.Statements:
      WriteStatement(helper, child)
    helper.write_statement('JMP LBL[%i] ;' % (end_label_index))
    label_index += 1
  
  helper.write_statement('LBL[%i] ;' % (end_label_index))


def WriteWaitBin(helper, statement):
  value = 'OFF'
  if statement.InputValue:
    value = 'ON'
  helper.write_statement('WAIT DI[%i]= %s ;' % (statement.InputPort, value))


def WriteWhile(helper, statement):
  label_index = helper.label_index
  helper.label_index += 2
  helper.while_label_stack.append(label_index)
  helper.write_statement('LBL[%i] ;' % (label_index))
  for child in statement.Scope.Statements:
    WriteStatement(helper, child)
  if statement.Condition.lower().strip() == 'true':
    helper.write_statement('JMP LBL[%i] ;' % (label_index))
  else:
    condition = CheckExpression(helper, statement.Condition)
    helper.write_statement('IF (%s), JMP LBL[%i] ;' % (condition, label_index))
  helper.write_statement('LBL[%i] ;' % (label_index + 1))
  helper.while_label_stack.pop()


def Unhandled(helper, statement):
  helper.write_statement('!Unsupported type %s;' % (indent, statement.Type))


def CheckExpression(helper, exp):
  #Check expression for variables and try to create those variables if needed. Also check some literal formatting.
  #ATM support simple comparisons between numbers and boolean variables
  
  if ('IN' in exp) or ('OUT' in exp):
    #Format boolean literals in IO expressions
    exp = exp.replace('True', 'ON')
    exp = exp.replace('False', 'OFF')
    exp = exp.replace('IN', 'DI')
    exp = exp.replace('OUT', 'DO')
  else:
    #Format boolean literals in other expressions
    exp = exp.replace('True', '1')
    exp = exp.replace('False', '0')
  
  #Format string
  exp = exp.replace('"', "'")
  
  #Format operators and specials
  exp = exp.replace('==', '=')
  exp = exp.replace('!=', '<>')
  exp = exp.replace('&&', ' AND ')
  exp = exp.replace('&', ' AND ')
  exp = exp.replace('||', ' OR ')
  exp = exp.replace('|', ' OR ')
  exp = exp.replace('/', ' DIV ')
  exp = exp.replace('%', ' MOD ')
  
  #Format variables
  exp_left = exp
  exp = ''
  match = re.search(RE.VAR, exp_left, RE.FLAGS)
  while match:
    exp += exp_left[:match.start()]
    var_name = match.group(0)
    if not var_name in helper.keywords:
      if var_name in helper.variables.keys():
        var_name = helper.variables[var_name]
      else:
        #Find property, convert name to proper register
        match2 = re.match('(S?R%s)|(S?R\[%s\])' % (RE.INT, RE.INT), var_name, RE.FLAGS)
        if match2:
          #Already named as register (e.g. R1 or SR[2]), use that
          new_var_name = match2.group(0)
          if match2.group(1):
            match2 = re.match('(S?R)(%s)' % (RE.INT), var_name, RE.FLAGS)
            if match2:
              new_var_name = '%s[%s]' % (match2.group(1), match2.group(2))
        else:
          #Find property type and convert name to register
          type = VC_INTEGER
          prop = helper.current_routine.getProperty(var_name)
          if not prop:
            prop = helper.component.getProperty(var_name)
          if prop:
            type = prop.Type
          prefix = 'R'
          if type == VC_STRING:
            prefix = 'SR'
          for i in range(1, 9999):
            new_var_name = '%s[%s]' % (prefix, str(i))
            if not new_var_name in helper.variables.values():
              break
        helper.variables[var_name] = new_var_name
        var_name = new_var_name
    exp += var_name
    exp_left = exp_left[match.end():]
    match = re.search(RE.VAR, exp_left, RE.FLAGS)
  exp += exp_left
  
  return exp


def BaseToolSwapped(helper, statement):
  # Check if statement is using external tcp by swapping base and tool frames
  
  tool_on_robot = False
  base_on_robot = False
  
  tool = statement.Tool
  if not tool or not tool.Node:
    tool_on_robot = True
  else:
    node = tool.Node
    while node != helper.app.Simulation.World:
      if node == helper.controller.FlangeNode:
        tool_on_robot = True
        break
      node = node.Parent
  
  base = statement.Base
  if base and base.Node:
    node = base.Node
    while node != helper.app.Simulation.World:
      if node == helper.controller.FlangeNode:
        base_on_robot = True
        break
      node = node.Parent
  
  return (not tool_on_robot and base_on_robot)


def GetAllStatements(program):
  allstatements = []
  for s in program.MainRoutine.Statements:
    allstatements = ExtendStatements(s,allstatements)
  for routine in program.Routines:
    for s in routine.Statements:
      allstatements = ExtendStatements(s,allstatements)
  return allstatements


def ExtendStatements(statement, statements):
  statements.append(statement)
  if statement.Type == VC_STATEMENT_WHILE:
    for s in statement.Scope.Statements:
      statements = ExtendStatements(s, statements)
  elif statement.Type == VC_STATEMENT_IF:
    for s in statement.ThenScope.Statements:
      statements = ExtendStatements(s, statements)
    for s in statement.ElseScope.Statements:
      statements = ExtendStatements(s, statements)
  return statements


def GetToolIndex(controller, motiontarget):
  i = 0
  for t in controller.Tools:
    i += 1
    if t.Name == motiontarget.ToolName:
      return i
  return 0


def GetBaseIndex(controller, motiontarget):
  i = 0
  for b in controller.Bases:
    i += 1
    if b.Name == motiontarget.BaseName:
      return i
  return 0


def GetConfigs(motiontarget):
  rconf = motiontarget.RobotConfig
  if rconf == 0:
    c = "\'N U T"
  elif rconf == 1:
    c = "\'F U T"
  elif rconf == 2:
    c = "\'N D T"
  elif rconf == 3:
    c = "\'F D T"
  elif rconf == 4:
    c = "\'N U B"
  elif rconf == 5:
    c = "\'F U B"
  elif rconf == 6:
    c = "\'N D B"
  elif rconf == 7:
    c = "\'F D B"
  #if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
  # check this out!
  j = motiontarget.JointValues
  t =""
  if len(j) >= 6:
    for jt in [j[3],j[4],j[5]]:
      if jt <-179.0:
        t+=", -1"
      elif jt > 179.0:
        t+=", 1"
      else:
        t+=", 0"
  t+="\',"   
  #t = ", %i, %i, %i" %(motiontarget.JointTurns4, 0, motiontarget.JointTurns6)
  #t = ", %i, %i, %i" %(motiontarget.JointTurns4, motiontarget.JointTurns5, motiontarget.JointTurns6)
  # t = ", 0, 0, 0\',"
  #else:
  #  t = ", 0, 0, 0\',"
  return c + t


def GetAppVersion(app):
  platform = 4
  major = 0
  minor = 0
  version_string = app.ProductVersion
  args = version_string.split('.')
  if len(args) >= 3:
    try:
      platform = int(args[0])
      major = int(args[1])
      minor = int(args[2])
    except:
      pass
  return (platform, major, minor)