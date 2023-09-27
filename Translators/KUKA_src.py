#-----
#KUKA KRL post-processor for VC4.X products
#-----
#Version 1.13
#-----

from vcCommand import *
import vcMatrix, vcVector
import time, os.path
import re
import locale

#-------------------------------------------------------------------------------
def postProcess(appx,progx,uri):
  global app, cmd, prog, app_version
  global use_spline_motions, use_inline_form, comment_out_frames
  global ctr, motiontarget
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  global base_dict, tool_dict, ex_tcp_pairs
  global c_tool, c_base, c_velaxis, c_vel_cp, c_apodist, c_bwdstart, c_ipoframe
  global pos_names
  
  app = appx
  cmd = getCommand()
  prog = progx
  app_version = 4.0
  try:
    app_version = float(app.ProductVersion[:app.ProductVersion.rfind('.')])
  except:
    pass
  
  #This is to keep all numeric conversions consistent across platform locales.
  locale.setlocale(locale.LC_NUMERIC,'C')
  
  #file name
  head, tail = os.path.split(uri)
  main_name = tail[:len(tail)-4]
  
  setPostProcessorOptions()
  
  #init globals
  ctr = prog.Executor.Controller
  motiontarget = prog.Executor.Controller.createTarget()
  write_statement = getAllStatementWriters()
  fold_templates = getFoldTemplates()
  data_lines=[]
  pos_lines=[]
  command_lines=[]
  data_names = []
  data_intend = ''
  cmd_intend = ''
  base_dict = {}
  tool_dict = {}
  ex_tcp_pairs = []
  pos_names = []
  
  #init motion globals
  c_tool = -1
  c_base = -1
  c_velaxis = -1
  c_vel_cp = -1
  c_vel_cp = -1
  c_apodist = -1
  c_bwdstart = True
  c_ipoframe = ''
  
  #create base and tool maps
  createFrameMaps(prog)
  
  #headers
  writeHeaders(main_name)
  
  #main routine
  writeRoutine(prog.MainRoutine, main_name)

  #subroutines
  for routine in prog.Routines:
    command_lines.append('')
    writeRoutine(routine, routine.Name)
    
  #footers
  writeFooters(main_name)
  
  #write tools and frames
  writeFrames(main_name)
  
  #write actual files
  filenamelist = writeFiles(uri)
  
  return True,filenamelist
#-------------------------------------------------------------------------------
def getProperties():
  #Properties for action panel
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  props.append((VC_BOOLEAN, 'Use spline motions (SLIN/SPTP)', True, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Use inline form (Folds)', True, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Comment out Base/Tool', False, None, None, 0, 0))
  return props
#-------------------------------------------------------------------------------
def writeFiles(uri):
  #Write .src and .dat files
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  files = []
  
  #Data into .dat file
  data_uri = uri[0:-4] + '.dat'
  files.append(data_uri)
  with open(data_uri,'wb') as output_file:
    for line in data_lines:
      output_file.write(line + '\n')
    for line in pos_lines:
      output_file.write(line + '\n')
  
  #Commands into .src file
  files.append(uri)
  with open(uri,'wb') as output_file:
    for line in command_lines:
      output_file.write(line + '\n')
      
  return files
#-------------------------------------------------------------------------------
def writeRoutine(routine, name):
  #Write one routine
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'DEF %s()' % name)
  
  cmd_intend = cmd_intend + '  '
  writeInit()
  command_lines.append(cmd_intend + ';COMMANDS')
  for statement in routine.Statements:

    write_statement[statement.Type](statement)
  cmd_intend = cmd_intend[0:-2]
  
  command_lines.append(cmd_intend + 'END')
  
  return
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Functions prone to customization
#-------------------------------------------------------------------------------
def setPostProcessorOptions():
  #Set post processor options
  global app, cmd, prog
  global use_spline_motions, use_inline_form, comment_out_frames
  
  #SLIN/SPTP instead of LIN/PTP
  use_spline_motions = cmd.getProperty('Use spline motions (SLIN/SPTP)').Value
  
  #Encapsulate commands into folds
  use_inline_form = cmd.getProperty('Use inline form (Folds)').Value
  
  #Should base and tool data assignments be commented out in .src header
  comment_out_frames = cmd.getProperty('Comment out Base/Tool').Value
  
  #DEBUG
  #use_spline_motions = False
  #use_inline_form = False
  #comment_out_frames = True
  #ENDDEBUG
  pass
#-------------------------------------------------------------------------------
def createFrameMaps(program):
  #Create frame maps as dictionarys.
  #Default frame names are mapped to their corresponding indices (e.g. TOOL_DATA[4] => 4).
  #Other frame names are mapped to unused indices if such exist.
  global base_dict, tool_dict, ex_tcp_pairs
  
  stats = getStatementsInProgram(program)
  
  #Get default frames first, exclude external tcp pairs
  for stat in stats:
    if stat.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      #Exclude external tcp
      if stat.Base and stat.Tool:
        if usesExternalTCP(stat.Base.Name, stat.Tool.Name):
          ex_tcp_pairs.append([stat.Base.Name, stat.Tool.Name])
          continue
      #Base
      base_name = 'Null'
      baseno = 0
      if stat.Base:
        base_name = stat.Base.Name
      baseno = getDefaultBaseIndex(base_name)
      if baseno >= 0 and not base_name in base_dict:
        base_dict[base_name] = baseno
      #Tool
      tool_name = 'Null'
      toolno = 0
      if stat.Tool:
        tool_name = stat.Tool.Name
      toolno = getDefaultToolIndex(tool_name)
      if toolno >= 0 and not tool_name in tool_dict:
        tool_dict[tool_name] = toolno
  #endfor stat in stats:
  
  #Get other frames, still exclude external tcp pairs
  for stat in stats:
    if stat.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION, VC_STATEMENT_PATH]:
      #Exclude external tcp
      if stat.Base and stat.Tool:
        if usesExternalTCP(stat.Base.Name, stat.Tool.Name):
          continue
      #Base
      base_name = 'Null'
      baseno = 0
      if stat.Base:
        base_name = stat.Base.Name
      baseno = getDefaultBaseIndex(base_name)
      if not base_name in base_dict:
        for i in range(1,33):
          if not i in base_dict.values():
            base_dict[base_name] = i
            print 'INFO: Mapping %s to BASE_DATA[%i].' % (base_name, i)
            break
        else: #no break, no free slot
          print 'WARNING: Cannot map ' + base_name + ' to any default base frame index.'
      #Tool
      tool_name = 'Null'
      toolno = 0
      if stat.Tool:
        tool_name = stat.Tool.Name
      toolno = getDefaultToolIndex(tool_name)
      if not tool_name in tool_dict:
        for i in range(1,17):
          if not i in tool_dict.values():
            tool_dict[tool_name] = i
            print 'INFO: Mapping %s to TOOL_DATA[%i].' % (tool_name, i)
            break
        else: #no break, no free slot
          print 'WARNING: Cannot map ' + tool_name + ' to any default tool frame index.'
  #endfor stat in stats:
  
  #Map external tcp pairs so that bases are mapped into tools and vice versa
  for base_name, tool_name in ex_tcp_pairs:
    baseno = getDefaultBaseIndex(base_name)
    toolno = getDefaultToolIndex(tool_name)
    if baseno == toolno and baseno > 0 and not tool_name in base_dict and not base_name in tool_dict:
      #Same index on both, use that
      base_dict[tool_name] = toolno
      tool_dict[base_name] = baseno
      print 'INFO: Mapping %s (extTCP) to BASE_DATA[%i].' % (tool_name, toolno)
      print 'INFO: Mapping %s (extTCP) to TOOL_DATA[%i].' % (base_name, baseno)
      continue
    #Base
    if not tool_name in base_dict:
      for i in range(1,33):
        if not i in base_dict.values():
          base_dict[tool_name] = i
          print 'INFO: Mapping %s (extTCP) to BASE_DATA[%i].' % (tool_name, i)
          break
      else: #no break, no free slot
        print 'WARNING: Cannot map ' + tool_name + ' (extTCP) to any default base frame index.'
    #Tool
    if not base_name in tool_dict:
      for i in range(1,17):
        if not i in tool_dict.values():
          tool_dict[base_name] = i
          print 'INFO: Mapping %s (extTCP) to TOOL_DATA[%i].' % (base_name, i)
          break
      else: #no break, no free slot
        print 'WARNING: Cannot map ' + base_name + ' (extTCP) to any default tool frame index.'
  #endfor base_name, tool_name in ex_tcp_pairs:
  pass
#-------------------------------------------------------------------------------
def getFoldTemplates():
  #Fold templates for different commands when using inline form _VAR_ are variables that are replaced in the final fold line
  fold_templates = {
    'PTP':';FOLD PTP _POS_ _CON_ Vel=_VEL_ % _PDAT_ Tool[_TOOLNO_] Base[_BASENO_] _ET_;%{PE}%R 8.3.48,%MKUKATPBASIS,%CMOVE,%VPTP,%P 1:PTP, 2:_POS_, 3:_APO_, 5:_VEL_, 7:_PDAT_',
    'LIN':';FOLD LIN _POS_ _CON_ Vel=_VEL_ m/s _CPDAT_ Tool[_TOOLNO_] Base[_BASENO_] _ET_;%{PE}%R 8.3.48,%MKUKATPBASIS,%CMOVE,%VLIN,%P 1:LIN, 2:_POS_, 3:_APO_, 5:_VEL_, 7:_CPDAT_',
    'SPTP':';FOLD SPTP _POS_ _CON_ Vel=_VEL_ % _PDAT_ Tool[_TOOLNO_] Base[_BASENO_] _ET_;%{PE}%R 8.3.48,%MKUKATPBASIS,%CSPLINE,%VSPTP_SB,%P 1:SPTP_SB, 2:_POS_, 3:_APO_, 5:_VEL_, 7:_PDAT_',
    'SLIN':';FOLD SLIN _POS_ _CON_ Vel=_VEL_ m/s _CPDAT_ Tool[_TOOLNO_] Base[_BASENO_] _ET_;%{PE}%R 8.3.48,%MKUKATPBASIS,%CSPLINE,%VSLIN_SB,%P 1:SLIN_SB, 2:_POS_, 3:_APO_, 5:_VEL_, 7:_CPDAT_',
    'COMMENT':';FOLD ;_COM_;%{PE}%R 8.3.48,%MKUKATPBASIS,%CCOMMENT,%VNORMAL,%P 2:_COM_',
    'DELAY':';FOLD WAIT Time=_TIM_ sec;%{PE}%R 8.3.48,%MKUKATPBASIS,%CWAIT,%VWAIT,%P 3:_TIM_',
    'SETBIN':';FOLD OUT _OUT_ \'\' State=_VAL_ ;%{PE}%R 8.3.48,%MKUKATPBASIS,%COUT,%VOUTX,%P 2:_OUT_, 3:, 5:_VAL_, 6:',
    'WAITBIN':';FOLD WAIT FOR (_NOT_ IN _IN_ \'\');%{PE}%R 8.3.48,%MKUKATPBASIS,%CEXT_WAIT_FOR,%VEXT_WAIT_FOR,%P 2:, 4:_NOT_, 5:$IN, 6:_IN_, 7:, 9:' #FIXTHIS
  }
  return fold_templates
#-------------------------------------------------------------------------------
def writeHeaders(main_name):
  #Header lines for .dat and .src files.
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  #.dat header in data_lines
  data_lines.append(data_intend + 'DEFDAT %s' % main_name)
  data_intend = data_intend + '  '
  
  #.src header
  #DEF name() is written in writeRoutine()
  pass
#-------------------------------------------------------------------------------
def writeFooters(main_name):
  #Footer lines for .dat and .src files.
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  #.dat footer in pos_lines
  data_intend = data_intend[0:-2]
  pos_lines.append(data_intend + 'ENDDAT')
  
  #.src footer
  #END (routine) is written in writeRoutine()
  pass
#-------------------------------------------------------------------------------
def writeFrames(main_name):
  #Base and tool data
  #Base data is converted into robot coordinates.
  #Tool data is valid only if attached to mountplate.
  #Assign data in the beginning of main routine
  global use_spline_motions, use_inline_form, comment_out_frames
  global ctr, motiontarget
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  global base_dict, tool_dict, ex_tcp_pairs
  
  comp = ctr.Component
  base_data = {}
  tool_data = {}
  comment = ''
  if comment_out_frames:
    comment = ';'
  
  #Bases
  for base_name, baseno in base_dict.iteritems():
    if baseno == 0:
      continue      #Robot world, no need to write

    #Check if external tool
    ex_tcp = False
    for pair in ex_tcp_pairs:
      if base_name == pair[1]:
        ex_tcp = True
        break
    #
    m = vcMatrix.new()
    if ex_tcp:
      for t in ctr.Tools:
        if t.Name == base_name:
          m = t.PositionMatrix
          if t.Node:
            m = comp.InverseWorldPositionMatrix * t.Node.WorldPositionMatrix * m
          break
    else:
      for b in ctr.Bases:
        if b.Name == base_name:
          m = b.PositionMatrix
          if b.Node:
            m = comp.InverseWorldPositionMatrix * b.Node.WorldPositionMatrix * m
          break
    if m:
      base_data[baseno] = '  %sBASE_DATA[%i]={%s}' %(comment, baseno, matrixToString(m))
  #endfor
  
  #Tools
  for tool_name, toolno in tool_dict.iteritems():
    if toolno == 0:
      continue      #default tool, no need to write

    #Check if external tool
    ex_tcp = False
    for pair in ex_tcp_pairs:
      if tool_name == pair[0]:
        ex_tcp = True
        break
    #
    m = vcMatrix.new()
    if ex_tcp:
      for b in ctr.Bases:
        if b.Name == tool_name:
          m = b.PositionMatrix
          break
    else:
      for t in ctr.Tools:
        if t.Name == tool_name:
          m = t.PositionMatrix
          break
    if m:
      tool_data[toolno] = '  %sTOOL_DATA[%i]={%s}' %(comment, toolno, matrixToString(m))
  #endfor
  
  #Write lines
  index = command_lines.index('DEF %s()' % main_name) + 1
  for baseno, line in base_data.iteritems():
    command_lines.insert(index, line)
    index += 1
  for toolno, line in tool_data.iteritems():
    command_lines.insert(index, line)
    index += 1
  command_lines.insert(index, '  ')
  
  pass
#-------------------------------------------------------------------------------
def writeInit():
  #Write routine init.
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if use_inline_form:
    command_lines.append(cmd_intend + ';FOLD INI;%{PE}')
    cmd_intend = cmd_intend + '  '
    command_lines.append(cmd_intend + ';FOLD BASISTECH INI')
    cmd_intend = cmd_intend + '  '
  command_lines.append(cmd_intend + 'GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM()')
  command_lines.append(cmd_intend + 'INTERRUPT ON 3')
  command_lines.append(cmd_intend + 'BAS (#INITMOV,0 )')
  if use_inline_form:
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD (BASISTECH INI)')
    command_lines.append(cmd_intend + ';FOLD USER INI')
    command_lines.append(cmd_intend + ';Make your modifications here')
    command_lines.append(cmd_intend + '')
    command_lines.append(cmd_intend + ';ENDFOLD (USER INI)')
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD (INI)')
  command_lines.append(cmd_intend + '')
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
#Statement writers
#-------------------------------------------------------------------------------
def getAllStatementWriters():
  statementhandlers = {
    VC_STATEMENT_BREAK:writeBreak,
    VC_STATEMENT_CALL:writeCall,
    VC_STATEMENT_COMMENT:writeComment,
    VC_STATEMENT_DEFINE_BASE:writeDefineBase,
    VC_STATEMENT_DEFINE_TOOL:writeDefineTool,
    VC_STATEMENT_DELAY:writeDelay,
    VC_STATEMENT_HALT:writeHalt,
    VC_STATEMENT_IF:writeIf,
    VC_STATEMENT_LINMOTION:writeLinMotion,
    VC_STATEMENT_PRINT:unhandled,
    VC_STATEMENT_PTPMOTION:writePtpMotion,
    VC_STATEMENT_RETURN:writeReturn,
    VC_STATEMENT_SETBIN:writeSetBin,
    VC_STATEMENT_WAITBIN:writeWaitBin,
    VC_STATEMENT_SETPROPERTY:writeSetProperty,
    VC_STATEMENT_WHILE:writeWhile,
    VC_STATEMENT_CONTINUE:unhandled,
    VC_STATEMENT_CUSTOM:unhandled,
    VC_STATEMENT_GRASP:unhandled,
    VC_STATEMENT_HOME:unhandled,
    VC_STATEMENT_PROG_SYNC:unhandled,
    VC_STATEMENT_RELEASE:unhandled,
    VC_STATEMENT_REMOTECALL:unhandled,
    VC_STATEMENT_REMOTEWAIT:unhandled,
    VC_STATEMENT_PATH:writePath,
    VC_STATEMENT_PROCESS:unhandled
  }
  if app_version >= 4.4:
    statementhandlers[VC_STATEMENT_SETROBOTSTATISTICSSTATE] = unhandled
    statementhandlers[VC_STATEMENT_SWITCHCASE] = writeSwitchCase
  return statementhandlers
#-------------------------------------------------------------------------------
def unhandled(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  print 'WARNING: Unhandled statement : %s %s' % (statement.Name,statement.Type)
#-------------------------------------------------------------------------------
def writeBreak(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'EXIT')
#-------------------------------------------------------------------------------
def writeCall(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if not statement.Routine:
    return
  command_lines.append(cmd_intend + '%s()' % statement.Routine.Name)
#-------------------------------------------------------------------------------
def writeComment(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  commentText = statement.Comment

  if commentText.startswith('USR_CMD'):
    commentText = commentText.replace('USR_CMD', '')
    command_lines.append(cmd_intend + '%s' % commentText)
    return

  if use_inline_form:
    if 'COMMENT' in fold_templates:
      fold = fold_templates['COMMENT']
      fold = fold.replace('_COM_',commentText)
      fold = fold.replace('_COM_',commentText)
    else:
      fold = ';FOLD'
    command_lines.append(cmd_intend + fold)
    command_lines.append(cmd_intend + ';ENDFOLD')
  else:
    command_lines.append(cmd_intend + ';%s' % commentText)
#-------------------------------------------------------------------------------
def writeDefineBase(statement):
  #Define base
  #Might not work if base node is other than None.
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if not statement.Base:
    return
  m = statement.Position
  if statement.IsRelative:
    command_lines.append(cmd_intend + '%s=%s:{%s}' %(statement.Base.Name, statement.Base.Name, matrixToString(m)))
  else:
    if statement.Node:
      #Convert to robot coordinates
      comp = statement.ParentRoutine.Program.Executor.Component
      m = comp.InverseWorldPositionMatrix * statement.Node.WorldPositionMatrix * m
    command_lines.append(cmd_intend + '%s={%s}' %(statement.Base.Name, matrixToString(m)))
#-------------------------------------------------------------------------------
def writeDefineTool(statement):
  #Define tool
  #Doesn't work if tool node is other than mountplate.
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if not statement.Tool:
    return
  m = statement.Position
  if statement.IsRelative:
    command_lines.append(cmd_intend + '%s=%s:{%s}' %(statement.Tool.Name, statement.Tool.Name, matrixToString(m)))
  else:
    command_lines.append(cmd_intend + '%s={%s}' %(statement.Tool.Name, matrixToString(m)))
#-------------------------------------------------------------------------------
def writeDelay(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if use_inline_form:
    if 'DELAY' in fold_templates:
      fold = fold_templates['DELAY']
      fold = fold.replace('_TIM_',str(statement.Delay))
    else:
      fold = ';FOLD'
    command_lines.append(cmd_intend + fold)
    cmd_intend = cmd_intend + '  '
    
  command_lines.append(cmd_intend + 'WAIT SEC %s' % statement.Delay)
  
  if use_inline_form:
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD')
#-------------------------------------------------------------------------------
def writeHalt(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'HALT')
#-------------------------------------------------------------------------------
def writeIf(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'IF %s THEN' % checkExpression(statement.Condition))
  
  cmd_intend = cmd_intend + '  '
  for child in statement.ThenScope.Statements:
    write_statement[child.Type](child)
  cmd_intend = cmd_intend[0:-2]
  
  if app_version >= 4.4 and statement.ElseIfScopes:
    # Ugly conversion from elseifs to nested ifs
    print "WARNING: Converting elseif statements to nested if statements."
    for elseifscope in statement.ElseIfScopes:
      command_lines.append(cmd_intend + 'ELSE')
      cmd_intend = cmd_intend + '  '
      
      command_lines.append(cmd_intend + 'IF %s THEN' % checkExpression(elseifscope.Condition))
      cmd_intend = cmd_intend + '  '
      for child in elseifscope.Statements:
        write_statement[child.Type](child)
      cmd_intend = cmd_intend[0:-2]
      
    command_lines.append(cmd_intend + 'ELSE')
    cmd_intend = cmd_intend + '  '
      
    for child in statement.ElseScope.Statements:
      write_statement[child.Type](child)
    
    for i in range(len(statement.ElseIfScopes)):
      cmd_intend = cmd_intend[0:-2]
      command_lines.append(cmd_intend + 'ENDIF')
    cmd_intend = cmd_intend[0:-2]
      
  elif statement.ElseScope.Statements:
    # Just regular else
    command_lines.append(cmd_intend + 'ELSE')
    cmd_intend = cmd_intend + '  '
    
    for child in statement.ElseScope.Statements:
      write_statement[child.Type](child)
    cmd_intend = cmd_intend[0:-2]
  
  command_lines.append(cmd_intend + 'ENDIF')
#-------------------------------------------------------------------------------
def writeLinMotion(statement):
  global ctr, motiontarget
  
  statement.writeToTarget(motiontarget)
  pos_name = statement.Positions[0].Name
  
  writeMotion(pos_name)
#-------------------------------------------------------------------------------
def writePtpMotion(statement):
  global ctr, motiontarget
  
  statement.writeToTarget(motiontarget)
  pos_name = statement.Positions[0].Name
  
  writeMotion(pos_name)
#-------------------------------------------------------------------------------
def writePath(statement):
  global ctr, motiontarget
  
  if statement.Base:
    motiontarget.BaseName = statement.Base.Name
  else:
    motiontarget.BaseName = 'Null'
  if statement.Tool:
    motiontarget.ToolName = statement.Tool.Name
  else:
    motiontarget.ToolName = 'Null'
    
  prop_names = []
  for prop in statement.SchemaProperties:
    prop_names.append(prop.Name)
    
  for point_index in range(len(statement.Positions)):
    #Target
    motiontarget.Target = statement.getSchemaValue(point_index,'Position')
    motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR
    
    #Ex axes
    ex_joints = []
    for i in range(6):
      prop_name = 'E' + str(i)
      if prop_name in prop_names:
        ex_joints.append(statement.getSchemaValue(point_index,prop_name))
    joints = motiontarget.JointValues
    if len(ex_joints) <= len(joints):
      robo_joint_count = len(joints) - len(ex_joints)
      for i in range(len(ex_joints)):
        joints[robo_joint_count + i] = ex_joints[i]
    motiontarget.JointValues = joints
    
    #Speed
    if 'MaxSpeed' in prop_names:
      motiontarget.CartesianSpeed = statement.getSchemaValue(point_index,'MaxSpeed')
    
    #Approximation
    if 'AccuracyMethod' in prop_names and 'AccuracyValue' in prop_names:
      motiontarget.AccuracyMethod = statement.getSchemaValue(point_index,'AccuracyMethod')
      motiontarget.AccuracyValue = statement.getSchemaValue(point_index,'AccuracyValue')
    
    #Acceleration
    if 'Acceleration' in prop_names:
      motiontarget.CartesianAcceleration = statement.getSchemaValue(point_index,'Acceleration')
    
    pos_name = statement.Name + '_P' + str(point_index+1)
    
    #Write motion
    writeMotion(pos_name)
#-------------------------------------------------------------------------------
def writeMotion(pos_name):
  #Write position from current motiontarget. Also write motion param commands into command lines.
  global use_spline_motions, use_inline_form, comment_out_frames
  global ctr, motiontarget
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  global base_dict, tool_dict, ex_tcp_pairs
  global c_tool, c_base, c_velaxis, c_vel_cp, c_apodist, c_bwdstart, c_ipoframe
  
  pos_name = getUniquePosName(pos_name)
  
  m = motiontarget.Target
  s = getStatus()
  t = getTurn()
  exs =['0','0','0','0','0','0']
  e6pos = 'X' + pos_name
  fdat = 'F' + pos_name
  pdat = 'P' + pos_name
  ldat = 'L' + pos_name
  
  base_name = motiontarget.BaseName
  if not base_name:
    base_name = 'Null'
  tool_name = motiontarget.ToolName
  if not tool_name:
    tool_name = 'Null'
  
  external_tcp = usesExternalTCP(base_name, tool_name)
  if external_tcp:
    baseno = base_dict.get(tool_name, -1)
    toolno = tool_dict.get(base_name, -1)
    ipoframe = '#TCP'
  else:
    baseno = base_dict.get(base_name, -1)
    toolno = tool_dict.get(tool_name, -1)
    ipoframe = '#BASE'
  
  #External axes
  ex_values = getExternalAxisValues()
  for i, ex_value in enumerate(ex_values):
    if i < len(exs):
      exs[i] = '%.3f' % ex_value
      
  
  #-----
  #Data
  #-----
  #
  #E6POS
  pos_lines.append(data_intend + 'DECL E6POS %s={%s,S %i,T %i,E1 %s,E2 %s,E3 %s,E4 %s,E5 %s,E6 %s}' \
    % (e6pos, matrixToString(m), s, t, exs[0], exs[1], exs[2], exs[3], exs[4], exs[5]))
  if use_inline_form:
    #FDAT
    pos_lines.append(data_intend + 'DECL FDAT %s={TOOL_NO %i,BASE_NO %i,IPO_FRAME %s,POINT2[] " ",TQ_STATE FALSE}' \
      % (fdat, toolno, baseno, ipoframe))
    if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
      #PDAT
      pos_lines.append(data_intend + 'DECL PDAT %s={VEL %.3f,ACC 100.000,APO_DIST %.3f,GEAR_JERK 100.000,EXAX_IGN 0}' \
        % (pdat, motiontarget.JointSpeedFactor * 100.0, motiontarget.AccuracyValue))
    else:#if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
      #LDAT
      try:
        acc = motiontarget.CartesianAcceleration / ctr.MaxCartesianAccel * 100.0
      except:
        acc = 100.0
      pos_lines.append(data_intend + 'DECL LDAT %s={VEL %.5f,ACC %.3f,APO_DIST %.3f,APO_FAC 50.0000,AXIS_VEL 100.000,AXIS_ACC %.3f,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,GEAR_JERK 50.0000,EXAX_IGN 0}' \
        % (ldat, motiontarget.CartesianSpeed / 1000.0, acc, motiontarget.AccuracyValue, acc))

        
  #-----
  #Commands
  #-----
  if use_inline_form:
    #-----
    #Open fold
    #-----
    if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
      if use_spline_motions:
        #SPTP
        if 'SPTP' in fold_templates:
          fold = fold_templates['SPTP']
        else:
          fold = ';FOLD'
      else:
        #PTP
        if 'PTP' in fold_templates:
          fold = fold_templates['PTP']
        else:
          fold = ';FOLD'
      fold = fold.replace('_VEL_',str(motiontarget.JointSpeedFactor * 100.0))
      fold = fold.replace('_PDAT_',str(pdat))
    else: #if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
      if use_spline_motions:
        #SLIN
        if 'SLIN' in fold_templates:
          fold = fold_templates['SLIN']
        else:
          fold = ';FOLD'
      else:
        #LIN
        if 'LIN' in fold_templates:
          fold = fold_templates['LIN']
        else:
          fold = ';FOLD'
      fold = fold.replace('_VEL_',str(motiontarget.CartesianSpeed / 1000.0))
      fold = fold.replace('_CPDAT_',str(ldat))
    
    cont = ''
    apo = ''
    if motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and motiontarget.AccuracyValue > 0:
      cont = 'CONT'
      if use_spline_motions: # and motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        apo = 'C_SPL'
      else:
        apo = 'C_DIS'
    et = ''
    if external_tcp:
      et = 'extTCP'
      
    fold = fold.replace('_CON_',cont)
    fold = fold.replace('_APO_',apo)
    fold = fold.replace('_POS_',str(pos_name))
    fold = fold.replace('_TOOLNO_',str(toolno))
    fold = fold.replace('_BASENO_',str(baseno))
    fold = fold.replace('_ET_',et)
    
    command_lines.append(cmd_intend + fold)
    cmd_intend = cmd_intend + '  '
  #endif use_inline_form
    
  if use_spline_motions:
    #---
    #SLIN/SPTP, spline linear/ptp motion
    #---
    
    params = []
    
    if use_inline_form:
      #-----
      #Spline motion with folds and FDAT, PDAT and LDAT
      #-----
      #;FOLD SPTP P1 Vel=100.0...
      #  SPTP XP1 WITH $VEL_AX...
      #;ENDFOLD
      #-----
      #Axis speed (change value for axis 1 which is usually slowest)
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        params.append('$VEL_AXIS[1]=SVEL_JOINT(%.0f)' % (motiontarget.JointSpeedFactor * 100.0))
        
      #Velocity
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        params.append('$VEL=SVEL_CP(%.2f, ,%s)' % ((motiontarget.CartesianSpeed / 1000.0), ldat))
      
      #Tool
      params.append('$TOOL=STOOL2(%s)' % fdat)
      
      #Base
      params.append('$BASE=SBASE(%s.BASE_NO)' % fdat)
      
      #IPO
      params.append('$IPO_MODE=SIPO_MODE(%s.IPO_FRAME)' % fdat)
      
      #Load
      params.append('$LOAD=SLOAD(%s.TOOL_NO)' % fdat)
      
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        #Axis acceleration (change value for axis 1 which is usually slowest)
        params.append('$ACC_AXIS[1]=SACC_JOINT(%s)' % pdat)
        #Approximation
        params.append('$APO=SAPO_PTP(%s)' % pdat)
        #Gear jerk (change value for axis 1 which is usually slowest)
        params.append('$GEAR_JERK[1]=SGEAR_JERK(%s)' % pdat)
      
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        #Acceleration
        params.append('$ACC=SACC_CP(%s)' % ldat)
        #Approximation
        params.append('$APO=SAPO(%s)' % ldat)
        #Jerk
        params.append('$JERK=SJERK(%s)' % ldat)
      
      param_string = ''
      if params:
        param_string = ' WITH ' + params[0]
        for i in range(1, len(params)):
          param_string += ', ' + params[i]
      
      #Approximation tag at the end
      if motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and motiontarget.AccuracyValue > 0:
        param_string += ' C_SPL'
      
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        command_lines.append(cmd_intend + 'SPTP %s%s' % (e6pos, param_string))
      elif motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        command_lines.append(cmd_intend + 'SLIN %s%s' % (e6pos, param_string))
    else:
      #-----
      #Spline motion without folds and FDAT, PDAT and LDAT
      #-----
      #SPTP XP1 WITH $VEL_AX...
      #-----
      #Axis speed (change value for axis 1 which is usually slowest)
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        params.append('$VEL_AXIS[1]=%.1f' % (motiontarget.JointSpeedFactor * 100.0))
        
      #Velocity
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        params.append('$VEL.CP=%.2f' % (motiontarget.CartesianSpeed / 1000.0))
        
      #Tool
      if toolno == 0:
        params.append('$TOOL=$NULLFRAME')
      else:
        params.append('$TOOL=TOOL_DATA[%i]' % toolno)
      
      #Base
      if baseno == 0:
        params.append('$BASE=$WORLD')
      else:
        params.append('$BASE=BASE_DATA[%i]' % baseno)
      
      #IPO
      params.append('$IPO_MODE=%s' % ipoframe)
      
      #Load
      if toolno == 0:
        params.append('$LOAD.M=$DEF_L_M')
        params.append('$LOAD.CM=$DEF_L_CM')
        params.append('$LOAD.J=$DEF_L_J')
      else:
        params.append('$LOAD=LOAD_DATA[%i]' % toolno)
      
      #Approximation
      if motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and motiontarget.AccuracyValue > 0:
        params.append('$APO.CDIS=%.1f C_SPL' % motiontarget.AccuracyValue)
      
      #Command line
      
      param_string = ''
      if params:
        param_string = ' WITH ' + params[0]
        for i in range(1, len(params)):
          param_string += ', ' + params[i]
      
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        command_lines.append(cmd_intend + 'SPTP %s%s' % (e6pos, param_string))
      elif motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        command_lines.append(cmd_intend + 'SLIN %s%s' % (e6pos, param_string))
    #endif use_spline_motions:
    
  else:
    #---
    #LIN/PTP, old (non-spline) linear/ptp motion
    #---
    
    if use_inline_form:
      #-----
      #Non-spline motion with folds and FDAT, PDAT and LDAT
      #-----
      #;FOLD PTP P1 Vel=100...
      #  $BWDSTART=FALSE
      #  PDAT_ACT=PPDAT4
      #  FDAT_ACT=FP1
      #  BAS(#PTP_PARAMS,100)
      #  PTP XP1 
      #;ENDFOLD
      #-----
      
      #BWDSTART
      if c_bwdstart:
        command_lines.append(cmd_intend + '$BWDSTART=FALSE')
        c_bwdstart = False
        
      #FDAT
      command_lines.append(cmd_intend + 'FDAT_ACT=%s' % fdat)
      
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        #PDAT
        command_lines.append(cmd_intend + 'PDAT_ACT=%s' % pdat)
        command_lines.append(cmd_intend + 'BAS(#PTP_PARAMS,%.0f)' % (motiontarget.JointSpeedFactor * 100.0))
      elif motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        #LDAT
        command_lines.append(cmd_intend + 'LDAT_ACT=%s' % ldat)
        command_lines.append(cmd_intend + 'BAS(#CP_PARAMS,%.2f)' % (motiontarget.CartesianSpeed / 1000.0))
      
      #Approximation
      approx = ''
      if motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and motiontarget.AccuracyValue > 0:
        approx = ' C_DIS'
      
      #Command line
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        command_lines.append(cmd_intend + 'PTP %s%s' % (e6pos, approx))
      elif motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        command_lines.append(cmd_intend + 'LIN %s%s' % (e6pos, approx))
      #-----
    else:
      #-----
      #Non-spline motion without folds and FDAT, PDAT and LDAT
      #-----
      #$BWDSTART=FALSE
      #$TOOL=TOOL_DATA[1]
      #$BASE=BASE_DATA[1]
      #$VEL_AXIS[1]=100.0
      #$APO.CDIS=20.0
      #PTP XP1 C_DIS
      #-----
      
      #BWDSTART
      if c_bwdstart:
        command_lines.append(cmd_intend + '$BWDSTART=FALSE')
        c_bwdstart = False
      
      #Tool
      if toolno != c_tool:
        if toolno == 0:
          command_lines.append(cmd_intend + '$TOOL=$NULLFRAME')
        else:
          command_lines.append(cmd_intend + '$TOOL=TOOL_DATA[%i]' % toolno)
        c_tool = toolno

        #Load
        if toolno == 0:
          command_lines.append(cmd_intend + '$LOAD.M=$DEF_L_M')
          command_lines.append(cmd_intend + '$LOAD.CM=$DEF_L_CM')
          command_lines.append(cmd_intend + '$LOAD.J=$DEF_L_J')
        else:
          command_lines.append(cmd_intend + '$LOAD=LOAD_DATA[%i]' % toolno)
      
      #Base
      if baseno != c_base:
        if toolno == 0:
          command_lines.append(cmd_intend + '$BASE=$WORLD')
        else:
          command_lines.append(cmd_intend + '$BASE=BASE_DATA[%i]' % baseno)
        c_base = baseno
        
      #IPO
      if ipoframe != c_ipoframe:
        command_lines.append(cmd_intend + '$IPO_MODE=%s' % ipoframe)
        c_ipoframe = ipoframe
      
      #Axis speed (change value for axis 1 which is usually slowest)
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT and motiontarget.JointSpeedFactor != c_velaxis:
        command_lines.append(cmd_intend + '$VEL_AXIS[1]=%.1f' % (motiontarget.JointSpeedFactor * 100.0))
        c_velaxis = motiontarget.JointSpeedFactor
      
      #Velocity
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR and motiontarget.CartesianSpeed != c_vel_cp:
        command_lines.append(cmd_intend + '$VEL.CP=%.2f' % (motiontarget.CartesianSpeed / 1000.0))
        c_vel_cp = motiontarget.CartesianSpeed
        
      #Approximation
      approx = ''
      if motiontarget.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE and motiontarget.AccuracyValue > 0:
        approx = ' C_DIS'
        if motiontarget.AccuracyValue != c_apodist:
          command_lines.append(cmd_intend + '$APO.CDIS=%.1f' % motiontarget.AccuracyValue)
          c_apodist = motiontarget.AccuracyValue
      
      #Command line
      if motiontarget.MotionType == VC_MOTIONTARGET_MT_JOINT:
        command_lines.append(cmd_intend + 'PTP %s%s' % (e6pos, approx))
      elif motiontarget.MotionType == VC_MOTIONTARGET_MT_LINEAR:
        command_lines.append(cmd_intend + 'LIN %s%s' % (e6pos, approx))
      #-----
    #endif use_inline_form
  #endif use_spline_motions:
  
  
  if use_inline_form:
    #-----
    #Close fold
    #-----
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD')
#-------------------------------------------------------------------------------
def writeReturn(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'RETURN')
#-------------------------------------------------------------------------------
def writeSetBin(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  if use_inline_form:
    if 'SETBIN' in fold_templates:
      fold = fold_templates['SETBIN']
      fold = fold.replace('_OUT_',str(statement.OutputPort))
      fold = fold.replace('_VAL_',str(statement.OutputValue).upper())
    else:
      fold = ';FOLD'
    command_lines.append(cmd_intend + fold)
    cmd_intend = cmd_intend + '  '
  
  command_lines.append(cmd_intend + '$OUT[%i]=%s' % (statement.OutputPort, str(statement.OutputValue).upper()))
  
  if use_inline_form:
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD')
#-------------------------------------------------------------------------------
def writeWaitBin(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  inverse = ''
  if not statement.InputValue:
    inverse = 'NOT'
  
  if use_inline_form:
    if 'WAITBIN' in fold_templates:
      fold = fold_templates['WAITBIN']
      fold = fold.replace('_IN_',str(statement.InputPort))
      fold = fold.replace('_NOT_',inverse)
    else:
      fold = ';FOLD'
    command_lines.append(cmd_intend + fold)
    cmd_intend = cmd_intend + '  '
  
  command_lines.append(cmd_intend + 'WAIT FOR (%s $IN[%i] )' %(inverse, statement.InputPort))
  
  if use_inline_form:
    cmd_intend = cmd_intend[0:-2]
    command_lines.append(cmd_intend + ';ENDFOLD')
#-------------------------------------------------------------------------------
def writeSetProperty(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  #Declare data
  if not statement.TargetProperty in data_names:
    comp = statement.ParentRoutine.Program.Executor.Component
    prop = statement.ParentRoutine.getProperty(statement.TargetProperty)
    if not prop:
      prop = comp.getProperty(statement.TargetProperty)
      
    if prop:
      if prop.Type == VC_BOOLEAN:
        data_lines.append(data_intend + 'DECL BOOL %s' % statement.TargetProperty)
        data_names.append(statement.TargetProperty)
      elif prop.Type == VC_INTEGER:
        data_lines.append(data_intend + 'DECL INT %s' % statement.TargetProperty)
        data_names.append(statement.TargetProperty)
      else:
        print 'WARNING: Unsupported data type %s in assign statement : %s=%s' % (statement.TargetProperty, statement.TargetProperty, statement.ValueExpression)
    else:
      print 'WARNING: Unknown variable %s in assign statement : %s=%s' % (statement.TargetProperty, statement.TargetProperty, statement.ValueExpression)
      
    
  #Add command line
  command_lines.append(cmd_intend + statement.TargetProperty + '=' + statement.ValueExpression)
#-------------------------------------------------------------------------------
def writeWhile(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'WHILE %s' % checkExpression(statement.Condition))
  
  cmd_intend = cmd_intend + '  '
  for child in statement.Scope.Statements:
    write_statement[child.Type](child)
  cmd_intend = cmd_intend[0:-2]
  
  command_lines.append(cmd_intend + 'ENDWHILE')
#-------------------------------------------------------------------------------
def writeSwitchCase(statement):
  global use_spline_motions, use_inline_form, comment_out_frames
  global write_statement, fold_templates
  global data_lines, pos_lines, command_lines, data_names, data_intend, cmd_intend
  
  command_lines.append(cmd_intend + 'SWITCH %s' % checkExpression(statement.Condition))
  
  cmd_intend = cmd_intend + '  '
  for case in statement.Cases:
    if case.CaseCondition.replace(' ','').lower() == 'default':
      command_lines.append(cmd_intend + 'DEFAULT')
    else:
      command_lines.append(cmd_intend + 'CASE %s' % checkExpression(case.CaseCondition))
    cmd_intend = cmd_intend + '  '
    for child in case.Statements:
      write_statement[child.Type](child)
    cmd_intend = cmd_intend[0:-2]
  cmd_intend = cmd_intend[0:-2]
  
  command_lines.append(cmd_intend + 'ENDSWITCH')
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Helper functions
#-------------------------------------------------------------------------------
def getStatus():
  #Get status from current motiontarget
  global ctr, motiontarget
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
#-------------------------------------------------------------------------------
def getTurn():
  #Get turn from current motiontarget
  global ctr, motiontarget
  
  turn = 0
  joints = motiontarget.JointValues
  if len(joints) >= 6:
    if joints[0] < 0:
      turn += 1
    if joints[1] < 0:
      turn += 2
    if joints[2] < 0:
      turn += 4
    if joints[3] < 0:
      turn += 8
    if joints[4] < 0:
      turn += 16
    if joints[5] < 0:
      turn += 32
  return turn
#-------------------------------------------------------------------------------
def matrixToString(m):
  s = 'X %.3f,Y %.3f,Z %.3f,A %.4f,B %.4f,C %.4f' % (m.P.X, m.P.Y, m.P.Z, m.WPR.Z, m.WPR.Y, m.WPR.X)
  return s
#-------------------------------------------------------------------------------
def getDefaultBaseIndex(base_name):
  if base_name.lower() == 'null':
    return 0
  if re.sub('1|2|3|4|5|6|7|8|9|0', '', base_name) == 'BASE_DATA[]':
    baseno = int(base_name[10:-1])
    if baseno > 0 and baseno <= 32:
      return baseno
  return -1
#-------------------------------------------------------------------------------
def getDefaultToolIndex(tool_name):
  if tool_name.lower() == 'null':
    return 0
  if re.sub('1|2|3|4|5|6|7|8|9|0', '', tool_name) == 'TOOL_DATA[]':
    toolno = int(tool_name[10:-1])
    if toolno > 0 and toolno <= 16:
      return toolno
  return -1
#-------------------------------------------------------------------------------
def getExternalAxisValues():
  #Get ex axes values from current motiontarget
  global ctr, motiontarget
  
  exs = []
  for i,joint in enumerate(ctr.Joints):
    if joint.ExternalController and i < len(motiontarget.JointValues):
      exs.append(motiontarget.JointValues[i])
  return exs
#-------------------------------------------------------------------------------
def getStatementsInProgram(program):
  #Get all statements in a program including statements in while/if scopes
  
  rous = []
  rous.append(program.MainRoutine)
  rous.extend(program.Routines)
  stats = []
  for rou in rous:
    stats.extend(getStatementsInScope(rou))
    
  return stats
#-------------------------------------------------------------------------------
def getStatementsInScope(scope):
  #Get all statements in a scope including statements in while/if scopes
  
  stats = []
  for stat in scope.Statements:
    stats.append(stat)
    if stat.Type == VC_STATEMENT_WHILE:
      stats.extend(getStatementsInScope(stat.Scope))
    elif stat.Type == VC_STATEMENT_IF:
      stats.extend(getStatementsInScope(stat.ThenScope))
      stats.extend(getStatementsInScope(stat.ElseScope))
  
  return stats
#-------------------------------------------------------------------------------
def usesExternalTCP(base_name, tool_name):
  #Check if current base/tool pair is external tcp combination.
  global ctr, motiontarget
  
  #Check that tool is not attached to a node in robot node structure (including child components) or
  #base is attached to a node in rsobot node structure.
  
  base_in_robot = False
  for base in ctr.Bases:
    if base.Name == base_name:
      parent = base.Node
      while parent:
        if parent == ctr.FlangeNode:
          base_in_robot = True
          break
        parent = parent.Parent
      break
      
  tool_in_robot = False
  for tool in ctr.Tools:
    if tool.Name == tool_name:
      if tool.IPOMode != 1:
        #IPOMode #BASE not set. Could be a normal tool used with tool changer. Skip
        tool_in_robot = True
        break
      parent = tool.Node
      while parent:
        if parent == ctr.FlangeNode:
          tool_in_robot = True
          break
        parent = parent.Parent
      break
  else:
    #no break, propably Null
    tool_in_robot = True
  
  ex_tcp = False  
  if base_in_robot or not tool_in_robot:
    ex_tcp = True
  
  return ex_tcp
#-------------------------------------------------------------------------------
def getUniquePosName(pos_name):
  global pos_names
  
  if pos_name in pos_names:
    index = pos_name.rfind('_')
    if index >= 0:
      pos_name = pos_name[:index]
    for i in range(1, 99999):
      test_name = pos_name + '_' + str(i)
      if not test_name in pos_names:
        pos_name = test_name
        break
  pos_names.append(pos_name)
  return pos_name
#-------------------------------------------------------------------------------
def checkExpression(exp):
  
  # Boolean literals
  exp = exp.replace('False', 'FALSE')
  exp = exp.replace('True', 'TRUE')
  
  # Signals
  exp = exp.replace('IN', '$IN')
  exp = exp.replace('OUT', '$OUT')
  
  # Operators
  exp = exp.replace('!=', '<>')
  exp = exp.replace('&&', ' AND ')
  exp = exp.replace('&', ' AND ')
  exp = exp.replace('||', ' OR ')
  exp = exp.replace('|', ' OR ')
  
  exp = exp.replace('  ', ' ')
  
  return exp
