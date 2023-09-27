# Version 0.1 (02.01.2020)

from vcCommand import *
import vcMatrix, os.path, math


def writeWaitBin(output_file, statement):
  global linenum
  output_file.write("%i Wait M_In(%i)=%i\n" %(linenum,statement.InputPort,1 if statement.InputValue else 0))
  linenum+=1

def writeSetBin(output_file, statement):
  global linenum
  output_file.write("%i M_Out(%i)=%i\n" %(linenum,statement.OutputPort,1 if statement.OutputValue else 0) )
  linenum+=1
  
def writeDelay(output_file, statement):
  global linenum
  output_file.write("%i Dly %3.2f\n" % (linenum,statement.Delay))
  linenum+=1

def writeComment(output_file, statement):
  global linenum
  output_file.write("%i ' %s\n" % (linenum,statement.Comment))
  linenum+=1

def writeCall(output_file, statement):
  global linenum
  if statement.getProperty("Routine").Value:
    routine = statement.getProperty("Routine").Value.Name
    output_file.write("%i CallP \"%s\"\n" % (linenum, routine))
    linenum+=1

def writeLinMotion(output_file, statement):
  global currentspeed, linenum
  statement.writeToTarget(motiontarget)
  writeToolDefinitionIfChanged(output_file, motiontarget)
  writeBaseDefinitionIfChanged(output_file, motiontarget)
  if currentspeed != statement.MaxSpeed:
    # if statement speed changes, output new speed value statement to the robot program
    currentspeed=statement.MaxSpeed
    output_file.write( "%i Spd %3.2f\n" % (linenum, currentspeed) )
    linenum += 1
  output_file.write("%i Mvs %s\n" % (linenum,statement.Positions[0].Name))
  linenum+=1
  # store position data to write it later to the end of file
  name = statement.Positions[0].Name
  p = motiontarget.Target.P
  ori = motiontarget.Target.getWPR()
  external_j_values = statement.Positions[0].ExternalJointValues
  config = GetConfigs(motiontarget)
  positions.append( [name, p, ori, external_j_values,config] )

def writePtpMotion(output_file, statement):
  global currentjointspeed, linenum
  statement.writeToTarget(motiontarget)
  writeToolDefinitionIfChanged(output_file, motiontarget)
  writeBaseDefinitionIfChanged(output_file, motiontarget)
  if currentjointspeed != statement.JointSpeed*100:
    # if joint statement speed changes, output new speed value statement to the robot program
    currentjointspeed=statement.JointSpeed*100
    output_file.write("%i JOvrd %3.2f\n" % (linenum,currentjointspeed))
    linenum+=1
  output_file.write("%i Mov %s\n" % (linenum,statement.Positions[0].Name))
  linenum+=1
  # store position data to write it later to the end of file
  name = statement.Positions[0].Name
  p = motiontarget.Target.P
  ori = motiontarget.Target.getWPR()
  external_j_values = statement.Positions[0].ExternalJointValues
  config = GetConfigs(motiontarget)
  positions.append( [name, p, ori, external_j_values,config] )

def writePath(output_file,statement):
  global currentspeed, linenum
  external_joints = [x for x in controller.Joints if x.ExternalController]
  external_joints = range(len(external_joints))
  motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR    
  if statement.Base == None:
    motiontarget.BaseName = ""
  else:
    motiontarget.BaseName = statement.Base.Name
  if statement.Tool == None:
    motiontarget.ToolName = ""
  else:
    motiontarget.ToolName = statement.Tool.Name
  writeToolDefinitionIfChanged(output_file, motiontarget)
  writeBaseDefinitionIfChanged(output_file, motiontarget)
  for i in xrange( statement.getSchemaSize()):
    target = statement.getSchemaValue(i,"Position")
    motiontarget.Target = target
    jv = motiontarget.JointValues
    motiontarget.JointValues = jv
    motiontarget.AccuracyMethod = statement.getSchemaValue(i,"AccuracyMethod")
    motiontarget.AccuracyValue = statement.getSchemaValue(i,"AccuracyValue")
    speed = statement.getSchemaValue(i,"MaxSpeed")
    if currentspeed != speed:
      # if statement speed changes, output new speed value statement to the robot program
      currentspeed = speed
      output_file.write( "%i Spd %3.2f\n" % (linenum, currentspeed) )
      linenum += 1
    name = '%s_%i' % (statement.Name, i)
    output_file.write("%i Mvs %s\n" % (linenum,name))
    linenum += 1
    # store position data to write it later to the end of file
    p = motiontarget.Target.P
    ori = motiontarget.Target.getWPR()
    external_j_values = [statement.getSchemaValue(i,'E%i' % (x+1)) for x in external_joints]
    config = GetConfigs(motiontarget) 
    positions.append( [name, p, ori, external_j_values,config] )

def writeTargetDefinition(output_file, position):
  name = position[0]
  p = position[1]
  ori = position[2]
  external_j_values = position[3]
  config = position[4]
  output_file.write("%s=(%8.3f, %8.3f, %8.3f, %3.3f, %3.3f, %3.3f" % (name,p.X,p.Y,p.Z,ori.Z,ori.Y,ori.X))
  for j in external_j_values:
    output_file.write(", ")
    output_file.write("%8.3f" % j)
  output_file.write(")(%s)\n" % config)

def writeToolDefinitionIfChanged(output_file,motiontarget):
  global currenttool,linenum
  if motiontarget.ToolName != currenttool:
    currenttool = motiontarget.ToolName
    toolvalue = 0
    for i, tool in enumerate(controller.Tools):
      if tool.Name == currenttool:
        toolvalue=i
        break
    if toolvalue==0:
      output_file.write("%i Tool P_NTool\n" % linenum)
    else:
      output_file.write("%i M_Tool=%i\n" % (linenum,toolvalue))
    linenum+=1

def writeBaseDefinitionIfChanged(output_file,motiontarget):
  global currentbase,linenum
  if motiontarget.BaseName != currentbase:
    currentbase=motiontarget.BaseName
    basevalue = 0
    for i, base in enumerate(controller.Bases):
      if base.Name == currentbase:
        basevalue=i
        break
    output_file.write("%i Base %i\n" % (linenum,basevalue))
    linenum+=1

def GetConfigs(motiontarget):
  joints = motiontarget.JointValues
  rconf = motiontarget.RobotConfig
  if rconf == 0:
    lf1 = 3
  elif rconf == 1:
    lf1 = 2
  elif rconf == 2:
    lf1 = 1
  elif rconf == 3:
    lf1 = 0
  elif rconf == 4:
    lf1 = 7
  elif rconf == 5:
    lf1 = 6
  elif rconf == 6:
    lf1 = 5
  elif rconf == 7:
    lf1 = 4
  numofjoints=len(motiontarget.JointValues)
  lf2=0
  c=0
  for j in joints:
    turns=0
    if j >= 180.0:
      turns=1
      turns+=int((j-180.0)/360.0)
    elif j <= -180.0:
      turns=-1
      turns-=int((j+180.0)/360.0)
    if turns<0:
      turns=16-turns
    lf2+=turns*16**c
    c+=1
  return "%i,%i" % (lf1,lf2)

def unknown(output_file, statement):
  print '> Unsupported statement type skipped:', statement.Type

def translateRoutine( routine, name, output_file):
  pointCount = 0
  statementCount = 0
  for statement in routine.Statements:
    translator = statement_translators.get(statement.Type, unknown)
    translator(output_file,statement)

def postProcess(app,program,uri):
  global motiontarget, controller, positions
  global currentspeed, currentjointspeed, currentbase, currenttool, linenum
  positions = []
  currentspeed=-1
  currentjointspeed=-1
  currentbase=None
  currenttool=None
  linenum=1
  controller = program.Executor.Controller
  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-7]
  motiontarget = program.Executor.Controller.createTarget()
  ret = []
  with open(uri,"w") as output_file:
    # main
    translateRoutine(program.MainRoutine, mainName, output_file)
    output_file.write("%i End\n" % linenum);
    for position in positions:
      writeTargetDefinition(output_file, position)
    ret.append(uri)
    
  folder, filename = os.path.split(uri)
  for routine in program.Routines:
    positions = []
    currentspeed=-1
    currentjointspeed=-1
    currentbase=None
    currenttool=None
    linenum=1
    uri = os.path.join(folder, routine.Name + '.prg')
    with open(uri,"w") as output_file:
    # subroutines
      translateRoutine(routine, routine.Name, output_file)
      output_file.write("%i End\n" % linenum);
      for position in positions:
        writeTargetDefinition(output_file, position)
      ret.append(uri)
  return True,ret

indentation = '  '
statement_translators = {
VC_STATEMENT_WAITBIN:writeWaitBin,
VC_STATEMENT_SETBIN:writeSetBin,
VC_STATEMENT_DELAY:writeDelay,
VC_STATEMENT_COMMENT:writeComment,
VC_STATEMENT_CALL:writeCall,
VC_STATEMENT_LINMOTION:writeLinMotion,
VC_STATEMENT_PTPMOTION:writePtpMotion,
VC_STATEMENT_PATH:writePath}
