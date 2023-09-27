#
# Kawasaki Robots AS program sample post-processor V0.1
# Created on: 13.08.2020
#

import vcHelpers.Robot2
from vcCommand import *
from vcHelpers.Selection import *
import vcMatrix, os.path


def WriteTransformationPoints(mod, statement):
  '''
  Convert statement position to X, Y, Z, O, A, T
  '''
  global motiontarget

  statement.writeToTarget(motiontarget)
  p = motiontarget.Target.P
  x,y,z = p.X, p.Y, p.Z

  ori = motiontarget.Target.getEuler()
  o, a, t = ori.Z, ori.Y, ori.X

  mod.write(statement.Positions[0].Name + ' ')
  mod.write("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f" % (x, y, z, o, a, t))
  mod.write("\n")

def WriteBasePoints(mod, statement):  
  '''
  Convert base position to X, Y, Z, O, A, T
  '''
  name = statement.Base.Name
  m = vcMatrix.new(statement.Position)
  p = m.P
  ori = m.getEuler()
  x,y,z = p.X, p.Y, p.Z
  o, a, t = ori.Z, ori.Y, ori.X
  mod.write(name + ' ')
  mod.write("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f" % (x, y, z, o, a, t))
  mod.write("\n")

def WriteToolPoints(mod, statement): 
  '''
  Convert tool position to X, Y, Z, O, A, T
  '''
  name = statement.Tool.Name
  m = vcMatrix.new(statement.Position)
  p = m.P
  ori = m.getEuler()
  x,y,z = p.X, p.Y, p.Z
  o, a, t = ori.Z, ori.Y, ori.X
  mod.write(name + ' ')
  mod.write("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f" % (x, y, z, o, a, t))
  mod.write("\n")

def WritePrecisionPoints(mod, statement):
  '''
  Convert to joint values
  '''
  global motiontarget

  statement.writeToTarget(motiontarget)
  mod.write("#")
  mod.write(statement.Positions[0].Name + ' ')
  js= motiontarget.JointValues
  pos = "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f" % (js[0], js[1], js[2], js[3], js[4], js[5])
  mod.write("%s" % pos)
  mod.write("\n")

def writeRobotConfig(mod, statement):
  '''
  Write robot configuration
  '''
  global motiontarget, currentconfig

  statement.writeToTarget(motiontarget)
  if currentconfig != motiontarget.RobotConfig:
    newconf = motiontarget.RobotConfig   
    if newconf == 0:
      mod.write("RIGHTY\n")
      mod.write("ABOVE\n")
      mod.write("DWRIST\n")
    elif newconf == 1:
      mod.write("RIGHTY\n")
      mod.write("ABOVE\n")
      mod.write("UWRIST\n")
    elif newconf == 2:
      mod.write("RIGHTY\n")
      mod.write("BELOW\n")
      mod.write("DWRIST\n")
    elif newconf == 3:
      mod.write("RIGHTY\n")
      mod.write("BELOW\n")
      mod.write("UWRIST\n")
    elif newconf == 4:
      mod.write("LEFTY\n")
      mod.write("ABOVE\n")
      mod.write("DWRIST\n")
    elif newconf == 5:
      mod.write("LEFTY\n")
      mod.write("ABOVE\n")
      mod.write("UWRIST\n")
    elif newconf == 6:
      mod.write("LEFTY\n")
      mod.write("BELOW\n")
      mod.write("DWRIST\n")
    elif newconf == 7:
      mod.write("LEFTY\n")
      mod.write("BELOW\n")
      mod.write("UWRIST\n")      
    currentconfig = motiontarget.RobotConfig

#-------------------------------------------------------------------------------

def WriteProgramInit(mod, first_stmt):  
  '''
  Write program initialization 
  '''
  global currentbase, currenttool, currentaccuracy, currentjointspeed, currentspeed

  mod.write(";Initialization" + "*"*66 + "\n")
  currentbase = first_stmt.Base
  currenttool = first_stmt.Tool
  currentaccuracy = first_stmt.AccuracyValue
  
  if currentbase is not None:
    currentbase = first_stmt.Base.Name
    mod.write("BASE %s\n" %currentbase)
  else:
    mod.write("BASE NULL\n")
  if currenttool is not None:
    currenttool = first_stmt.Tool.Name
    mod.write("TOOL %s\n" %currenttool)
  else:
    mod.write("TOOL NULL\n") 
  if first_stmt.Type == VC_STATEMENT_PTPMOTION:
    currentjointspeed = first_stmt.JointSpeed
    jspeed = currentjointspeed * 100
    mod.write("SPEED %.2f ALWAYS\n" %jspeed)
  if first_stmt.Type == VC_STATEMENT_LINMOTION:
    currentspeed = first_stmt.MaxSpeed
    lspeed = str(currentspeed)+' MM/S'
    mod.write("SPEED %s ALWAYS\n" %lspeed)
  if currentaccuracy > -1:
    mod.write("ACCURACY %.2f ALWAYS\n" %currentaccuracy)
    mod.write("CP ON\n")
  else:
    mod.write("CP OFF\n")
  mod.write("ACCEL 100 ALWAYS\n")
  mod.write("DECEL 100 ALWAYS\n")
  mod.write("ABS.SPEED OFF\n")
  mod.write(";" + "*"*80 + "\n")


def writeBaseToolStatement(mod, statement):
  '''
  Convert to base and tool statement
  '''
  global currentbase, currenttool

  if statement.Type == VC_STATEMENT_DEFINE_BASE:
    currentbase = statement.Base.Name
    mod.write("BASE %s\n" % (currentbase))
  elif statement.Type == VC_STATEMENT_DEFINE_TOOL:
    currenttool = statement.Tool.Name
    mod.write("TOOL %s\n" % (currenttool))

def writeMotionStatement(mod, statement):
  '''
  Check for base and tool change and write JMOVE and LMOVE
  '''
  global  currentspeed, currentjointspeed, currentaccuracy, currentconfig, currentbase, currenttool
  
  newbase = statement.Base
  #Check for base change
  if statement.Base != None:
    newbase = statement.Base.Name
    if currentbase != newbase:
      mod.write("BASE %s\n" % (newbase))
  elif currentbase != newbase:
    mod.write("BASE NULL\n")
  currentbase = newbase

  #Check for tool change
  newtool = statement.Tool
  if statement.Tool != None:
    newtool = statement.Tool.Name
    if currenttool != newtool:
      mod.write("TOOL %s\n" % (newtool))
  elif currenttool != newtool:
    mod.write("TOOL NULL\n")
  currenttool = newtool

  #Check for accuracy value change
  if currentaccuracy != statement.AccuracyValue:
    currentaccuracy = statement.AccuracyValue
    mod.write("ACCURACY %s ALWAYS\n" % (currentaccuracy))

  #PTP statement 
  if statement.Type == VC_STATEMENT_PTPMOTION:
    if currentjointspeed != statement.JointSpeed: #Check speed change
      currentjointspeed = statement.JointSpeed
      jspeed = currentjointspeed * 100
      mod.write("SPEED %s\n" % (jspeed))
    writeRobotConfig(mod, statement)
    mod.write("JMOVE ")
    mod.write("#")
    mod.write(statement.Positions[0].Name)
    mod.write("\n")
  #LIN statement
  elif statement.Type == VC_STATEMENT_LINMOTION:
    if currentspeed != statement.MaxSpeed: #Check speed change
      currentspeed = statement.MaxSpeed
      lspeed = str(currentspeed)+' MM/S'
      mod.write("SPEED %s\n" %lspeed)
    mod.write("LMOVE ")
    mod.write("#")
    mod.write(statement.Positions[0].Name)
    mod.write("\n")

def writeDelay(mod, statement):
  '''
  Convert to delay statement
  '''
  delay = statement.Delay
  mod.write("TWAIT %.1f" % delay)
  mod.write("\n")

def writeWaitBin(mod, statement):
  '''
  Convert to wait signal
  '''
  signal_number = statement.InputPort
  signal_value = statement.InputValue
  if signal_value == True:
    mod.write("SWAIT %s" % signal_number)
  else:
    mod.write("SWAIT -%s" % signal_number)
  mod.write("\n")

def writeSetBin(mod, statement):
  '''
  Convert to set digital signal
  '''
  signal_number = statement.OutputPort
  signal_value = statement.OutputValue
  if signal_value == True:
    mod.write("SIGNAL %s" % signal_number)
  else:
    mod.write("SIGNAL -%s" % signal_number)
  mod.write("\n")

def writeHalt(mod, statement):
  '''
  Convert to halt statement
  '''
  mod.write("HALT")
  mod.write("\n")


def callSubroutine(mod, statement):
  '''
  Convert to subroutines
  '''
  subRoutine = statement.Routine
  subRoutine_name = subRoutine.Name
  mod.write("CALL %s\n" %subRoutine_name)

def unknown(mod, statement):
  print '> Unsupported statement type skipped:', statement.Type

#Get the statement type  
def WriteProgramBody(routine, name, filename):
  '''
  Initialization of tool, base, speed, accuracy
  Check VC statements and call other functions to write AS syntax 
  '''
  global currentspeed, currentjointspeed, currentaccuracy, currentbase, currenttool, currentconfig, bases, tools

  #set current speed to undefined value, the first statement sets the speed
  currentspeed = -1
  currentjointspeed = -1
  currentaccuracy = -1
  #set current base and tool to NULL, the next statement sets the base and tool
  currentbase = None
  currenttool = None
  #Initialize configuration
  currentconfig = -1

  bases = []
  tools = []

  try:
    mod = open(filename, "w")
  except:
    print "Cannot open file \'%s\' for writing" % filename
    return False


  #MainRoutine
  mod.write(".PROGRAM %s()\n" %routine.Name)

  #Write initialisation values from the first statement
  for statement in routine.Statements:
    if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      WriteProgramInit(mod, statement)
      break

  for statement in routine.Statements:
    translator = statement_translators.get(statement.Type, unknown)
    translator(mod,statement)
  mod.write(".END\n")
  
  subRoutines = []

  #SubRoutine
  for statement in routine.Statements:
    if statement.Type == VC_STATEMENT_CALL:
      subRoutine = statement.Routine
      subRoutines.append(subRoutine)
      subRoutine_name = subRoutine.Name
      mod.write(".PROGRAM %s()\n" %subRoutine_name)
      for statement in subRoutine.Statements:
        translator = statement_translators.get(statement.Type, unknown)
        translator(mod,statement)
      mod.write("RETURN\n")
  mod.write(".END\n")

  #MainRoutine Transformation points (X, Y, Z, O, A, T) 
  mod.write(";" + "*"*80)
  mod.write("\n")
  mod.write(".TRANS")
  mod.write("\n")

  for statement in routine.Statements:
    if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      #Write the cartesian points for PTP and LIN Statement
      WriteTransformationPoints(mod,statement)

  #SubRoutines Transformation points (X, Y, Z, O, A, T) 
  if len(subRoutines) > 0:
    for routine in subRoutines:
      for statement in routine.Statements:
        if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
          WriteTransformationPoints(mod, statement)

  for statement in routine.Statements:
    if statement.Type == VC_STATEMENT_DEFINE_BASE:  
      bases.append(statement)
    if statement.Type == VC_STATEMENT_DEFINE_TOOL:
      tools.append(statement)
  for b in bases:
    if not b:
      continue
    #Write the cartesian points for BASE Statement
    WriteBasePoints(mod, b)
  for t in tools:
    if not t:
      continue
    #Write the cartesian points for TOOL Statement
    WriteToolPoints(mod, t)
  mod.write(".END\n")


  #MainRoutine Persicion points(J1, J2, J3, J4, J5, J6)
  mod.write(";" + "*"*80)
  mod.write("\n")
  mod.write(".JOINTS")
  mod.write("\n")
  for statement in routine.Statements:
    if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
      WritePrecisionPoints(mod,statement)
  
  #SubRoutines Persicion points(J1, J2, J3, J4, J5, J6)
  if len(subRoutines) > 0:
    for routine in subRoutines:
      for statement in routine.Statements:
        if statement.Type in [VC_STATEMENT_PTPMOTION, VC_STATEMENT_LINMOTION]:
          WritePrecisionPoints(mod, statement)  
  mod.write(".END\n")

  mod.close()
  return True

#-------------------------------------------------------------------------------------
def postProcess(app, program, uri):
  '''
  Initial function when calling post-processor
  '''
  global motiontarget

  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-3]

  filenamelist=[]
  motiontarget = program.Executor.Controller.createTarget()
  routine = program.MainRoutine
  filenamelist.append(uri)
  if not WriteProgramBody(routine, mainName, uri  ):
    return False,filenamelist
  #endif
  return True,filenamelist

statement_translators = {
VC_STATEMENT_DEFINE_BASE:writeBaseToolStatement,
VC_STATEMENT_DEFINE_TOOL:writeBaseToolStatement,
VC_STATEMENT_PTPMOTION:writeMotionStatement,
VC_STATEMENT_LINMOTION:writeMotionStatement,
VC_STATEMENT_DELAY:writeDelay,
VC_STATEMENT_WAITBIN:writeWaitBin,
VC_STATEMENT_SETBIN:writeSetBin,
VC_STATEMENT_HALT:writeHalt,
VC_STATEMENT_CALL:callSubroutine
}