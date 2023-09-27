#---------------------
#                    |
#  Version 1.05      |
#                    |
#---------------------
#
#Release notes:
#
#-Supports Yaskawa robots in eCat. Do validate that eCat model has correct joint signs and zero offsets.
#
#-Supports pulse and rectangular jobs.
#  -By default pp produces rectangular jobs. Pulse jobs can be made if:
#     -'UsePulses' set to True.
#     -ALL.PRM or RC.PRM is given for pulse ratios. In case robot is not first if RC parameters you can create 'RCXG' string property to robot and give it matching value, e.g. 'RC2G'.
#       OR
#     -Robot has string property 'PulsesPerRev' which contains gear ratios (pulses per axis revolution) as a list separated with ';' e.g. '100000;200000;300000;400000;500000;600000'
#       -PPR values can be calculated from robot's RC1G parameters (in ALL.prm) 972-977 (gear reduction ratio denominators) and 980-985 (gear reduction ratio numerators). 
#        Value is 4096*num/denom (4096 encoder pulses per motor revolution). Note that sign can be negative if model's positive axis direction is different than on real robot.
#          -S=4096*RC1G980/RC1G972
#          -L=4096*RC1G981/RC1G973
#          -U=4096*RC1G982/RC1G974
#          -R=4096*RC1G983/RC1G975
#          -B=4096*RC1G984/RC1G976
#          -T=4096*RC1G985/RC1G977
#     -Robot has string property 'PulsesZeros' which contains joint zero offsets (in pulses) as a list separated with ';' e.g. '0;0;0;0;250000;0'
#       -pz values are zeroes if robot model's zero positions are set correctly.
#  -In rectangular jobs all motions in a routine should use the same base frame. If not so, first base frame is used for the job and all motions are converted to that frame.
#     -*Null* and UFRAME0 bases are considered as ROBOT Frame in job file.
#     -To force jobs to be converted into a some frame create string property 'ConvertToFrame' to robot and set value to 'ROBOT' or one of the base frame names (e.g. UFRAME1).
#
#-Supports Position Level to some degree.
#   -To activate PL enable boolean property 'UsePL'.
#     -For motions with zero acceleration PL is not used
#     -PL is either 0 for Accuracy value 0 and 1 otherwise
#
#-Creates one job file per routine. Main routine's file name is selected with dialog, other files are created with names of the subroutines.
#
#-Creates support files TOOL.CND and UFRAME.CND which contains TOOL and UFRAME data for used frames.
#
#-Supports external axis to some degree.
#  -Track and positioner are supported. Servo tools are not supported.
#  -For pulse jobs external axis components should have these these properties:
#     -String property 'PulsesPerRev' with StepValues list which contains gear ratios (pulses per axis revolution (positioner) / pulses per mm (track)) as a list separated with ';' e.g. '100000;200000'
#     -String property 'PulsesZeros' with StepValues list which contains joint zero offsets (in pulses) as a liset separated with ';' e.g. '0;0'
#  -Supports sync moves 'UseSyncMoves'.
#     -If base frame is attached to positioner's flange, sync moves are used for linear motions (SMOVL).
#
#End release notes.
#
#---------------------

from vcCommand import *
from math import *
import vcMatrix
import vcVector
import re, os, os.path, sys, string, time
import locale

EPSILON = 1e-6
RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0

#-------------------------------------------------------------------------------  
def postProcess(app,program,uri):
  #Post-Processor entry point
  global comp, controller, motiontarget
  global jobfolder
  global usePulses, useSyncMoves, usePL, convertToFrame
  global bases, tools
  global usedBases, usedTools
  global compSTATION, posCmpSTATION, trackCmpSTATION
  global robotAxes, trackAxes, posAxes
  global posCmp, trackCmp, posIsR2, r2Tool
  global ppr, pz
  
  #This is to keep all numeric conversions consistent across platform locales.
  locale.setlocale(locale.LC_NUMERIC,'C')
  
  #get basic handles
  cmd = getCommand()
  comp = program.Executor.Component
  controller = program.Executor.Controller
  motiontarget = controller.createTarget()
  currentTarget = controller.createTarget()

  #get job folder
  if uri[len(uri)-8:] == '.JBI.JBI':
    uri = uri[:len(uri)-4]
  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-4]
  idx = mainName.find( '-' ) 
  if idx > -1:
    mainName = mainName[:idx]
  jobfolder = head + '\\'

  #--
  #Settings
  #--
  #PULSE vs. RECTAN job
  usePulses = cmd.getProperty('Use Pulses').Value
  #Use sync moves (SMOVL)
  useSyncMoves = cmd.getProperty('Use Sync moves (positioner)').Value
  #Position level
  usePL = cmd.getProperty('Use PL').Value
  #Convert positions to frame in RECTAN job
  convertToFrame = ''
  prop = cmd.getProperty('Convert to Frame')
  if prop and prop.Value != 'OFF':
    convertToFrame = prop.Value
  
  #get robot joints
  robotAxes = len([ j for j in controller.Joints if not j.ExternalController ])
  trackAxes = 0
  posAxes = 0
  posCmp = None
  posIsR2 = False
  r2Tool = -1
  trackCmp = None
  ppr = []
  pz = []
  propPPR = comp.getProperty('PulsesPerRev')
  propz  = comp.getProperty('PulsesZeros')
  if propPPR and propz:
    sppr = propPPR.Value.split(';')
    spz = propz.Value.split(';')
    if len(sppr) == robotAxes and len(spz) == robotAxes:
      for j in range(robotAxes):
        try:
          ppr.append( float(sppr[j]) )
          pz.append( float(spz[j]) )
        except:
          pass

  #get robot positioner joints
  for bhr in comp.getBehavioursByType(VC_ONETOONEINTERFACE):
    if bhr.IsAbstract:
      continue
    trackCmp = bhr.ConnectedComponent
    trackAxes = 0
    if trackCmp:
      ctrls = trackCmp.findBehavioursByType(VC_SERVOCONTROLLER)
      if ctrls:
        trackCnt = ctrls[0]
        if trackCnt:
          trackAxes = trackCnt.JointCount
          if trackAxes:
            propPPR = trackCmp.getProperty('PulsesPerRev')
            propz  = trackCmp.getProperty('PulsesZeros')
            if propPPR and propz:
              sppr = propPPR.Value.split(';')
              spz = propz.Value.split(';')
              if len(sppr) == trackAxes and len(spz) == trackAxes:
                for j in range(trackAxes):
                  try:
                    ppr.append( float(sppr[j]) )
                    pz.append( float(spz[j]) )
                  except:
                    pass

  #get workpiece positioners joints
  for bhr in comp.getBehavioursByType(VC_ONETOMANYINTERFACE):
    if not bhr.IsAbstract:
      continue
    if not bhr.ConnectedComponent:
      continue
    posCmp = bhr.ConnectedComponent
    posAxes = 0
    if posCmp:
      try:
        posCnt = posCmp.findBehavioursByType(VC_SERVOCONTROLLER)[0]
      except:
        posCnt = posCmp.findBehavioursByType(VC_ROBOTCONTROLLER)[0]
        posIsR2 = True      
      if posIsR2:
        mt = posCnt.createTarget()
        match = re.search('\d+', mt.ToolName, re.I|re.S)
        if match:
          try:
            r2Tool = int(match.group(0))
          except:
            pass
      if posCnt:
        posAxes += posCnt.JointCount
        if posAxes:
          propPPR = posCmp.getProperty('PulsesPerRev')
          propz  = posCmp.getProperty('PulsesZeros')
          if propPPR and propz:
            sppr = propPPR.Value.split(';')
            spz = propz.Value.split(';')
            if len(sppr) == posAxes and len(spz) == posAxes:
              for j in range(posAxes):
                try:
                  ppr.append( float(sppr[j]) )
                  pz.append( float(spz[j]) )
                except:
                  pass
  #
  if usePulses:
    readPulseRatios()
  if len(ppr) < len(motiontarget.JointValues) or len(pz) < len(motiontarget.JointValues):
    if usePulses:
      print 'WARNING! Using pulses but PPR/PZ data is invalid! Using default PPR/PZ values.'
      print 'len(ppr):', len(ppr), 'len(pz):', len(pz), 'len(motiontarget.JointValues):', len(motiontarget.JointValues)
    ppr = []
    pz = []
    for i in range(len(motiontarget.JointValues)):
      ppr.append(100000)
      pz.append(0)
      
  #make a list of tools and bases used in this program
  bases = []
  for b in controller.Bases:
    bases.append(b.Name)
  tools = []
  for t in controller.Tools:
    tools.append(t.Name)
  usedBases = []
  usedTools = []

  #get station names for job header
  prop = comp.getProperty( 'STATION' )
  if prop:
    compSTATION = prop.Value
  else:
    compSTATION = 'RB1'
  #track
  if trackCmp:
    prop = trackCmp.getProperty( 'STATION' )
    if prop:
      trackCmpSTATION = prop.Value
    else:
      trackCmpSTATION = 'BS1'
  #positioner
  if posCmp:
    prop = posCmp.getProperty( 'STATION' )
    if prop:
      posCmpSTATION = prop.Value
    else:
      posCmpSTATION = 'ST1'
      
  #main routine
  filenamelist=[]
  filenamelist.append(uri)
  if writeJob(program.MainRoutine,mainName) == False:
    print 'Failed to write mainroutine'
    return False,filenamelist

  #subroutines
  for routine in program.Routines:
    filename = head + '\\' + routine.Name + '.JBI'
    filenamelist.append(filename)
    if writeJob(routine,routine.Name) == False:
      print 'Failed to write subroutine \'%s\'' % routine.Name
      return False,filenamelist

  #return to initial position
  controller.moveImmediate( currentTarget )
  
  #Write frame info
  writeFrameInfo(filenamelist)

  return True,filenamelist
#-------------------------------------------------------------------------------
def getProperties():
  #Properties for action panel
  props = [] #type, name, def_value, constraints, step_values, min_value, max_value
  props.append((VC_BOOLEAN, 'Use Pulses', False, None, None, 0, 0))
  props.append((VC_URI, 'ALL.PRM or RC.PRM', '', None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Use Sync moves (positioner)', True, None, None, 0, 0))
  props.append((VC_BOOLEAN, 'Use PL', False, None, None, 0, 0))
  frames = ['OFF', 'ROBOT'] + [('UFRAME' + str(x)) for x in range(1,17)]
  props.append((VC_STRING, 'Convert to Frame', 'OFF', VC_PROPERTY_STEP, frames, 0, 0))
  return props
#-------------------------------------------------------------------------------
def readPulseRatios():
  global ppr, pz
  
  #Read pulse ratios (PPR, PZ) from ALL.PRM or RC.PRM file
  cmd = getCommand()
  uri = cmd.getProperty('ALL.PRM or RC.PRM').Value[8:]
  if not uri or not os.path.exists(uri):
    return False
  
  with open(uri, 'r') as file:
    data = file.read()
  
  all_ppr = []
  all_pz = []
  comps = [comp, trackCmp, posCmp]
  axes = [robotAxes, trackAxes, posAxes]
  axis_index = 0
  for i in range(3):
    mech_comp = comps[i]
    mech_axes = axes[i]
    if not mech_comp or not mech_axes:
      continue
    
    #Find ratios in params
    header = '///RC%iG' % (i + 1)
    if mech_comp.getProperty('RCXG'):
      header = '///%s' % (mech_comp.getProperty('RCXG').Value)
    footer = '///'
    exp = '%s(%s?)%s' % (header, '.+', footer)
    match = re.search(exp, data, re.I|re.S)
    if not match:
      continue
    rc_data = match.group(1)
    lines = [x for x in rc_data.split('\n') if x]
    use_abs = True
    if len(lines) < 100:
      continue
    if len(lines) < 200:
      #Old controller (e.g. DX), denoms at 972-> and nums at 980->
      denoms = lines[97].split(',')
      nums = lines[98].split(',')
      if len(denoms) >= 10:
        denoms = denoms[2:]
    else:
      #New controller (e.g. YRC)
      denoms = lines[109].split(',')
      nums = lines[110].split(',')
    if len(denoms) < 8 or len(nums) < 8:
      continue
    
    #Init ppr and pz
    mech_ppr = mech_axes * [1000.0]
    mech_pz = mech_axes * [0.0]
    ppr_prop = mech_comp.getProperty('PulsesPerRev')
    pz_prop = mech_comp.getProperty('PulsesZeros')
    if ppr and pz_prop:
      try:
        backup_ppr = ppr_prop.Value.split(';')
        backup_pz = pz_prop.Value.split(';')
        for j in range(len(backup_ppr)):
          mech_ppr[j] = float(backup_ppr[j])
          mech_pz[j] = float(backup_pz[j])
      except:
        pass
    
    #Do the mapping
    for j in range(mech_axes):
      if True:#try:
        joint = controller.Joints[axis_index]
        num = float(nums[j])
        denom = float(denoms[j])
        if denom == 0:
          continue
        if joint.Type in [VC_JOINTTYPE_TRANSLATIONAL, VC_JOINT_TRANSLATIONAL]:
          #Translational
          pass#Not supported ATM, ratios have to be set manually
        else:
          #Rotational
          if use_abs:
            mech_ppr[j] = abs(4096.0 * num / denom)
          else:
            mech_ppr[j] = 4096.0 * num / denom
        all_ppr.append(mech_ppr[j])
        all_pz.append(mech_pz[j])
      #except:
      #  pass
      axis_index += 1
    
    #Save as property
    if not ppr_prop:
      ppr_prop = mech_comp.createProperty(VC_STRING, 'PulsesPerRev')
      ppr_prop.IsVisible = False
    ppr_value = ''
    for j in range(len(mech_ppr)):
      ppr_value += str(int(mech_ppr[j]))
      if j < len(mech_ppr) - 1:
        ppr_value += ';'
    ppr_prop.Value = ppr_value
    if not pz_prop:
      pz_prop = mech_comp.createProperty(VC_STRING, 'PulsesZeros')
      pz_prop.IsVisible = False
    pz_value = ''
    for j in range(len(mech_pz)):
      pz_value += str(int(mech_pz[j]))
      if j < len(mech_pz) - 1:
        pz_value += ';'
    pz_prop.Value = pz_value
  #endfor
  
  #Set globals
  if len(all_ppr) == len(controller.Joints) and len(all_pz) == len(controller.Joints):
    ppr = all_ppr
    pz = all_pz
#-------------------------------------------------------------------------------
def writeJob(routine, name):
  #Write a job into a file
  global jobfolder
  global usePulses, useSyncMoves, usePL, convertToFrame
  global compSTATION, trackCmpSTATION, posCmpSTATION
  global robotAxes, trackAxes, posAxes
  global Ccount, BCcount, ECcount
  global cxxxxx, bcxxxxx, ecxxxxx, inst, indent
  global posIndex
  global tsync
  global initbase, inittool, prevtool, prevrconf, prevt4, prevt6
  global pointsWereConverted
  
  jbiname = getJobName(name)
  jbifilename = jobfolder + jbiname + '.JBI'

  try:
    jbi = open(jbifilename,'w')
  except:
    print 'Cannot open file \'%s\' for writing' % jbifilename
    return False

  cxxxxx = []
  bcxxxxx = []
  ecxxxxx = []
  inst = []
  indent = ''

  Ccount = 0
  BCcount = 0
  ECcount = 0
  
  posIndex = -1
  tsync = 0
  
  initbase = -2
  inittool = -2
  prevtool = -2
  prevrconf = -1
  prevt4 = -1
  prevt6 = -1
  frame = 'ROBOT'
  
  pointsWereConverted = False

  if convertToFrame:
    initbase = getFrameIndex(convertToFrame, 'UFRAME', bases )
  
  writeStatement = getAllStatementWriters()
  for statement in routine.Statements:
    writeStatement.get(statement.Type,unhandled)(statement)
  
  if initbase < -1:
    initbase = -1
  
  if inittool < -1:
    inittool = -1
  
  #Header
  jbi.write('/JOB\n')
  jbi.write('//NAME %s\n' % jbiname)
  jbi.write('//POS\n')
  jbi.write('///NPOS %i,%i,%i,0,0,0\n' % (Ccount,BCcount,ECcount))
  jbi.write('///TOOL %i\n' % inittool)
  if usePulses:
    jbi.write('///POSTYPE PULSE\n')
    jbi.write('///PULSE\n')
  else:
    jbi.write('///RECTAN\n')
    if initbase == -1 or initbase == 0:
      frame = 'ROBOT'
      jbi.write('///POSTYPE ROBOT\n' )
      if pointsWereConverted:
        print 'Multiple base frames used in job ' + name + '. Coverting points to ROBOT frame.'
    else:
      frame = 'USER %i' % initbase
      jbi.write('///USER %i\n' % initbase)
      jbi.write('///POSTYPE USER\n' )
      if pointsWereConverted:
        print 'Multiple base frames used in job ' + name + '. Coverting points to UFRAME ' + str(initbase) + '.'
  
  for line in cxxxxx:
    jbi.write(line)
  
  if trackAxes:
    if prevtool != 0:
      jbi.write('///TOOL 0\n')
    if not usePulses:
      jbi.write('///RCONF 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' )
    for line in bcxxxxx:
      jbi.write(line)
  
  if posAxes:
    for line in ecxxxxx:
      jbi.write(line)
  
  #Instructions
  jbi.write('//INST\n')
  jbi.write('///DATE %s\n' % (time.strftime('%Y/%m/%d %H:%M')))
  jbi.write('///ATTR SC,RW')
  if not usePulses:
    jbi.write(',RJ')
  jbi.write('\n')
  if not usePulses:
    jbi.write('////FRAME %s\n' % frame )
  jbi.write('///GROUP1 %s' % compSTATION )  
  if trackAxes:
    jbi.write(',%s' % trackCmpSTATION )
  if not useSyncMoves:
    if posCmp:
      jbi.write(',%s' % posCmpSTATION )
    jbi.write('\n')
  else:
    jbi.write('\n')
    if posCmp:
      jbi.write('///GROUP2 %s\n' % posCmpSTATION )
  jbi.write('NOP\n')

  for line in inst:
    jbi.write( line )

  jbi.write('END\n')
  jbi.close()

  return True
#-------------------------------------------------------------------------------
def writeFrameInfo(filenamelist):
  #Creates a support files TOOL.CND and UFRAME.CND which contains TOOL and UFRAME data for used frames.
  global comp, controller
  global usedBases, usedTools

  #UFRAME.CND
  filename = jobfolder + 'UFRAME.CND'
  
  filenamelist.append(filename)

  try:
    file = open(filename,'w')
  except:
    print 'Cannot open file \'%s\' for writing' % filename
    return False
    
  rm = comp.WorldPositionMatrix
  kin = comp.findBehaviour('Kinematics')
  if kin:
    offset_p = kin.getProperty('L01Z')
    if offset_p:
      rm.translateRel(0,0,offset_p.Value)
  irm = vcMatrix.new(rm)
  irm.invert()
  
  for b in controller.Bases:
    if b.Name in usedBases:
      if b.Node:
        bm = b.Node.WorldPositionMatrix * b.PositionMatrix
        m = irm * bm
      else:
        #Reference is robot world 
        #Note, might produce faulty coordinates if robot is on track or world frame has been moved
        m = b.PositionMatrix
      #
      index = getFrameIndex(b.Name,'UFRAME',tools)
      if index < 1:
        #*NULL* or UFRAME0 (=ROBOT frame), skip
        continue
      name = ''
      if len(b.Name) < 6 or b.Name[0:6] != 'UFRAME':
        name = ' ' + goodName(b.Name)
      file.write('//UFRAME %i\n' % index)
      file.write('///NAME%s\n' % name)
      file.write('///TOOL 0\n')
      file.write('///GROUP 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n')
      file.write('///PULSE\n')
      file.write('////RORG C000=0,0,0,0,0,0\n')
      file.write('////RXX C001=0,0,0,0,0,0\n')
      file.write('////RXY C002=0,0,0,0,0,0\n')
      file.write('////BUSER %.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n' % (m.P.X,m.P.Y,m.P.Z,m.WPR.X,m.WPR.Y,m.WPR.Z))
      
  file.close()
  
  #TOOL.CND
  filename = jobfolder + 'TOOL.CND'
  
  filenamelist.append(filename)
  
  try:
    file = open(filename,'w')
  except:
    print 'Cannot open file \'%s\' for writing' % filename
    return False
    
  t0m = vcMatrix.new()
  try:
    t0m = controller.FlangeNode.WorldPositionMatrix
  except:
    pass
  it0m = vcMatrix.new(t0m)
  it0m.invert()
  
  for i in range(64):
    t = None
    name = ''
    m = vcMatrix.new()
    index = i
    if i < len(controller.Tools):
      t = controller.Tools[i]
      if t.Name in usedTools:
        if t.Node:
          tm = t.Node.WorldPositionMatrix * t.PositionMatrix
        else:
          tm = t.PositionMatrix
        m = it0m * tm
        #
        index = getFrameIndex(t.Name,'TOOL',tools)
        name = ''
        if len(t.Name) < 4 or t.Name[0:4] != 'TOOL':
          name = ' ' + goodName(t.Name)
    file.write('//TOOL %i\n' % index)
    file.write('///NAME%s\n' % name)
    file.write('%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n' % (m.P.X,m.P.Y,m.P.Z,m.WPR.X,m.WPR.Y,m.WPR.Z))
    file.write('0.000,0.000,0.001\n')
    file.write('0.001\n')
    file.write('0.000,0.000,0.000\n')
    file.write('0.000,0,1\n')
      
  file.close()
#-------------------------------------------------------------------------------
def writeCall(statement):
  global inst, indent
  inst.append('%sCALL JOB:%s\n' % (indent, goodName(statement.Routine.Name)))
#-------------------------------------------------------------------------------
def writeComment(statement):
  global inst, indent
  inst.append('%s\'%s\n' % (indent, statement.Comment))
#-------------------------------------------------------------------------------
def writeDefineBase(statement):
  global inst, indent
  relative = ''
  if statement.IsRelative:
    relative = 'RELATIVE'
  p = statement.Position.P
  e = calcRxRyRz( statement.Position )
  if statement.Base:
    name = statement.Base.Name
  else:
    name = '*NULL*'
  inst.append('%s\'DEFINE %s BASE %s %g,%g,%g,%g,%g,%g\n' % (indent,relative,name,p.X,p.Y,p.Z,e.X,e.Y,e.Z))
#-------------------------------------------------------------------------------
def writeDefineTool(statement):
  global inst, indent
  relative = ''
  if statement.IsRelative:
    relative = 'RELATIVE'
  p = statement.Position.P
  e = calcRxRyRz( statement.Position )
  if statement.Tool:
    name = statement.Tool.Name
  else:
    name = '*NULL*'
  inst.append('%s\'DEFINE %s TOOL %s %g,%g,%g,%g,%g,%g\n' %(indent,relative,name,p.X,p.Y,p.Z,e.X,e.Y,e.Z))
#-------------------------------------------------------------------------------
def writeDelay(statement):
  global inst, indent
  inst.append('%sTIMER T=%g\n' % (indent,statement.Delay))
#-------------------------------------------------------------------------------
def writeGrasp(statement):
  global inst, indent
  inst.append('%s\'GRASP\n' % indent)
#-------------------------------------------------------------------------------
def writeHalt(statement):
  global inst, indent
  inst.append('%sPAUSE\n' % indent)
#-------------------------------------------------------------------------------
def writeHome(statement):
  global inst, indent
  inst.append('%s\'MOVE HOME\n' % indent)
#-------------------------------------------------------------------------------
def writeIf(statement):
  global inst, indent
  
  condition = formatCondition(statement.Condition)
  inst.append('%sIFTHENEXP %s\n' % (indent,condition))

  writeStatement = getAllStatementWriters()
  for child_statement in statement.ThenScope.Statements:
    indent = doIndent(indent, True)
    writeStatement.get(child_statement.Type,unhandled)(child_statement)
    indent = doIndent(indent, False)
  inst.append('%sELSE\n' % indent)
  for child_statement in statement.ElseScope.Statements:
    indent = doIndent(indent, True)
    writeStatement.get(child_statement.Type,unhandled)(child_statement)
    indent = doIndent(indent, False)
  
  inst.append('%sENDIF\n' % indent)
#-------------------------------------------------------------------------------
def writeWhile(statement):
  global inst, indent
  
  condition = formatCondition(statement.Condition)
  inst.append('%sWHILEEXP %s\n' % (indent,condition))

  writeStatement = getAllStatementWriters()
  for child_statement in statement.Scope.Statements:
    indent = doIndent(indent, True)
    writeStatement.get(child_statement.Type,unhandled)(child_statement)
    indent = doIndent(indent, False)
  
  inst.append('%sENDWHILE\n' % indent)
#-------------------------------------------------------------------------------
def writeLinMotion(statement):
  global controller, motiontarget
  global usePL
  global posAxes, trackAxes
  global inst, indent
  global posIndex

  statement.writeToTarget(motiontarget)
  controller.moveImmediate(motiontarget)
  
  writePosition(motiontarget)
  
  motion = 'MOVL'
  speed = '%0.1f' % statement.MaxSpeed
  jspeed = (100.0*motiontarget.JointSpeedFactor)
  
  pl = ''
  if usePL:
    if statement.Acceleration > EPSILON: # not continuous
      if statement.AccuracyValue > EPSILON:
        pl = ' PL=1'
      else:
        pl = ' PL=0'
  
  if trackAxes:
    bc = ' BC%05i' % posIndex
  else:
    bc = ''
    
  if posAxes:
    xc = 'EC'
    eIndex = posIndex
    if posIsR2:
      xc = 'C'
      eIndex +=1
    if useSyncMoves:
      if isBaseInStation(motiontarget):
        motion = 'SMOVL'
        ec = '  +MOVJ %s%05i' % (xc, eIndex)
      else:
        motion = 'MOVL'
        ec = '  +MOVJ %s%05i VJ=%s' % (xc, eIndex,jspeed)
    else:
      bc += ' EC%05i' % eIndex
      ec = ''
  else:
    ec = ''
   
  inst.append('%s%s C%05i%s V=%s%s%s\n' % (indent,motion,posIndex,bc,speed,pl,ec) )
  
  if posIsR2:
    posIndex += 1
#-------------------------------------------------------------------------------
def writePath(statement):
  global posAxes, trackAxes
  global usePL
  global inst, indent
  global motiontarget
  global posIndex

  inst.append('%s\'Move along path %s\n' % (indent,statement.Name))

  motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR

  if statement.Base == None:
    motiontarget.BaseName = ''
  else:
    motiontarget.BaseName = statement.Base.Name
  if statement.Tool == None:
    motiontarget.ToolName = ''
  else:
    motiontarget.ToolName = statement.Tool.Name
  ej = statement.ExternalJointCount    

  for i in range(statement.getSchemaSize()):
    target = statement.getSchemaValue(i,'Position')
    motiontarget.Target = target
    
    motion = 'MOVL'
    try:
      jv = motiontarget.JointValues
      for j in range(ej):
        jv[6+j] = statement.getSchemaValue(i,'E%d' % (j+1))
      motiontarget.JointValues = jv
      #
      pl = ''
      if usePL:
        if statement.getSchemaValue(i,'Acceleration') > EPSILON: # not continuous
          if statement.getSchemaValue(i,'AccuracyValue') > EPSILON:
            pl = ' PL=1'
          else:
            pl = ' PL=0'
      #
      speed = '%0.1f' % statement.getSchemaValue(i,'MaxSpeed')
      jspeed = (100.0*motiontarget.JointSpeedFactor)
    except:
      pl = ''
      #
      speed = '100.0'
      jspeed = '10.00'
      
    if trackAxes:
      bc = ' BC%05i' % posIndex
    else:
      bc = ''
    
    if posAxes:
      xc = 'EC'
      eIndex = posIndex
      if posIsR2:
        xc = 'C'
        eIndex += 1
      if useSyncMoves:
        if isBaseInStation(motiontarget):
          motion = 'SMOVL'
          ec = '  +MOVJ %s%05i' % (xc, eIndex)
        else:
          motion = 'MOVL'
          ec = '  +MOVJ %s%05i VJ=%s' % (xc, eIndex,jspeed)
      else:
        bc += ' EC%05i' % eIndex
        ec = ''
    else:
      ec = ''
    
    writePosition(motiontarget)
    
    inst.append('%s%s C%05i%s V=%s%s%s\n' % (indent,motion,posIndex,bc,speed,pl,ec) )
    
    if posIsR2:
      posIndex += 1
  #endfor
  
  inst.append('%s\'End of path %s\n' % (indent,statement.Name))
#-------------------------------------------------------------------------------
def writePosition( motiontgt ):
  global comp, controller
  global usePulses, useSyncMoves, convertToFrame
  global bases, tools
  global usedBases, usedTools
  global robotAxes, trackAxes, posAxes
  global Ccount, BCcount, ECcount 
  global cxxxxx, ecxxxxx, bcxxxxx
  global posIndex
  global initbase, inittool, prevtool, frame
  global pointsWereConverted

  #Incr index
  posIndex += 1
  
  #
  if not motiontgt.BaseName in usedBases:
    usedBases.append(motiontgt.BaseName)
  if not motiontgt.ToolName in usedTools:
    usedTools.append(motiontgt.ToolName)
  
  #Tool
  tool = getFrameIndex(motiontgt.ToolName,'TOOL',tools)
  if inittool < -1:
    inittool = tool
    prevtool = tool
  if tool != prevtool:
    cxxxxx.append('///TOOL %i\n' % tool)
    prevtool = tool
  
  #Robot position
  if usePulses:
    #PULSE
    jv = ''
    sep = '='
    for ii in range(robotAxes):
      jv += sep + '%i' % int(motiontgt.JointValues[ii]*ppr[ii]/360.0 + pz[ii] )
      sep = ','
    cxxxxx.append('C%05i%s\n' % (posIndex,jv))
  else:
    #RECTAN
    base = getFrameIndex(motiontgt.BaseName, 'UFRAME', bases ) #-1=ROBOT,0:ROBOT,1=UFRAME1 etc.
    if initbase < -1:
      initbase = base
    #Conf
    writeRCONF( True )
    #Target
    if base != initbase or initbase <= 0:
      #Need to convert target into different base. *NULL* and UFRAME0 positions are always converted to correct robot world frame.
      if base != initbase:
        pointsWereConverted = True
      if initbase > 0:
        #initbase is UFRAME1-15 or custom base frame
        base = controller.Bases[initbase]
        if base.Node:
          bm = base.Node.WorldPositionMatrix * base.PositionMatrix
        else:
          #No base node, reference to robot world frame
          rm = comp.WorldPositionMatrix
          kin = comp.findBehaviour('Kinematics')
          if kin:
            offset_p = kin.getProperty('L01Z')
            if offset_p:
              rm.translateRel(0,0,offset_p.Value)
          bm = rm * base.PositionMatrix
      else:
        #initbase is *NULL* or UFRAME0, use robot world frame
        rm = comp.WorldPositionMatrix
        kin = comp.findBehaviour('Kinematics')
        if kin:
          offset_p = kin.getProperty('L01Z')
          if offset_p:
            rm.translateRel(0,0,offset_p.Value)
        bm = rm
      ibm = vcMatrix.new(bm)
      ibm.invert()
      tm = motiontarget.getSimWorldToRobotTool()
      m = ibm * tm
      p = m.P
      e = calcRxRyRz( m )
      p = zeroOut( p, 0.001 )
      e = zeroOut( e, 0.001 )
    else:
      #Use target in reference if base is not *NULL* or UFRAME0 and base is the same as initbase.
      p = motiontgt.Target.P
      e = calcRxRyRz( motiontgt.Target )
      p = zeroOut( p, 0.001 )
      e = zeroOut( e, 0.001 )
    cxxxxx.append('C%05i=%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n' % (posIndex,p.X,p.Y,p.Z,e.X,e.Y,e.Z))
  Ccount += 1

  #Track position
  if trackAxes:
    jv = ''
    sep = '='
    for ii in range(robotAxes, robotAxes+trackAxes):
      if usePulses:
        if controller.Joints[ii].Type in [VC_JOINTTYPE_TRANSLATIONAL, VC_JOINT_TRANSLATIONAL]:
          jv += sep + '%i' % int(motiontgt.JointValues[ii]*ppr[ii] + pz[ii])
        else:
          jv += sep + '%i' % int(motiontgt.JointValues[ii]*ppr[ii]/360.0 + pz[ii] )
      else:
        jv += sep + '%.3f' % (motiontgt.JointValues[ii])
      sep = ','
    bcxxxxx.append('BC%05i%s\n' % (posIndex,jv))
    BCcount += 1

  #Positioner position
  if posAxes:
    jv = ''
    sep = '='
    for ii in range(robotAxes+trackAxes, robotAxes+trackAxes+posAxes):
      if usePulses:
        if controller.Joints[ii].Type in [VC_JOINTTYPE_TRANSLATIONAL, VC_JOINT_TRANSLATIONAL]:
          jv += sep + '%i' % int(motiontgt.JointValues[ii]*ppr[ii] + pz[ii])
        else:
          jv += sep + '%i' % int(motiontgt.JointValues[ii]*ppr[ii]/360.0 + pz[ii] )
      else:
        jv += sep + '%.3f' % (motiontgt.JointValues[ii])
      sep = ','
    if not posIsR2:
      ecxxxxx.append('EC%05i%s\n' % (posIndex,jv))
      ECcount += 1
    else:
      if r2Tool >= 0 and r2Tool != prevtool:
        cxxxxx.append('///TOOL %i\n' % r2Tool)
        prevtool = r2Tool
      cxxxxx.append('C%05i%s\n' % (posIndex+1,jv))
      Ccount += 1
  
  return posIndex
#-------------------------------------------------------------------------------
def writePrint(statement):
  global inst, indent
  inst.append('%sPRINT \'%s\'\n' % (indent,statement.Message))
#-------------------------------------------------------------------------------      
def writeProgSync(statement):
  global inst, indent
  global tsync
  try:
    tsync = (int(statement.SyncMsg)-1)%32+1
  except:
    tsync = tsync%32 + 1
  inst.append('%sTSYNC %i\n' % (indent,tsync) )
#-------------------------------------------------------------------------------
def writePtpMotion(statement):
  global controller, motiontarget
  global usePL
  global posAxes, trackAxes
  global inst, indent
  global posIndex

  statement.writeToTarget(motiontarget)
  controller.moveImmediate(motiontarget)
  
  writePosition(motiontarget)
  
  motion = 'MOVJ'
  jspeed = (100.0*statement.JointSpeed)
  
  pl = ''
  if usePL:
    if statement.AccuracyValue > EPSILON:
      pl = ' PL=1'
    else:
      pl = ' PL=0'
  
  if trackAxes:
    bc = ' BC%05i' % posIndex
  else:
    bc = ''
    
  if posAxes:
    xc = 'EC'
    eIndex = posIndex
    if posIsR2:
      xc = 'C'
      eIndex += 1
    if useSyncMoves:
      ec = '  +MOVJ %s%05i VJ=%0.2f ' % (xc, eIndex, jspeed )
    else:
      bc += ' %s%05i' % (xc, eIndex)
      ec = ''
  else:
    ec = ''

  inst.append('%s%s C%05i%s VJ=%0.2f%s%s\n' % (indent,motion,posIndex,bc,jspeed,pl,ec))
  
  if posIsR2:
    posIndex += 1
#-------------------------------------------------------------------------------
def writeRelease(statement):
  global inst, indent
  inst.append('%s\'RELEASE\n' % indent)
#-------------------------------------------------------------------------------
def writeRemoteCall(statement):
  global inst, indent
  r = goodName(statement.RemoteRoutine.split(':')[2])
  remoteCount += 1
  remoteDict[r] = remoteCount
  inst.append('%sPSTART JOB:%s SUB%i\n' % (indent,r,remoteCount))
#-------------------------------------------------------------------------------
def writeRemoteWait(statement):
  global inst, indent
  r = goodName(statement.RemoteRoutine.split(':')[2])
  inst.append('%sPWAIT SUB%i\n' % (indent,remoteDict.pop(r)) )
  remoteCount = 0
#-------------------------------------------------------------------------------
def writeSetBin(statement):
  global inst, indent
  if statement.OutputValue:
    value = 'ON'
  else:
    value = 'OFF'
  inst.append('%sDOUT OT#(%i) %s\n' %(indent,statement.OutputPort,value))
#-------------------------------------------------------------------------------
def writeSetProperty(statement):
  global inst, indent
  value_expression = formatCondition(statement.ValueExpression.strip())
  inst.append('%sSET %s %s\n' % (indent,statement.TargetProperty, value_expression))
#-------------------------------------------------------------------------------  
def writeWaitBin(statement):
  global inst, indent
  if statement.InputValue:
    value = 'ON'
  else:
    value = 'OFF'
  inst.append('%sWAIT IN#(%i)=%s\n' %(indent,statement.InputPort,value))
#-------------------------------------------------------------------------------
def isBaseInStation(motiontgt):
  #Return true, if target's base moves with station. Else otherwise
  global controller
  global posCmp
  base = None
  for b in controller.Bases:
    if b.Name == motiontgt.BaseName:
      base = b
      break
  if not base:
    return False
  ctrs = posCmp.findBehavioursByType(VC_SERVOCONTROLLER)
  if len(ctrs) == 0:
    ctrs = posCmp.findBehavioursByType(VC_ROBOTCONTROLLER)
  if len(ctrs) == 0:
    return False
  ctr = ctrs[0]
  #
  node = base.Node
  while node:
    if node == ctr.FlangeNode:
      return True
    node = node.Parent
  return False
#-------------------------------------------------------------------------------
def getJobName(name):
  if name == '':
    return 'MAIN'
  return goodName( name ) 
#-------------------------------------------------------------------------------
def goodName( inName ):
  badChars  = re.compile( '[^\w!%&\'()_-]' ) 
  outName = inName.strip()
  outName = outName.upper()
  outName = badChars.sub( '_', outName )
  if len(outName) > 32:
    print 'WARNING: Routine name <%s> exceeds 32 characters' % outName
  return outName
#-------------------------------------------------------------------------------
def doIndent(indent_string, increase):
  if increase:
    if indent_string == '':
      indent_string = '\t '
    else:
      indent_string = '\t' + indent_string
  else:
    if len(indent_string) > 0:
      indent_string = indent_string[1:]
    if len(indent_string) == 1:
      indent_string = ''
  return indent_string
#-------------------------------------------------------------------------------
def formatCondition(condition):
  #Replace equality operator
  condition = condition.replace('==', '=')
  #Replace inputs
  replace_boolean = False
  exp = 'IN\[([0-9]+)\]'
  old_condition = condition
  for match in re.finditer(exp, old_condition):
    replace_boolean = True
    condition = condition.replace(match.group(0), 'IN#(%s)' % (match.group(1)))
  #Replace outputs
  exp = 'OUT\[([0-9]+)\]'
  old_condition = condition
  for match in re.finditer(exp, old_condition):
    replace_boolean = True
    condition = condition.replace(match.group(0), 'OUT#(%s)' % (match.group(1)))
  #
  #Replace Boolean literals
  if replace_boolean:
    condition = condition.replace('True', 'ON')
    condition = condition.replace('False', 'OFF')
    condition = condition.replace('=1', '=ON')
    condition = condition.replace('=0', '=OFF')
  #
  #Replace logical operators
  condition = condition.replace('&&', ' AND ')
  condition = condition.replace('&', ' AND ')
  condition = condition.replace('||', ' OR ')
  condition = condition.replace('|', ' OR ')
  condition = condition.replace('!', ' NOT ')
  condition = condition.replace('  ', ' ')
  #
  return condition
#-------------------------------------------------------------------------------
def getFrameIndex(s,frame,slist):
  if frame == s[:len(frame)]:
    try:
      idx = int(s[len(frame):])
      return idx
    except:
      pass
  match = re.search(r'\s*-?\d+', s, re.I|re.S)
  if match:
    try:
      idx = int(match.group(0))
      return idx
    except:
      pass
  idx = getStringIndex(s,slist)
  if frame == 'UFRAME' and idx == 0:
    idx = -1 #UFRAME0 does not exist in real robot, use ROBOT instead
  elif frame == 'TOOL' and idx < 0:
    idx = 0 #Use TOOL0 if index is not found (e.g. *NULL)
  return idx
#-------------------------------------------------------------------------------
def getStringIndex(s,slist):
  #Check if s is is slist and return its index (first hit)
  i = 0
  for sx in slist:
    if sx == s:
      return i
    i += 1
  return -1
#-------------------------------------------------------------------------------
def zeroOut( p, tol ):
  if abs(p.X) < tol:
    p.X = 0.0
  if abs(p.Y) < tol:
    p.Y = 0.0
  if abs(p.Z) < tol:
    p.Z = 0.0

  return p
#-------------------------------------------------------------------------------
def calcRxRyRz( m ):
  #Set useEulerAngles True, if robot has eulers enabled
  useEulerAngles = False
  if useEulerAngles:
    A = atan2( m.A.Y, m.A.X )*RAD_TO_DEG
    B = asin( m.A.Z )*RAD_TO_DEG
    C = atan2( -m.O.Z, -m.N.Z )*RAD_TO_DEG
    return vcVector.new( A, B, C )
  else:
    return m.WPR
#-------------------------------------------------------------------------------
def writeRCONF( turns ):
  global prevrconf, prevt4, prevt6, cxxxxx, bcxxxxx

  rconf = motiontarget.RobotConfig

  jv = motiontarget.JointValues
  if turns and abs(jv[3]) > 180.0:
    t4 = 1
  else:
    t4 = 0
  if turns and abs(jv[5]) >= 180.0:
    t6 = 1
  else:
    t6 = 0

  if rconf == prevrconf and t4 == prevt4 and t6 == prevt6:
    return

  if rconf == 0:
    line = '///RCONF 1,0,0'
  elif rconf == 1:
    line = '///RCONF 0,0,0'
  elif rconf == 2:
    line = '///RCONF 1,1,0'
  elif rconf == 3:
    line = '///RCONF 0,1,0'
  elif rconf == 4:
    line = '///RCONF 1,0,1'
  elif rconf == 5:
    line = '///RCONF 0,0,1'
  elif rconf == 6:
    line = '///RCONF 1,1,1'
  elif rconf == 7:
    line = '///RCONF 0,1,1'

  line += ',%i,%i,0' %(t4,t6)

  if True:  #Controllers 'DX100', and 'DX200'
    line += ',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
  line += '\n'

  cxxxxx.append(line)

  prevrconf = rconf
  prevt4 = t4
  prevt6 = t6
#-------------------------------------------------------------------------------
def unhandled(statement):
  print 'Unhandled statement : %s %s' % (statement.Name,statement.Type)
#------------------------------------------------------------------------------- 
def getAllStatementWriters():
  statementhandlers = {
  VC_STATEMENT_BREAK:unhandled,
  VC_STATEMENT_CALL:writeCall,
  VC_STATEMENT_COMMENT:writeComment,
  VC_STATEMENT_CONTINUE:unhandled,
  VC_STATEMENT_CUSTOM:unhandled,
  VC_STATEMENT_DEFINE_BASE:writeDefineBase,
  VC_STATEMENT_DEFINE_TOOL:writeDefineTool,
  VC_STATEMENT_DELAY:writeDelay,
  VC_STATEMENT_GRASP:writeGrasp,
  VC_STATEMENT_HALT:writeHalt,
  VC_STATEMENT_HOME:writeHome,
  VC_STATEMENT_IF:writeIf,
  VC_STATEMENT_LINMOTION:writeLinMotion,
  VC_STATEMENT_PATH:writePath,
  VC_STATEMENT_PRINT:writePrint,
  VC_STATEMENT_PROCESS:unhandled,
  VC_STATEMENT_PROG_SYNC:writeProgSync,
  VC_STATEMENT_PTPMOTION:writePtpMotion,
  VC_STATEMENT_RELEASE:writeRelease,
  VC_STATEMENT_REMOTECALL:writeRemoteCall,
  VC_STATEMENT_REMOTEWAIT:writeRemoteWait,
  VC_STATEMENT_RETURN:unhandled,
  VC_STATEMENT_SETBIN:writeSetBin,
  VC_STATEMENT_SETPROPERTY:writeSetProperty,
  VC_STATEMENT_WAITBIN:writeWaitBin,
  VC_STATEMENT_WHILE:writeWhile,
  }
  return statementhandlers