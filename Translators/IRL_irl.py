from vcCommand import *
import time, os.path

#IRL (DIN 66312 ) sample post-processor for VC4.0 products. V0.3

#-------------------------------------------------------------------------------
# This is to keep all numeric conversions consistent across platform locales.
import locale
locale.setlocale(locale.LC_NUMERIC,'C')

#-------------------------------------------------------------------------------
def writeWhile(mod,statement,indentation):
  cnd = statement.Condition.strip()
  mod.write(" "*indentation+"WHILE %s\n" %(cnd))
  indentation += 2
  for s in statement.Scope.Statements:
    WriteStatement(mod,s,indentation)
  indentation -= 2
  mod.write(" "*indentation+"ENDWHILE;\n")
#-------------------------------------------------------------------------------
def writeIf(mod,statement,indentation):
  cnd = statement.Condition.strip()
  mod.write(" "*indentation+"IF %s\n" %(cnd))
  indentation += 2
  for s in statement.ThenScope.Statements:
    WriteStatement(mod,s,indentation)
  indentation -= 2
  mod.write(" "*indentation+"ELSE\n")
  indentation += 2
  for s in statement.ElseScope.Statements:
    WriteStatement(mod,s,indentation)
  indentation -= 2
  mod.write(" "*indentation+"ENDIF;\n")
#-------------------------------------------------------------------------------
def writePath(mod,statement,indentation):
  global motiontarget
  mod.write(" "*indentation+"{ Move along path %s }\n" % (statement.Name))
  motiontarget.JointTurnMode = VC_MOTIONTARGET_TURN_NEAREST
  motiontarget.TargetMode = VC_MOTIONTARGET_TM_NORMAL
  motiontarget.MotionType = VC_MOTIONTARGET_MT_LINEAR    
  for i in range( statement.getSchemaSize()):
    target = statement.getSchemaValue(i,"Position")
    motiontarget.Target = target
    p = motiontarget.Target.P
    a = motiontarget.Target.getWPR()
    mod.write(" "*indentation+"MOVE LIN ROBTARGET(((%8.2f, %8.2f, %8.2f), ORIZYX(%8.2f, %8.2f, %8.2f)), 4, 100, 0) SPEED:=%g;\n" % (p.X,p.Y,p.Z,a.Z,a.Y,a.X, statement.getSchemaValue(i,"MaxSpeed")))
  mod.write(" "*indentation+"{ End of path %s }\n" % (statement.Name))
#-------------------------------------------------------------------------------
def extendStatements(statement,statements):
  statements.append(statement)
  if statement.Type == VC_STATEMENT_WHILE:
    for s in statement.Scope.Statements:
      statements = extendStatements(s,statements)
  elif statement.Type == VC_STATEMENT_IF:
    for s in statement.ThenScope.Statements:
      statements = extendStatements(s,statements)
    for s in statement.ElseScope.Statements:
      statements = extendStatements(s,statements)
  return statements
#-------------------------------------------------------------------------------
def writeSetProperty(mod,statement,indentation):
  ve = statement.ValueExpression.strip()
  mod.write(" "*indentation+"%s := %s;\n" %(statement.TargetProperty,ve))
#-------------------------------------------------------------------------------
def WriteStatement(mod,statement,indentation):
  global motiontarget,pointCount
  
  if statement.Type == VC_STATEMENT_CALL:
    mod.write(" "*indentation+"%s ;\n" % statement.getProperty("Routine").Value.Name)

  elif statement.Type == VC_STATEMENT_COMMENT:
    c = statement.Comment
    mod.write(" "*indentation+"{%s}\n" % (c))
      
  elif statement.Type == VC_STATEMENT_DELAY:
    d = statement.Delay
    mod.write(" "*indentation+"WAIT %6.2f SEC ;\n" % (d))

  elif statement.Type == VC_STATEMENT_HALT:
    mod.write(" "*indentation+"PAUSE ;\n")
    
  elif statement.Type == VC_STATEMENT_RETURN:
    mod.write(" "*indentation+"RETURN ;\n")  

  elif statement.Type == VC_STATEMENT_LINMOTION:
    statement.writeToTarget(motiontarget)
    p = motiontarget.Target.P
    a = motiontarget.Target.getWPR()
    mod.write(" "*indentation+"MOVE LIN ROBTARGET(((%8.2f, %8.2f, %8.2f), ORIZYX(%8.2f, %8.2f, %8.2f)), 4, 100, 0) SPEED:=%g;\n" % (p.X,p.Y,p.Z,a.Z,a.Y,a.X, statement.MaxSpeed))

  elif statement.Type == VC_STATEMENT_PTPMOTION:
    statement.writeToTarget(motiontarget)
    p = motiontarget.Target.P
    a = motiontarget.Target.getWPR()
    mod.write(" "*indentation+"MOVE PTP ROBTARGET(((%8.2f, %8.2f, %8.2f), ORIZYX(%8.2f, %8.2f, %8.2f)), 4, 100, 0) SPEED_PTP:=%g;\n" % (p.X,p.Y,p.Z,a.Z,a.Y,a.X, statement.JointSpeed))

  elif statement.Type == VC_STATEMENT_SETBIN:
    mod.write(" "*indentation+"%s_%i= " %(statement.Name,statement.OutputPort))
    if statement.OutputValue:
      mod.write("TRUE;\n" )
    else:
      mod.write("FALSE;\n" )
      
  elif statement.Type == VC_STATEMENT_WAITBIN:
    if statement.InputValue:
      mod.write(" "*indentation+"WAIT FOR IN_%i = TRUE;\n" %(statement.InputPort))
    else:
      mod.write(" "*indentation+"WAIT FOR IN_%i = FALSE;\n" %(statement.InputPort))
    
  elif statement.Type == VC_STATEMENT_WHILE:
    writeWhile(mod,statement,indentation)
    
  elif statement.Type == VC_STATEMENT_IF:
    writeIf(mod,statement,indentation)
    
  elif statement.Type == VC_STATEMENT_PATH:
    writePath(mod,statement,indentation)
  
  elif statement.Type == VC_STATEMENT_SETPROPERTY:
    ve = statement.ValueExpression.strip()
    mod.write(" "*indentation+"%s := %s;\n" %(statement.TargetProperty,ve))
#-------------------------------------------------------------------------------  
def WriteProgramBody( routine, name, filename ):
  global pointCount

  try:
    mod = open(filename,"w")
  except:
    print "Cannot open file \'%s\' for writing" % filename
    return False

  #header
  td = time.strftime("DATE %y-%m-%d  TIME %H:%M:%S")
  mod.write("PROGRAM %s;\n" % name)
  #mod.write("VAR\n")
    
  mod.write("BEGIN\n")
  # print robot statements
  pointCount = 0
  indentation = 2
  for statement in routine.Statements:
    WriteStatement(mod,statement,indentation)

  # end of program
  mod.write("ENDPROGRAM;\n")
  mod.close

  return True

#-------------------------------------------------------------------------------
def postProcess(app,program,uri):
  global motiontarget

  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-4]
  
  #motiontarget = controller.createTarget()
  motiontarget = program.Executor.Controller.createTarget()
  filenamelist=[]
  
  # main routine
  filenamelist.append(uri)
  if not WriteProgramBody(program.MainRoutine, mainName, uri  ):
    return False,filenamelist
  #endif  

  # subroutines
  for routine in program.Routines:
    filename = head + "\\" + routine.Name + ".irl"
    filenamelist.append(filename)
    if not WriteProgramBody(routine, routine.Name,  filename ):
      return False,filenamelist

  return True,filenamelist

