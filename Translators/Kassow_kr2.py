from vcCommand import *
import time, os.path
import json
import math
from collections import OrderedDict

# Globals
master_dict = OrderedDict()
list_tree = []
list_vars = []
temp_list = []
ref_num = 0
current_routine = ""

# Kassow post-processor for VC4.X products. V0.1b
# Attempt to follow PEP8 --> snake_case over CamelCase
"""
Abstract

The idea of this post processor is to read some objects within Visual components and convert the objects
into json. We are using OrderedDict and inputting values into the OrderedDict as list of tuples.
This ensures that the OrderedDict will have the correct insertion order which is important when writing
a json file. Insertion order for dictionaries is not kept until Python version 3.6.

First, we create a master dictionary that has some key:value pairs. The value is either constant, a
robot statement value or a mutable list. The list can be updated directly by appending data into the 
list. This allows us to update values into the rather large and growing master dictionary,
without ever looping through the dictionary itself.

Finally, we are using the imported json library to write (aka json dump method) the whole dictionary into
an user specified, or created text file with .kr2 file extension. These file(s) can then be given to a
Kassow robot. All the statements are not supported and this post processor is targeting Kassow robot
language version 0.9.0.

Written on 11th of October, 2022.
"""
#-------------------------------------------------------------------------------
# This is to keep all numeric conversions consistent across platform locales.
import locale
locale.setlocale(locale.LC_NUMERIC,'C')
#-------------------------------------------------------------------------------
def write_call(statement):
  global list_tree
  call_dict = OrderedDict([
    ("classId", 1012),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["PROGRAM", "CALL"]),
    ("arguments", OrderedDict([
      ("block", ""),
      ("isBlock", False),
      ("flatMode", False),
      ("subprogramName", statement.Routine.Name)
    ]))
  ])
  # Create a new list to ensure proper referencing
  sub_list = []
  sub_prog_dict = OrderedDict([
    ("classId", 1014),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags", ["SUBPROGRAM", "PROGRAM", "STRUCTURAL"]),
    ("arguments", OrderedDict([
      ("label", statement.Routine.Name),
      ("shortcut", False)
    ])),
    ("structure", sub_list)
  ])
  # Sub programs start at the highest level of the tree, just like main program
  list_tree.append(sub_prog_dict)
  # Loop through subroutines, empties are ignored
  for routine in statement.Routine.Statements:
    write_statement(routine, sub_list)
  return call_dict
#-------------------------------------------------------------------------------
def write_comment(statement):
  c = statement.Comment
  comment_dict = OrderedDict([
    ("classId", 1013),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["PROGRAM", "COMMENT"]),
    ("arguments", OrderedDict([
      ("textShort", c),
      ("textLong", "")
    ]))
  ])
  return comment_dict
#-------------------------------------------------------------------------------
def write_delay(statement):
  delay_dict = OrderedDict([
    ("classId", 1009),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["PROGRAM", "WAIT"]),
    ("arguments", OrderedDict([
      ("version", "1.1"),
      ("conditional", OrderedDict([
        ("argExpression", "")
        ])),
      ("interval", OrderedDict([
        ("argExpression", str(statement.Delay))
        ])),
      ("inputType", 1),
      ("conditionType", 1),
      ("timeout", OrderedDict([
        ("argExpression", "")
        ]))
    ]))
  ])
  return delay_dict
#-------------------------------------------------------------------------------
def write_return(statement):
  if current_routine == statement.ParentRoutine.Program.MainRoutine.Name:
    return_cpp_str = "return 0;"
  else:
    return_cpp_str = "return CmdResult<>::Success();"
  ret_dict = OrderedDict([
  ("classId", 1012),
  ("suppressed", False),
  ("breakpoint", False),
  ("tags",["PROGRAM", "CALL CPP", "RETURN"]),
  ("arguments", OrderedDict([
    ("block", return_cpp_str),
    ("isBlock", True),
    ("flatMode", True)
    ]))
  ])
  return ret_dict
#-------------------------------------------------------------------------------
def write_break(statement):
  break_dict = OrderedDict([
  ("classId", 1012),
  ("suppressed", False),
  ("breakpoint", False),
  ("tags",["PROGRAM", "CALL CPP", "BREAK"]),
  ("arguments", OrderedDict([
    ("block", "break;"),
    ("isBlock", True),
    ("flatMode", True)
    ]))
  ])
  return break_dict
#-------------------------------------------------------------------------------
def write_continue(statement):
  cont_dict = OrderedDict([
  ("classId", 1012),
  ("suppressed", False),
  ("breakpoint", False),
  ("tags",["PROGRAM", "CALL CPP", "CONTINUE"]),
  ("arguments", OrderedDict([
    ("block", "continue;"),
    ("isBlock", True),
    ("flatMode", True)
    ]))
  ])
  return cont_dict
#-------------------------------------------------------------------------------
def write_while(statement, nested_list):
  cnd = statement.Condition.strip()
  nested_list_while = []
  if cnd == "True":
    cnd_ = True
  elif cnd == "False":
    cnd_ = False
  else:
    cnd_ = cnd
    # make the condition (cnd_) JSON readable
    cnd_ = cnd_.replace("True", "true")
    cnd_ = cnd_.replace("False", "false")
    cnd_ = cnd_.replace("==", "=")

  ret_dict_while = OrderedDict([
    ("classId", 1003),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["DEPRECATED","INSTRUCTION","LOOP-STATEMENT"]),
    ("arguments",OrderedDict([
      ("argExpression", cnd_),
      ("syncFrequency", 1000.0),
      ("syncFrequencyEnabled", False),
      ("endlessModeEnabled", False)
    ])),
    ("structure", nested_list)
  ])  

  for s in statement.Scope.Statements:
    write_statement(s,nested_list)
  return ret_dict_while
#-------------------------------------------------------------------------------
def get_condition(statement):
  cnd = statement.Condition.strip()
  if cnd == "True":
    cnd_ = True
  elif cnd == "False":
    cnd_ = False
  else:
    cnd_ = cnd
    # make the condition (cnd_) JSON readable
    cnd_ = cnd_.replace("True", "true")
    cnd_ = cnd_.replace("False", "false")
    cnd_ = cnd_.replace("==", "=")
  return cnd_

#-------------------------------------------------------------------------------
def write_if(statement):
  cnd_ = get_condition(statement)
  if_list = []
  ret_dict_list = []
  ret_dict_if = OrderedDict([
    ("classId", 1002),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["DEPRECATED","INSTRUCTION","IF-STATEMENT"]),
    ("arguments",OrderedDict([
      ("argExpression", cnd_),
      ("structuralType", 0)
    ])),
    ("structure", if_list)
  ])
  for s in statement.ThenScope.Statements:
    write_statement(s, if_list)
  ret_dict_list.append(ret_dict_if)

  if len(statement.ElseIfScopes) > 0:
    for elsif_routine in statement.ElseIfScopes:
      cnd_ = get_condition(elsif_routine)
      else_if_list = []
      ret_dict_elseif = OrderedDict([
        ("classId", 1002),
        ("suppressed", False),
        ("breakpoint", False),
        ("tags",["DEPRECATED","INSTRUCTION","ELSEIF-STATEMENT"]),
        ("arguments",OrderedDict([
          ("argExpression", cnd_),
          ("structuralType", 1)
        ])),
        ("structure", else_if_list)
      ])
      for s in elsif_routine.Statements:
        write_statement(s, else_if_list)
      ret_dict_list.append(ret_dict_elseif)

  if len(statement.ElseScope.Statements) > 0:
    else_list = []
    ret_dict_else = OrderedDict([
      ("classId", 1002),
      ("suppressed", False),
      ("breakpoint", False),
      ("tags",["DEPRECATED","INSTRUCTION","ELSE-STATEMENT"]),
      ("arguments",OrderedDict([
        ("argExpression", ""),
        ("structuralType", 2)
      ])),
      ("structure", else_list)
    ])
    for s in statement.ElseScope.Statements:
      write_statement(s,else_list)
    ret_dict_list.append(ret_dict_else)

  return ret_dict_list
#-------------------------------------------------------------------------------
def write_move(statement):
  global ref_num, list_vars, motiontarget
  statement.writeToTarget(motiontarget)
  pos = statement.Positions[0]
  joint_val_list = pos.JointValues
  p = motiontarget.Target.P
  a = motiontarget.Target.getWPR()

  segment_type_stop = 2
  segment_type_via = 1
  
  blend_type_distance = 1
  blend_type_time = 2
  blend_type_acceleration = 3
  blend_type_angular_acceleration = 4
  blend_type = blend_type_acceleration
  blend_value = 0

  pace_type_velocity = 1
  pace_type_angular_velocity = 3
  pace_type_cycle_time = 2
  pace_type = pace_type_velocity
  pace_value = 0

  if statement.Type == VC_STATEMENT_LINMOTION:
    mv = "LIN"
    trajectory_type = 1

    # Setting via (blend) or stop (no blend) point through accuracy value
    if statement.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE:
      blend_type = blend_type_distance
      blend_value = (statement.AccuracyValue)/1000 # distance in meters
      calculated_blend_distance = pow(statement.MaxSpeed, 2)/(2*statement.Acceleration)
      if (statement.CycleTime == 0) and (calculated_blend_distance > statement.AccuracyValue):
        print "Warning, acceleration used to achive desired blend with accuracy " + str(statement.AccuracyValue) + "mm is higher than " + str(statement.Acceleration) + "mm/s2"
        print "Please set the accuracy value:", statement.AccuracyValue, "higher than", calculated_blend_distance
    elif statement.AccuracyMethod == VC_MOTIONTARGET_AM_TIME:
      blend_type = blend_type_time
      blend_value = statement.AccuracyValue # time in seconds
      calculated_blend_time = statement.MaxSpeed/statement.Acceleration
      if (statement.CycleTime == 0) and (calculated_blend_time > statement.AccuracyValue):
        print "Warning, acceleration used to achive desired blend with accuracy " + str(statement.AccuracyValue) + "s is higher than " + str(statement.Acceleration) + "mm/s2"
        print "Please set the accuracy value:", statement.AccuracyValue, "higher than", calculated_blend_time
    else:
      print "Warning, velocity as accuracy method is not supported for linear motion, skipping"

    if blend_value <= 0: # if no/unsupported blend specified
      segment_type = segment_type_stop
      blend_type = blend_type_acceleration
      blend_value = (statement.Acceleration)/1000 # acceleration in m/s2
      if statement.CycleTime > 0:
        pass
       # blend_value = # TODO set actual acceleration value used for the movement
    else:
      segment_type = segment_type_via

    if statement.CycleTime == 0:
      pace_type = pace_type_velocity
      pace_value = (statement.MaxSpeed)/1000 # speed in m/s
    else:
      pace_type = pace_type_cycle_time
      pace_value = statement.CycleTime      

  else:
    mv = "PTP"
    trajectory_type = 2
    # Grab the actual values, this could also be done upon initialization so PTP motions do not have to do this everytime
    ptp_blend_values = statement.ParentRoutine.Program.Executor.Controller.Joints
    for joint in ptp_blend_values:
      if joint.Name != "Swivel" or joint.Name != "mountplate":
        if "real_v" not in locals() and "real_a" not in locals():
          real_v = joint.MaxSpeed
          real_a = joint.MaxAcceleration
        else:
          # Get the lowest maxspeed and maxacceleration values
          if (joint.MaxSpeed < real_v):
            real_v = joint.MaxSpeed
          if (joint.MaxAcceleration < real_a):
            real_a = joint.MaxAcceleration

    # Setting via (blend) or stop (no blend) point through accuracy value
    if statement.AccuracyMethod == VC_MOTIONTARGET_AM_TIME:
      blend_type = blend_type_time
      blend_value = statement.AccuracyValue # time in seconds
      if statement.JointForce < 0.01:
        print "Warning, Joint Force lower than 1%, risk of division by zero. Setting Joint Force to 1%"
        div = 0.01
      else:
        div = statement.JointForce
      calculated_blend_time = (statement.JointSpeed*real_v)/(div*real_a)        # DONE calculate with actual values of speed/force (rad/s, not percentage)
      if (statement.CycleTime == 0) and (calculated_blend_time > statement.AccuracyValue):
        print "Warning, joint force used to achieve desired blend with accuracy " + str(statement.AccuracyValue) + "s is higher than " + str(statement.JointForce*calculated_blend_time) + "s"
        print "Please set the AccuracyValue higher than", calculated_blend_time, "seconds on statement:", statement.Name
    elif statement.AccuracyMethod == VC_MOTIONTARGET_AM_DISTANCE:
      print "Warning, distance as accuracy method is not supported for point-to-point motion, skipping. Please use Time as accuracy method."
    else:
      print "Warning, velocity as accuracy method is not supported for point-to-point motion, skipping. Please use Time as accuracy method."
  
    if blend_value <= 0: # if no/unsupported blend specified
      segment_type = segment_type_stop
      blend_type = blend_type_angular_acceleration
      blend_value = statement.JointForce*real_a    # DONE percentage -> rad/s2
      if statement.CycleTime > 0:
        pass
       # blend_value = # TODO set actual joint force value used for the movement
    else:
      segment_type = segment_type_via

    if statement.CycleTime == 0:
      pace_type = pace_type_angular_velocity
      pace_value = statement.JointSpeed*real_v      # DONE percentage -> rad/s
    else:
      pace_type = pace_type_cycle_time
      pace_value = statement.CycleTime  

  res_dict = OrderedDict([
    ("classId",1005),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags", ["DEPRECATED", "MOVE", mv]),
    ("arguments",OrderedDict([
      ("argThroughExpression",""),
      ("argTargetExpression","Ref_{}".format(ref_num)),
      ("segmentType", segment_type),
      ("syncTimeValue", 0.0),
      ("trajectoryType", trajectory_type),
      ("chainingType", 2),
      ("paceTypeMap", OrderedDict([
        ("trajectoryType_1", pace_type if (trajectory_type == 1) else 1),
        ("trajectoryType_3", pace_type if (trajectory_type == 3) else 4),
        ("trajectoryType_4", pace_type if (trajectory_type == 4) else 5),
        ("trajectoryType_2", pace_type if (trajectory_type == 2) else 3)
        ])),
      ("paceValueMap", OrderedDict([
        ("paceType_4", pace_value if (pace_type == 4) else 0.25),
        ("paceType_2", pace_value if (pace_type == 2) else 5.0),
        ("paceType_1", pace_value if (pace_type == 1) else 1.0),
        ("paceType_3", pace_value if (pace_type == 3) else 1.575)
        ])),
      ("blendTypeMap", OrderedDict([
        ("trajectoryType_1", blend_type if (trajectory_type == 1) else 3),
        ("trajectoryType_3", blend_type if (trajectory_type == 3) else 5),
        ("trajectoryType_4", blend_type if (trajectory_type == 4) else 5),
        ("trajectoryType_2", blend_type if (trajectory_type == 2) else 4)
        ])),
      ("blendValueMap", OrderedDict([
        ("blendType_5", blend_value if (blend_type == 5) else 1.0),
        ("blendType_4", blend_value if (blend_type == 4) else 1.58),
        ("blendType_1", blend_value if (blend_type == 1) else 1.0),
        ("blendType_2", blend_value if (blend_type == 2) else 5.0),
        ("blendType_3", blend_value if (blend_type == 3) else 4.0)
        ])),
        ("splineOrientation", 0),
        ("splineHorizon", 4),
        ("geomBlendValueMap", {"blendType_1" : 0.25})
    ]))
  ])
  # should've just made something called joint dict, and then loop through it to append
  # this was probably easier to implement with a simple loop, then again copy pasting is not hard
  # Some values need to be floated and then divided and THEN rounded
  var_dict = OrderedDict([
    ("type_id", 2000),
    ("reference_id", "Ref_{}".format(ref_num)),
    ("n_copy", 0),
    ("shortcut", True),
    ("label", "Ref_{}".format(ref_num)),
    ("content", OrderedDict([
      ("type_id", 3006),
      ("permissions", 3),
      ("position", OrderedDict([
        ("type_id", 3001),
        ("permissions",3),
        ("x", OrderedDict([
          ("type_id", 3000),
          ("permission", 3),
          ("value", round(float(p.X)/float(1000), 7))
        ])),
        ("y", OrderedDict([
          ("type_id", 3000),
          ("permission", 3),
          ("value", round(float(p.Y)/float(1000), 7))
        ])),
        ("z", OrderedDict([
          ("type_id", 3000),
          ("permission", 3),
          ("value", round(float(p.Z)/float(1000), 7))
        ]))
      ])),
      ("orientation",OrderedDict([
        ("type_id", 3002),
        ("permissions", 3),
        ("r", OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(a.X), 5))
        ])),
        ("p", OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(a.Y), 5))
        ])),
        ("y", OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(a.Z), 5))
        ]))
      ])),
      ("configuration", OrderedDict([
        ("type_id", 3003),
        ("permissions", 3),
        ("joint_1",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[0]), 5))
        ])),
        ("joint_2",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[1]), 5))
        ])),
        ("joint_3",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[2]), 5))
        ])),
        ("joint_4",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[3]), 5))
        ])),
        ("joint_5",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[4]), 5))
        ])),
        ("joint_6",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[5]), 5))
        ])),
        ("joint_7",OrderedDict([
          ("type_id", 3000),
          ("permissions", 3),
          ("value", round(math.radians(joint_val_list[6]), 5))
        ]))
      ]))
    ])),
    ("scope", "global")
  ])

  list_vars.append(var_dict)
  ref_num += 1
  return res_dict
#-------------------------------------------------------------------------------
def write_assign(statement):
  global list_vars
  #init
  string_conversion = "no_conversion"
  # JSON takes in true, not True. Same with false and False
  if statement.ValueExpression == "True":
    string_conversion = "true"
  elif statement.ValueExpression == "False":
    string_conversion = "false"
  else:
    string_conversion = statement.ValueExpression

  set_dict = OrderedDict([
    ("classId", 1008),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags", ["INSTRUCTION", "SET", "PROGRAM"]),
    ("arguments", OrderedDict([
      ("argLeftExpression", statement.TargetProperty),
      ("argRightExpression", string_conversion)
    ]))
  ])
  # init
  val_conversion = 0
  if statement.ValueExpression == "False":
    val_conversion = 0.0
  elif statement.ValueExpression == "True":
    val_conversion = 1.0
  else:
    val_conversion = statement.ValueExpression
    try:
      float(val_conversion)
    except:
      print "only True, False, integers or Reals supported in assign, forcing value to 0.0"
      val_conversion = 0.0

  var_dict_ = OrderedDict([
    ("type_id", 2000),
    ("reference_id", statement.TargetProperty),
    ("n_copy", 0),
    ("label", statement.TargetProperty),
    ("content", OrderedDict([
      ("type_id", 3000),
      ("permissions", 3),
      ("value",val_conversion)
    ])),
    ("scope", "global")
  ])
  list_vars.append(var_dict_)
  return set_dict
#-------------------------------------------------------------------------------
def write_statement(statement, nested = False):
  global motiontarget, list_tree, list_vars, temp_list, current_routine
  # book keeper on which subprogram we are on
  current_routine = statement.ParentRoutine.Name
  # function selector as a function

  if statement.Type == VC_STATEMENT_CALL:
    call_dict = write_call(statement)
    if nested == False:
      temp_list.append(call_dict)
    else:
      nested.append(call_dict)
      
  elif statement.Type == VC_STATEMENT_COMMENT:
    comment_dict = write_comment(statement)
    if nested == False:
      temp_list.append(comment_dict)
    else:
      nested.append(comment_dict)

  elif statement.Type == VC_STATEMENT_DELAY:
    # called WAIT in KR2
    delay_dict = write_delay(statement)
    if nested == False:
      temp_list.append(delay_dict)
    else:
      nested.append(delay_dict)

  elif statement.Type == VC_STATEMENT_HALT:
    print "Halt is not supported, skipping."

  elif statement.Type == VC_STATEMENT_RETURN:
    return_dict = write_return(statement)
    if nested == False:
      temp_list.append(return_dict)
    else:
      nested.append(return_dict)

  elif statement.Type == VC_STATEMENT_BREAK:
    break_dict = write_break(statement)
    if nested == False:
      temp_list.append(break_dict)
    else:
      nested.append(break_dict)

  elif statement.Type == VC_STATEMENT_CONTINUE:
    cont_dict = write_continue(statement)
    if nested == False:
      temp_list.append(cont_dict)
    else:
      nested.append(cont_dict)

  elif statement.Type == VC_STATEMENT_LINMOTION:
    move_dict = write_move(statement)
    if nested == False:
      temp_list.append(move_dict)
    else:
      nested.append(move_dict)

  elif statement.Type == VC_STATEMENT_PTPMOTION:
    move_dict = write_move(statement)
    if nested == False:
      temp_list.append(move_dict)
    else:
      nested.append(move_dict)

  elif statement.Type == VC_STATEMENT_SETBIN:
    print "Setting binary output is not supported, skipping."

  elif statement.Type == VC_STATEMENT_WAITBIN:
    print "Wait for binary output is not supported, skipping."

  elif statement.Type == VC_STATEMENT_WHILE:
    nested_while_list = []
    while_dict = write_while(statement, nested_while_list)
    if nested == False:
      temp_list.append(while_dict)
    else:
      nested.append(while_dict)

  elif statement.Type == VC_STATEMENT_IF:
    list_of_dicts = write_if(statement)
    if nested == False:
      for dictionary in list_of_dicts:
        temp_list.append(dictionary)
    else:
      for dictionary in list_of_dicts:
        nested.append(dictionary)

  elif statement.Type == VC_STATEMENT_PATH:
    print "Path statement is not supported, skipping."

  elif statement.Type == VC_STATEMENT_SETPROPERTY:
    set_dict = write_assign(statement)
    if nested == False:
      temp_list.append(set_dict)
    else:
      nested.append(set_dict)
#-------------------------------------------------------------------------------
def write_header(tree, vars_, structure_list):
  global temp_list
  # May cause inaccuracies of due to float type
  # Epoch is 1.1.1970  
  time_since_epoch = float(time.time())       
  epoch_in_ms = int(time_since_epoch*1000)    
  seq_dict = OrderedDict([
    ("classId", 1001),
    ("suppressed", False),
    ("breakpoint", False),
    ("tags",["SEQUENCE_PLACEHOLDER","PROGRAM","STRUCTURAL"]),
    ("arguments", OrderedDict([
      ("label", "Sequence 1"),
      ("include", ""),
      ("realtime", True),
      ("event", "kr2_signal::Alarm"),
      ("execType", 0)
    ])),
    ("structure", structure_list)
  ])

  my_dict = OrderedDict([
    ("version", "0.9.0"),
    ("stackLabel", "postProcessed from VC"),
    ("creationTimeStamp", epoch_in_ms),
    ("stackTrees", tree),
    ("stackVariables", vars_)
  ])
  # Can be appended earlier or later, does not matter due to reference updating
  tree.append(seq_dict)
  return my_dict
#-------------------------------------------------------------------------------
def write_program_body(routine, name, filename, header = False ):
  global master_dict, list_tree, list_vars, temp_list
  # Get the header
  if header == False:
    master_dict = write_header(list_tree, list_vars, temp_list)
  # Loop through all the statements and update the master_dict
  for statement in routine.Statements:
    write_statement(statement)
  # Write the whole dictionary in one go
  with open(filename, "w") as Kassow_PP:
    json.dump(master_dict, Kassow_PP, indent = 4)
  return True
#-------------------------------------------------------------------------------
def postProcess(app,program,uri): # DO NOT CHANGE THIS NAME
  # Entry point, writing to file is impossible in this function, it must be done on side function.
  global motiontarget, master_dict, list_tree, list_vars
  head, tail = os.path.split(uri)
  mainName = tail[:len(tail)-4]
  motiontarget = program.Executor.Controller.createTarget()
  filenamelist = []
  # Write the routine
  filenamelist.append(uri)
  if not write_program_body(program.MainRoutine, mainName, uri):
    return False, filenamelist
  return True, filenamelist