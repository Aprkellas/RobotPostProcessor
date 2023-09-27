-------------------------------------------------------------------------------
  # Post-processor add-on for Visual Components 4.X #
-------------------------------------------------------------------------------

  # GENERAL INFO #
  
  -Copy add-on folder to "Documents\Visual Components\4.X\My Commands" folder.
  
  -Add-on creates "Post Process" button on Program tab.
  
  -Add-on tries to select correct post-processor based on robot model.
   For unrecognized brands user can select post-processor.
  
  -Some PPs have settings that user can change on action panel.

-------------------------------------------------------------------------------

  # IRL - VC EXAMPLE FORMAT #
  
  -Produces one .ls file per routine.
  
  -Format is imagenary robot language and only serves as an example.

-------------------------------------------------------------------------------

  # KUKA - KRL #
  
  -Produces one .src and one .dat file.
  
  -Base/tool frame definitions are written at the beginning of main routine.
   They can be commented out from settings.

-------------------------------------------------------------------------------

  # KUKA SUNRISE - JAVA #
  
  -Produces one .java and one RoboticsAPI.data.xml file.
  
  -Base/tool frame definitions as well as position frames are written into
   RoboticsAPI.data.xml.
  
  -.java file contains the program itself.
  
  -Simulation IOs should be mapped to real IOs using wrapper functions
   GetDO and GetDI in .java file so that for given simulation IO port those 
   functions should return desired Output or Input object.

-------------------------------------------------------------------------------

  # ABB - RAPID #
  
  -Produces one .mod file. If you have IO signal statements in the program also
   produces EIO.cfg file that contains definitions for used signals.
  
  -Base/tool frame definitions are written as global variables.

-------------------------------------------------------------------------------

  # FANUC - LS #
  
  -Produces one .ls file per routine.
  
  -Produces SET_FRAMES.ls that synchronizes tool and base frames. This 
   subroutine is called from the beginning of main routine.
  
  -Use only tools 1-10 and bases NULL and 1-9.
  
  -Relative DefineBase/Tool need matrix multiplication function which you
   can find in "Vision support tools" option package. Also set system
   variable $KAREL_ENB to 1.

-------------------------------------------------------------------------------

  # YASKAWA - INFORM #
  
  -Produces one .JBI file per routine and also TOOL.CND and UFRAME.CND.
  
  -Base/tool frame definitions are written in TOOL.CND and UFRAME.CND.
   User should merge them manually to existing tools and uframes.
   
  -If you output position as pulses ('UsePulses') you should set ALL.PRM or
   RC.PRM file on the properties. These parameters are used to identify the
   pulses per axis revolution ratios which are saved into the model as
   'PulsesPerRev' and 'PulsesZeros' properties. Once saved these properties
   can be used for ratios and parameter file is not necessary to be given 
   again. In case robot is not first if RC parameters you can create 'RCXG' 
   string property to robot and give it matching value, e.g. 'RC2G'.

-------------------------------------------------------------------------------

  # UNIVERSAL ROBOTS - URP / SCRIPT #
  
  -Produces .URP for every VC routine. If "PP Type" is set to "Script" produces
   one .script file instead that can be impoted into a script command.
  
  -URP is supported only for e-Series UR models.
  
  -Note that in URP you can only have subroutine calls from main routine.
  
  -Tool definition:
    -The TCP name used in VC should be defined in Polyscope under INSTALLATION > TCP 
    -VC Tool > X, Y, Z    > TCP Position X, Y, Z 
    -VC Tool > Rx, Ry, Rz > TCP Orientation Unit (RPY in degree)
    -*NULL* TCP in VC will create 'Tool0' in MoveL and MoveJ. DEFINE THE TCP NAME
       as 'Tool0' IN POLYSCOPE UNDER INSTALLATION > TCP with position and orientation as '0'
  
  -SEQUENCE (routines):
    -During PostProcess, each subroutine in VC is created as *.urp file
    -Loading the *.urp(Main) file in polyscope, Call statement and SubProg with the 
      (sequence)routine name is created
    -User should manually navigate to the *.urp(sequence) file from their file system 
      and select it. Also assign it to Call statement.
  
  -Supported VC statements:
    -PTP/LIN/Path, Wait Input, Set output, Halt, Comment, Call, Assign, If, While
  
  -SETTINGS:
    
    -Use Active TCP (URP): sets if default TCP is used or if TCP is specified in 
     statement.
    
    -Hide Sub Program Tree (URP): Hide/show subroutine tree in URP.
    
    -Use set_tcp (Script): Use set_tcp function to set active tool pose. Use this 
     if there are many tool frames in your VC program.
    
    -Use acceleration values (Script): Use optional parameter for acceleration on 
     motions.
    
    -movel as joint values (Script): Post movel as joint values instead of cartesian 
     pose.
    
    -Path motion type (URP/Script): Post path statement as movel or movep.
    
    -Input mapping (URP/Script): Select IO type where wait input statements are
     mapped.
    
    -Output mapping (URP/Script): Select IO type where set ouput statements are
     mapped.

-------------------------------------------------------------------------------

  # DOOSAN - DRL #
  
  -Produces one .drl file that can be imported as son Doosan robot.
  
  -Produce base definitions as globals. Tool definitions are produces only as 
   comments and user should sync them manually.

-------------------------------------------------------------------------------

  # EPSON - SPEL+ #
  
  -Produces one .prg file and one .pts file.
  
  -IMPORTANT! Point file doesn't solve the header checksum. So user has to
   load point file on RC+ 7.0, make some trivial change that can be reverted.
   Change is needed so user can save file again on RC+ 7.0 and then checksum
   on header is set correctly. Without this operation point file may not work.
  
  -Base/tool frame definitions are written at the beginning of main routine.

-------------------------------------------------------------------------------

  # MITSUBISHI - MELFA V #
  
  -Produces one .prg file per routine.
  
  -IMPORTANT! Point file doesn't solve the header checksum. So user has to
   load point file on RC+ 7.0, make some trivial change that can be reverted.
   Change is needed so user can save file again on RC+ 7.0 and then checksum
   on header is set correctly. Without this operation point file may not work.
  
  -Doesn't produce base/tool definitions. User has to sync them manually.

-------------------------------------------------------------------------------

  # KAWASAKI- AS #
  
  - Produces one .pg file including main and subroutines.
  
  - Base/Tool definition are written at the beginning of main routine.
  
  - Conditional statements are not supported. User has to specify manually.

-------------------------------------------------------------------------------

  # COMAU - PDL #
  
  - Produces one .pdl file including main and subroutines and one .lsv file containing global variables such as positions.
  
  - Base/Tool definition are written at the beginning of the program before actual main routine is called.

-------------------------------------------------------------------------------

  # HYUNDAI - JOB #
  
  - Produces one .JOB file including main and subroutines.
  
  - Base (User) and tool frames can be printed into seprate info file (Create User/Tool frame text file).
    By default they do need to be synced manually to real controller. There is an option to include
    frame assignments at the start of main routine (Assign Tool/User frames in job). And there is an
    option to convert all positions to Robot frame in which case user frames are not used.
  
  - At the moment PP doesn't support external axes.

-------------------------------------------------------------------------------