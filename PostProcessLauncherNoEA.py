#Launcher version 1.15

from vcCommand import *
import sys, os, importlib, re

app = getApplication()
cmd = getCommand()
other_cmd = app.findCommand('interactiveTranslateTCP')
TRANSLATORS_DIR_NAME = 'Translators'
cmd_uri = getCommandPath() # this command's uri
cmd_uri = cmd_uri[8:].decode('utf8') # remove "file:///" header
cmdfolder, cmdfilename = os.path.split(cmd_uri)
sys.path.append(cmdfolder)


def OnStart():
  #Entry point  
  global post_processors, file_filters, program

  #Basic handles
  program = getProgram()
  if not program:
    app.messageBox("No program selected, aborting.","Warning",VC_MESSAGE_TYPE_WARNING,VC_MESSAGE_BUTTONS_OK)
    return    
  controller = program.Executor.Controller
  
  #Get translators
  file_filters = {}
  post_processors = {}
  translators_folder = os.path.join(cmdfolder, TRANSLATORS_DIR_NAME)

  for root, folder, filenames in os.walk( translators_folder ):
    for filename in filenames:
      filename = str(filename)
      filebasename, extension = os.path.splitext(filename)

      if extension != '.py':
        continue # skip non python files

      if '__init__' in filename:
        continue # skip the __init__.py file

      post_uri = os.path.join(cmdfolder, filename) 
      tokens = filebasename.split('_')
      
      if len(tokens) < 2:
        #Post processor name example:  test.py => "test Robot Program file (*.test)|*.test"
        manufacturer, filetype = tokens[0], tokens[0]
      else:
        #Post processor name example:  ACME_obd.py => "ACME Robot Program file (*.obd)|*.obd"
        manufacturer, filetype = tokens[-2], tokens[-1]

      #Import postprocessor module (and reload to allow easy editing)
      module_name = TRANSLATORS_DIR_NAME + '.' + filebasename
      post_processor_module = importlib.import_module( module_name )
      post_processor_module = reload(post_processor_module)
      #Dictionary of post processors per manufacturer
      post_processors[manufacturer] = post_processor_module
      #File filter for this post processor
      file_filters[manufacturer] = "%s Robot Program file (*.%s)|*.%s" % (manufacturer, filetype, filetype)

  if not post_processors:
    print 'No post processors found! Please add a post processor file to "%s"' % (cmdfolder)
    return
    
    
  #Try to figure out manufacturer and correct post-processor
  manufacturer = getManufacturer(controller)
  createProperties(manufacturer)
  
  if manufacturer and manufacturer in file_filters:
    #Matching post processor for this robot was found, allow using only that one
    post_processor = post_processors[manufacturer]
    #Check if pp has properties that need to be shown in action panel
    if 'getProperties' in dir(post_processor):
      #There are props, open action panel
      executeInActionPanel()
      return
    else:
      #No props, launch staright to save file dialog
      if not selectOutput():
        #Aborted
        return
      callPostProcessor()
  else:
    #Matching processor not found or not known, user selects pp in action panel
    executeInActionPanel()
    return
    
    
def getProgram():
  #Get active program
  teachcontext = app.TeachContext
  if teachcontext.ActiveRobot:
    executors = teachcontext.ActiveRobot.findBehavioursByType(VC_ROBOTEXECUTOR)
    if executors:
      return executors[0].Program
  return None


def getManufacturer(controller):
  #Try to figure out robot manufacturer based on the model
  comp = controller.Component
  keys = post_processors.keys()
  manufacturer = ''
  model = controller.Component.Name
  
  #Try model stamp
  prop = comp.getProperty('RobotModelID')
  if prop:
    words = prop.Value.replace(' ','').split('|')
    if words and words[0] in keys:
      manufacturer = words[0]
    if words and len(words) >= 2:
      model = words[1]
  
  
  if not manufacturer:
    #Try from BOM description
    bom = comp.BOMdescription.replace(' ', '')
    for key in keys:
      if key in bom:
        return key
  
  if not manufacturer:
    #Try controller names for big brands
    ctr_name = controller.Name
    if ctr_name in ['IRC5']:
      return 'ABB'
    elif ctr_name in ['KRC2', 'KRC3', 'KRC4']:
      return 'KUKA'
    elif ctr_name in ['R30iA', 'R-30iA Mate', 'R-30iB']:
      return 'Fanuc'
    elif ctr_name in ['DX100', 'DX200']:
      return 'Yaskawa'
    elif ctr_name in ['URControl', 'CB3']:
      return 'UniversalRobots'
  
  #Alternative languages
  if manufacturer == 'KUKA' and 'LBR' in model:
    manufacturer += '-Sunrise'
  
  
  return manufacturer


def postProcessorChanged(arg = None):
  #Rebuild action panel with new properties
  createProperties('')
  executeInActionPanel()


def selectOutput(arg = None):
  #Open save file dialog with correct file filter
  manufacturer = prop_sel_pp.Value
  ok = True
  post_processor = post_processors[manufacturer]
  file_filter = file_filters[manufacturer]
  if 'getFileFilter' in dir(post_processor):
    file_filter = post_processor.getFileFilter()
  
  savecmd = app.findCommand("dialogSaveFile")
  
  uri = savecmd.Param_1
  folder, file = '', ''
  try:
    folder, file_old = uri.split()
  except:
    pass
  post_processor = post_processors[manufacturer]
  if 'getDefaultJobName' in dir(post_processor):
    file = post_processor.getDefaultJobName()
  uri = os.path.join(folder, file)
  
  savecmd.execute(uri,ok,file_filter,'Choose File to save Robot Program file')
  if not savecmd.Param_2:
    print "No file selected, aborting command."
    return False
  uri = savecmd.Param_1
  prop_output.Value = uri[8:]
  
  
  if arg:
    #Called from action panel, open action panel again (savecmd closes it)
    executeInActionPanel()
  else:
    #Called from main, return status
    return True
  
  
def callPostProcessor(arg = None):
  global all_props

  #Call selected post processor
  manufacturer = prop_sel_pp.Value
  fileuri = prop_output.Value
  filebase, filetype = os.path.splitext(fileuri)
  #Call the post processor that matches with the manufacturer chosen by the user
  result = post_processors[manufacturer].postProcess(app, program, fileuri.decode('utf8'))
  if not result or len(result) < 2:
    #PP utilizes action panel and does ending actions inside its own code.
    return
  succesful, created_filenamelist = result
  if succesful:
    removeExternalAxis(fileuri)
    print 'Succesfully saved files:'
    for f in created_filenamelist:
      print '- %s' % f
  else:
    print 'File writing failed'
  #Cleanups
  program.Executor.Controller.clearTargets()
  
  if arg:
    #Called from action panel, call jog cmd to close action panel
    other_cmd.execute()
  
  # Clear all_props handle to clear property on changed event handlers
  all_props = None


def propChanged(*args):
  prop = cmd.getProperty('Post Processor')
  if not prop:
    return
  manufacturer = prop.Value
  if not manufacturer in post_processors:
    return
  post_processor = post_processors[manufacturer]
  if 'updateActionPanel' in dir(post_processor):
    post_processor.updateActionPanel()


def createProperties(lock_manufacturer = ''):
  #Confirm properties to be shown in action panel
  global post_processors, file_filters
  global prop_sel_pp, prop_output, prop_btn_output, prop_btn_pp
  global all_props
  
  manufacturer = 'IRL'
  filetype = '.irl'
  prop = cmd.getProperty('Post Processor')
  if prop:
    manufacturer = prop.Value
  if lock_manufacturer:
    manufacturer = lock_manufacturer
  post_processor = post_processors[manufacturer]
  manufacturer_properties = []
  if 'getProperties' in dir(post_processor):
    manufacturer_properties = post_processor.getProperties()
  filter = file_filters[manufacturer]
  if 'getFileFilter' in dir(post_processor):
    filter = post_processor.getFileFilter()
  filetype = filter[filter.rfind('.'):]
  uri = r'C:\temp%s' % (filetype)
  if 'getDefaultJobName' in dir(post_processor):
    uri = r'C:\%s' % (post_processor.getDefaultJobName())
  prop = cmd.getProperty('Output')
  if prop:
    uri = prop.Value
  properties = []
  properties.append((VC_STRING, 'Post Processor', manufacturer, VC_PROPERTY_STEP, post_processors.keys(), 0, 0))
  properties.append((VC_STRING, 'Output', uri, None, None, 0, 0))
  properties.append((VC_BUTTON, 'Select Output', None, None, None, 0, 0))
  properties.extend(manufacturer_properties)
  properties.append((VC_BUTTON, 'Post Process', None, None, None, 0, 0))
  
  
  #Check if properties already match
  match = len(properties) == (len(cmd.Properties) - 1) #cmd.Properties always has CallingCommandName as extra
  if match:
    for p in properties:
      if not cmd.getProperty(p[1]):
        match = False
        break
  
  #Create properties if there's a mismatch
  if not match:
    for p in cmd.Properties:
      cmd.deleteProperty(p)
    for p in properties:
      prop_type = p[0]
      prop_name = p[1]
      prop_def_value = p[2]
      prop_constraints = p[3]
      prop_step_values = p[4]
      prop_min_value = p[5]
      prop_max_value = p[6]
      if not prop_constraints:
        prop = cmd.createProperty(prop_type, prop_name)
      else:
        prop = cmd.createProperty(prop_type, prop_name, prop_constraints)
        if prop_constraints == VC_PROPERTY_STEP:
          prop.StepValues = prop_step_values
        elif prop_constraints == VC_PROPERTY_LIMIT:
          prop.MinValue = prop_min_value
          prop.MaxValue = prop_max_value
      if prop_def_value:
        prop.Value = prop_def_value
      if prop_name == 'Post Processor':
        prop_sel_pp = prop
        prop.OnChanged = postProcessorChanged
      elif prop_name == 'Output':
        prop_output = prop
        prop.WritableWhenConnected = False
        prop.WritableWhenDisconnected = False
        prop.WritableWhenSimulating = False
      elif prop_name == 'Select Output':
        prop_btn_output = prop
        prop.OnChanged = selectOutput
      elif prop_name == 'Post Process':
        prop_btn_pp = prop
        prop.OnChanged = callPostProcessor
  
  
  #Confirm some values
  prop_sel_pp.Value = manufacturer
  uri = prop_output.Value
  uri = uri[0:uri.rfind('.')] + filetype
  prop_output.Value = uri
  prop_sel_pp.WritableWhenConnected = lock_manufacturer == ''
  prop_sel_pp.WritableWhenDisconnected = lock_manufacturer == ''
  prop_sel_pp.WritableWhenSimulating = lock_manufacturer == ''
  
  #Take global handle of all properties to allow OnChanged event handlers
  all_props = [x for x in cmd.Properties]
  for prop in all_props:
    prop.OnChanged = propChanged
  propChanged()

def createProperty(type, name, defaultValue, callback):
    prop = cmd.getProperty(name)
    if prop == None:
        prop = cmd.createProperty(type, name)

    if defaultValue:
        prop.Value = defaultValue

    if callback:
        prop.OnChanged = callback

    return prop


def removeExternalAxis(input_file):
  output_dir = os.path.dirname(input_file)
  filename, file_extension = os.path.splitext(os.path.basename(input_file))
  output_file = os.path.join(output_dir, filename + "NOEA" + file_extension)


  with open(input_file, 'r') as infile, open (output_file, 'w+') as outfile:
    previous_line = ""

    for line in infile:
            # Check if the line contains "E1" followed by a value
            match = re.search(r'E1\s*=\s*[\d.]+\s*mm', line)
            if match and previous_line.strip().endswith(','):
                for line in outfile:
                    if line == previous_line:
                        line.replace('')
                previous_line = previous_line.rstrip(',\n')
                outfile.write(previous_line)

            if match:
                # Replace the matched part with an empty string
                line = line.replace(match.group(0), '')

            # Write the modified line to the output file
            outfile.write(line)
            previous_line = line