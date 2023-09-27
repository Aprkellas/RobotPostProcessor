from vcApplication import *

def OnStart():
  cmduri = getApplicationPath() + 'PostProcessLauncher.py'
  cmd = loadCommand('PostProcessLauncher',cmduri)
  addMenuItem('VcTabTeach/Export', "Post Process Custom", -1, "PostProcessLauncher")
