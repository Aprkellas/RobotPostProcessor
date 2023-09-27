from vcApplication import *

def OnStart():
  cmduri = getApplicationPath() + 'PostProcessLauncherNoEA.py'
  cmd = loadCommand('PostProcessLauncherNoEA',cmduri)
  addMenuItem('VcTabTeach/Export', "Post Process (NO EA)", -1, "PostProcessLauncherNoEA")
