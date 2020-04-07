import os
cwd = os.getcwd()
runfiles_dir = cwd.split("run.runfiles")[0]
bark_root = os.path.join(runfiles_dir, "run.runfiles/bark_project")

try:
    os.chdir(bark_root)
    print("Changing to bark root {}".format(bark_root))
except:
    pass