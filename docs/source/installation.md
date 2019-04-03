Installation
================================
This section describes the prerequisites and installation-steps of BARK. In order to be system independent, we decided to use a fully sandboxed build system ([Bazel](https://bazel.build/)).

All features are currently supported under Linux/MacOS. For Windows, Bark only works for the C++ modules due to missing support of combining Bazel and [Pybind11](https://pybind11.readthedocs.io/en/stable/intro.html). 


## Prerequisites
* Bazel (requires Java) (we have experienced problems with newer bazel versions (v0.20) and will investigate the problem, [v0.15](https://github.com/bazelbuild/bazel/releases/tag/0.15.0) works for sure)
* Python3
* virtualenv (Virtual environment)


## Linux/MacOS
### Setup
1. [Clone the repository](https://git.fortiss.org/bark-simulator/bark), change to base repository directory 
2. `bash install.sh`: this will create a virtual python environment (located in python/venv) and install all necessary python dependencies.
2. `source dev_into.sh`: this will activate the virtual environment (keep this in mind for the future: each time you use Bazel, even beyond this installation, be sure to have run this command beforehand)
3. `bazel build //...`: this will build the whole library and test cases (specific test cases or specify individual modules can be built with e.g `bazel build //modules/world/tests:world_test`, see the documentation of Bazel for more insights).
4. `bazel test //...`: this will run all specified tests (individual tests can be executed using, e.g. for the C++ tests `//modules/world/tests:world_test` and for the python test cases `bazel test //python:name_of_module`, e.g `bazel test //python:importer_test`).


### IDE
We recommend Visual Studio Code as IDE under Linux/MacOS and also provide launch and task files to work with Bark in VSCode.



## Windows
**CURRENTLY NOT FULLY SUPPORTED! Please use Linux or MacOS!**

### Setup
1. Install Bazel and Prerequisites for windows: https://docs.bazel.build/versions/master/install-windows.html
2. Add JAVA_HOME!! Even if we do not want to build Java code
3. Run Bazel: https://docs.bazel.build/versions/master/windows.html (BAZEL_VS and BAZEL_VC)
 set BAZEL_VS=C:\Program Files (x86)\Microsoft Visual Studio 14.0
 set BAZEL_VC=C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC
4. cd into bark directory and use bazel
5. Check if setup worked by //tests/geometry:geometry_tests, bazel run //tests/geometry:geometry_tests

### IDE
As pybind11 depends on the vscompiler, Visual Studio is the recommended IDE under Windows. An experimental setup for the C++ part of Bark is achieved with
1. Install [lavender](https://github.com/tmandry/lavender) to generate Visual Studio Projects for Bazel.
2. Then run eg. python ..\lavender\generate.py //modules/geometry //tests/geometry:geometry_tests to generate a Visual Studio project.

The python interfacing under Windows is currently untested.

## Debugging with VSCode

### Debugging C++ Code
Use build bazel in debug mode
```
bazel build //... --compilation_mode=dbg 
```
Add the debug configuration in `.vscode/launch.json` in Visual Studio Code and then launch the debugger (F5 in current file)
```
  {
      "name": "(gdb) Launch",
      "type": "cppdbg",

      "request": "launch",
      "program": "${workspaceFolder}/bazel-bin/{path-to-executable-file}",
      "args": [],
      "stopAtEntry": true,
      "cwd": "${workspaceFolder}",
      "environment": [],
      "externalConsole": true,
      "MIMode": "gdb",
      "setupCommands": [
          {
              "description": "Enable pretty-printing for gdb",
              "text": "-enable-pretty-printing",
              "ignoreFailures": true
          }
      ]
  }
```
Set breakpoints and debug


### Debug Python Code
```
{
    "name": "Python: Current File",
    "type": "python",
    "request": "launch",
    "program": "${file}",
    "console": "integratedTerminal",
    "env": {
        "PYTHONPATH": "${workspaceFolder}/bazel-bin/examples/od8_const_vel_one_agent.runfiles/__main__/python:${workspaceFolder}/bazel-bin/examples/od8_const_vel_one_agent.runfiles/__main__"
  }
```
The path `examples/od8_const_vel_one_agent.runfiles` needs to be changed if another python file should be debugged. Furthermore, make sure to be in the main executable file when launching the debugger (F5).

### Debug from Python into C++ Extension

We run two Debuggers in parallel. First, check if the shared object "bark.so" contains debug symbols with e.g. `readelf --debug-dump=decodedline bark.so` which should print out line numbers and source file name for each instruction. If not use `bazel build //path_to_python_binary --compilation_mode=dbg` to build all dependencies with debug symbols.  Then, add the following launch configuration and adapt the path in "additionalSOLibSearchPath":
```
{
            "name": "(gdb) Attach",
            "type": "cppdbg",
            "request": "attach",
            "program": "${workspaceFolder}/python/venv/bin/python3",
            "cwd" : "${workspaceFolder}",
            "additionalSOLibSearchPath":"${workspaceFolder}/bazel-bin/examples/od8_const_vel_two_agent.runfiles/__main__/python",
            "processId": "${command:pickProcess}",
            "MIMode": "gdb",
            "sourceFileMap" : {"/proc/self/cwd/": "${workspaceFolder}"},
            }
```
For debugging;
1. Add a breakpoint in the python file you want to debug, somewhere before an interesting code section, run the launch configuration "Python: Current File" (see before) and wait until the breakpoint is reached.
2. Run the "(gdb) Attach" launch configuration, select the python interpreter whose path contains "/.vscode/"
3. Set breakpoints in the C++ Files
4. Now, with the F5 vscode automatically jumps between the two debuggers between python and c++ code. 


## Generate Documentation
A local version of the Bark documentation can be generated using
```
cd docs
make html
```
This will create the documentation which can be accessed at `/docs/build/html/index.html`.

