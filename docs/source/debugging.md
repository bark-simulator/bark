Debugging
================================

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