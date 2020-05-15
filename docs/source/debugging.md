Debugging with VSCode
================================

## Debugging C++ Code

First, you need to build BARK in the debug mode using

```
bazel build //... --compilation_mode=dbg
```

Additionally, you need to modify the `.vscode/launch.json` in Visual Studio Code and then launch the debugger (F5 in the current file).

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

Now, you can set breakpoints and debug the c++ code.


## Debugging Python Code
To debug python code, you need to add the following code in the `.vscode/launch.json`:

```
{
<<<<<<< HEAD
  "name": "Python: Current File",
  "type": "python",
  "request": "launch",
  "program": "${file}",
  "console": "integratedTerminal",
  "env": {
    "PYTHONPATH": "${workspaceFolder}/bazel-bin/examples/{path-to-executable-file}.runfiles/__main__/python:${workspaceFolder}/bazel-bin/examples/{path-to-executable-file}.runfiles/__main__"
=======
    "name": "Python: Current File",
    "type": "python",
    "request": "launch",
    "program": "${file}",
    "console": "integratedTerminal",
    "env": {
        "PYTHONPATH": "${workspaceFolder}/bazel-bin/bark/examplesod8_const_vel_one_agent.runfiles/__main__/python:${workspaceFolder}/bazel-bin/bark/examplesod8_const_vel_one_agent.runfiles/__main__"
>>>>>>> automate bazel build and package build
  }
}
```
<<<<<<< HEAD
=======
The path `bark/examplesod8_const_vel_one_agent.runfiles` needs to be changed if another python file should be debugged. Furthermore, make sure to be in the main executable file when launching the debugger (F5).
>>>>>>> automate bazel build and package build

Make sure to be in the main executable file when launching the debugger (F5).


## Debugging C++ and Python

Here, both debuggers need to be run in parallel.
First, we need to build the "bark.so" in the debug mode by running `bazel build //... --compilation_mode=dbg`.
You can also check if the "bark.so" contains debug symbols by running `readelf --debug-dump=decodedline bark.so`.
Then, add the following launch configuration and adapt the path in the `.vscode/launch.json`:

<<<<<<< HEAD
=======
We run two Debuggers in parallel. First, check if the shared object "core.so" contains debug symbols with e.g. `readelf --debug-dump=decodedline core.so` which should print out line numbers and source file name for each instruction. If not use `bazel build //path_to_python_binary --compilation_mode=dbg` to build all dependencies with debug symbols.  Then, add the following launch configuration and adapt the path in "additionalSOLibSearchPath":
>>>>>>> Package Restructuring
```
{
<<<<<<< HEAD
  "name": "(gdb) Attach",
  "type": "cppdbg",
  "request": "attach",
  "program": "${workspaceFolder}/python/venv/bin/python3",
  "cwd" : "${workspaceFolder}",
  "additionalSOLibSearchPath":"${workspaceFolder}/bazel-bin/examples/{path-to-executable-file}.runfiles/__main__/python",
  "processId": "${command:pickProcess}",
  "MIMode": "gdb",
  "sourceFileMap" : {"/proc/self/cwd/": "${workspaceFolder}"},
}
=======
            "name": "(gdb) Attach",
            "type": "cppdbg",
            "request": "attach",
            "program": "${workspaceFolder}/python/venv/bin/python3",
            "cwd" : "${workspaceFolder}",
            "additionalSOLibSearchPath":"${workspaceFolder}/bazel-bin/bark/examplesod8_const_vel_two_agent.runfiles/__main__/python",
            "processId": "${command:pickProcess}",
            "MIMode": "gdb",
            "sourceFileMap" : {"/proc/self/cwd/": "${workspaceFolder}"},
            }
>>>>>>> automate bazel build and package build
```

Debugging process:
1. Add a breakpoint in the python file you want to debug, somewhere before an interesting code section, run the launch configuration "Python: Current File" (see before) and wait until the breakpoint is reached.
2. Run the "(gdb) Attach" launch configuration, select the python interpreter whose path contains "/.vscode/". You will be promted to enter your user password.
3. Set breakpoints in the C++ Files
4. The python debugger is currently stopped at a break point. Switch back from the debugger "(gdb) Attach" to the other debugger "Python: Current File" and press F5 (Continue). Now, vscode automatically jumps between the two debuggers between python and c++ code.


<<<<<<< HEAD
## Memory Checking
=======
Use Valgrind to profile the code in order to find memory leaks. Valgrind can be installed using apt-get.
1. Build the target with debug symbols, i.e. `bazel test //bark/world/tests:py_map_interface_tests --compilation_mode=dbg`
2. Profile via `valgrind --track-origins=yes --keep-stacktraces=alloc-and-free --leak-check=full ./bazel-bin/modules/world/tests/map_interface_test`. There are a lot of options, check out Valgrind's documentation!
>>>>>>> Package Restructuring

Use Valgrind to profile the code in order to find memory leaks.
1. Build the target with debug symbols, i.e. `bazel test //modules/world/tests:py_map_interface_tests --compilation_mode=dbg`
2. Profile via `valgrind --track-origins=yes --keep-stacktraces=alloc-and-free --leak-check=full ./bazel-bin/{path-to-executable-file}`.