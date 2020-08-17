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
  "name": "Python: Current File",
  "type": "python",
  "request": "launch",
  "program": "${file}",
  "console": "integratedTerminal",
  "env": {
    "PYTHONPATH": "${workspaceFolder}/bazel-bin/examples/{path-to-executable-file}.runfiles/__main__/python:${workspaceFolder}/bazel-bin/examples/{path-to-executable-file}.runfiles/__main__"
  }
}
```

Make sure to be in the main executable file when launching the debugger (F5).


## Debugging C++ and Python

Here, both debuggers need to be run in parallel.
First, we need to build the "bark.so" in the debug mode by running `bazel build //... --compilation_mode=dbg`.
You can also check if the "bark.so" contains debug symbols by running `readelf --debug-dump=decodedline bark.so`.
Then, add the following launch configuration and adapt the path in the `.vscode/launch.json`:

```
{
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
```

Debugging process:
1. Add a breakpoint in the python file you want to debug, somewhere before an interesting code section, run the launch configuration "Python: Current File" (see before) and wait until the breakpoint is reached.
2. Run the "(gdb) Attach" launch configuration, select the python interpreter whose path contains "/.vscode/". You will be promted to enter your user password.
3. Set breakpoints in the C++ Files
4. The python debugger is currently stopped at a break point. Switch back from the debugger "(gdb) Attach" to the other debugger "Python: Current File" and press F5 (Continue). Now, vscode automatically jumps between the two debuggers between python and c++ code.


## Memory Checking

Use Valgrind to profile the code in order to find memory leaks.
1. Build the target with debug symbols, i.e. `bazel test //bark/world/tests:py_map_interface_tests --compilation_mode=dbg`
2. Profile via `valgrind --track-origins=yes --keep-stacktraces=alloc-and-free --leak-check=full ./bazel-bin/{path-to-executable-file}`.