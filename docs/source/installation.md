Installation
================================
This section describes the prerequisites and installation-steps of BARK. In order to be system independent, we decided to use a fully sandboxed build system ([Bazel](https://bazel.build/)).

All features are currently supported under Linux/MacOS. For Windows, Bark only works for the C++ modules due to missing support of combining Bazel and [Pybind11](https://pybind11.readthedocs.io/en/stable/intro.html). 


## Prerequisites
* Bazel (requires Java)
* Python3
* virtualenv (Virtual environment)

### Java (necessity for Bazel) on Mac: 
We recomment to install JDK it manually from the OpenJDK download page
* Download OpenJDK for Mac OSX from http://jdk.java.net/
* Unarchive the OpenJDK tar, and place the resulting folder (i.e. jdk-12.jdk) into your /Library/Java/JavaVirtualMachines/ folder since this is the standard and expected location of JDK installs. You can also install anywhere you want in reality.

### Bazel on Mac:
See https://docs.bazel.build/versions/master/install-os-x.html, we recommend installing using the binary installer


## Linux/MacOS
### Setup
1. [Clone the repository](https://git.fortiss.org/bark-simulator/bark), change to base repository directory 
2. `bash install.sh`: this will create a virtual python environment (located in python/venv) and install all necessary python dependencies.
2. `source dev_into.sh`: this will activate the virtual environment (keep this in mind for the future: each time you use Bazel, even beyond this installation, be sure to have run this command beforehand)
3. `bazel build //...`: this will build the whole library and test cases (specific test cases or specify individual modules can be built with e.g `bazel build //modules/world/tests:world_test`, see the documentation of Bazel for more insights).
4. `bazel test //...`: this will run all specified tests (individual tests can be executed using, e.g. for the C++ tests `//modules/world/tests:world_test` and for the python test cases `bazel test //python:name_of_module`, e.g `bazel test //python:importer_test`).
5. If you experience problems with Pybind, you might not have installed the header files and static libraries for python. You can install them via `sudo apt-get install python3-dev`
6. If the viewer stays empty, you probably have to install tkinter using `sudo apt-get install python3-tk`.


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

## Generate Documentation
A local version of the Bark documentation can be generated using
```
cd docs
make html
```
This will create the documentation which can be accessed at `/docs/build/html/index.html`.

