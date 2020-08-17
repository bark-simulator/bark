How to Install BARK
================================

This section describes the prerequisites and installation-steps of BARK.


## Prerequisites

* Bazel > 3(requires Java)
* Python3.7 (`sudo apt-get install python3.7 python3.7-dev python3.7-tk`)
* Virtual Env (`pip3 install virtualenv==16.7.8`) (note that the newest version does not seem to link the Python.h)
* gcc7 (needs to be set as the default compiler)
* Visual Studio Code

## Install using pip
* pip3 install bark-simulator

## Setup on Linux

Optional: We recommend to use Anaconda. This way, you can create a clean python environment. After installation, create a conda env `conda create --name bark_python_env python=3.7` and follow with activating it: `conda activate bark_python_env`. You can now proceed with the following:

1. Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
2. Run `bash install.sh`: creates a virtual environment (located in python/venv) and installs all python packages
2. Run `source dev_into.sh`: activates the virtual environment (make sure to run this before bazel)
3. Use `bazel test //...` to validate that BARK is working.
4. Finally, try one of the examples provided in BARK by running `bazel run //examples:merging`.


## Setup on MacOS

1. Install pyenv: `brew install pyenv`.
2. Install a newer version of tcl-tk: `brew upgrade tcl-tk`.
3. Run `pyenv install python3.7-dev`. If you run into trouble with TKInter have a look [here](https://stackoverflow.com/questions/60469202/unable-to-install-tkinter-with-pyenv-pythons-on-macos).
4. Set this as your global Python version: `pyenv global 3.7-dev`.
5. Also add this Python version to your `~/.zshrc` by adding `eval "$(pyenv init -)"`.
6. Install an older version of the virtualenv package by running: `pip install virtualenv==16.7.8`
7. In order to set TKAgg as backend have a look [here](https://stackoverflow.com/questions/21784641/installation-issue-with-matplotlib-python).
8. Modify the file `install.sh` by using `virtualenv -p python ./python/venv` instead as python is now the pyenv version.
9. Now you can follow the same steps as when using Linux.


## Build Pip package
* Install twine using python3 -m pip install --user --upgrade twine
* Run script bash package.sh to build code, package and upload to pypi

## Frequently Asked Questions (FAQs)

### Python.h not found

Make sure that there is a 'Python.h' file in the `python/venv` folder.

### Feel free to add your questions here or asks us directly by submitting an issue!
