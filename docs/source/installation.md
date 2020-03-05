How to Install BARK
================================
This section describes the prerequisites and installation-steps of BARK.
We recommend using Ubuntu.


## Prerequisites
* Bazel (requires Java)
* Python3.7 (`sudo apt-get install python3.7 python3.7-dev python3.7-tk` and `pip install virtualenv`)
* gcc7 (needs to be set as the default compiler)
* Visual Studio Code


## Setup on Linux
1. Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
2. Run `bash install.sh`: creates a virtual environment (located in python/venv) and installs all python packages
2. Run `source dev_into.sh`: activates the virtual environment (make sure to run this before bazel)
3. Use`bazel test //...` to validate that BARK is working.
4. Finally, try one of the examples provided in BARK by running `bazel run //examples:od8_const_vel_two_agent`.


## Frequently Asked Questions (FAQs)
#### Python.h not found
Make sure that there is a 'Python.h' file in the `python/venv` folder.

#### Feel free to add your own questions here or asks us directly by submitting an issue!