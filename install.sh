#!/bin/bash
virtualenv -p python3.7 ./bark/python_wrapper/venv --system-site-packages
source ./bark/python_wrapper/venv/bin/activate && pip install -r tools/installers/requirements.txt
