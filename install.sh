#!/bin/bash
virtualenv -p python3.7 ./python/venv
source ./python/venv/bin/activate && pip install -r tools/installers/requirements.txt
