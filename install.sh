#!/bin/bash
virtualenv -p python3.7 ./python/venv --system-site-packages
source ./python/venv/bin/activate && pip install -r tools/installers/requirements.txt
