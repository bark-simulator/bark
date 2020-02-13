#!/bin/bash
# we need root access for that, should thus only be called in docker file
apt-get update
apt-get install -y python3.7-dev python3.7-tk #python3.7-venv

# Do not update virtualenv to new version, if not guaranteed that python headers are symlinked
pip3 install virtualenv==16.7.2 

pip3 install --upgrade pip
