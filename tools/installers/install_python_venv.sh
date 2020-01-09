#!/bin/bash
# we need root access for that, should thus only be called in docker file
apt-get update
apt-get install -y python3-pip python3-dev python3-tk
