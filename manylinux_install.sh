#!/bin/bash
#docker build -t mansoorcheema/bazel-manylinux -f tools/ManyLinuxDockerfile .
#docker run -it -v $(pwd):/io mansoorcheema/bazel-manylinux:latest /bin/bash
cd /bark
virtualenv --python=/opt/python/cp37-cp37m/bin/python ./bark/python_wrapper/venv
source ./bark/python_wrapper/venv/bin/activate && pip install -r tools/installers/requirements.txt
bash package.sh manylinux
