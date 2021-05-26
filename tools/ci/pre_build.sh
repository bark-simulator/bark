#!/bin/bash

yum install -y devtoolset-9 rh-python36 libtool

source scl_source enable devtoolset-9

curl -o libbacktrace-master.zip https://codeload.github.com/ianlancetaylor/libbacktrace/zip/master
unzip ./libbacktrace-master.zip
cd ./libbacktrace-master
./configure
make
make install
cd ../

python setup.py clean
python setup.py build_ext bdist_wheel