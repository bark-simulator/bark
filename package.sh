#!/bin/bash

pkg_name='pip_package'
pkg_version=0.0.1
workspace_name='bark_project'

# activate virtual environment
source ./python/venv/bin/activate 

echo "Building package"
#bazel run //bark:$pkg_name
build_dir=bazel-bin/bark/$pkg_name.runfiles

echo "Copying setup.py to project pirectory at $build_dir/$workspace_name"
#copy setup.py file to build directory
cp setup.py $build_dir/$workspace_name

echo "Copying README.md to project directory at $build_dir/$workspace_name"
#copy READEME.md file to build directory
cp README.md $build_dir/$workspace_name

#copy LICENSE file to build directory
cp LICENSE $build_dir/$workspace_name

#read bazel generated manifest to generate manifest for pip package
bazel_manifest=$build_dir/MANIFEST
pip_manifest=$build_dir/$workspace_name/MANIFEST.in


echo "Copying manifest from $bazel_manifest to $pip_manifest"
#remove existing manifest
rm $pip_manifest
while IFS=' ' read -r line delim; do

# python files are handled by bazel
if [[ ${line} != *".py"* ]];then
    echo "include $line" >> $pip_manifest
fi
done <$bazel_manifest


echo "Moving to build directory"
#move to the directory
cd $build_dir/$workspace_name
python3 setup.py clean
python3 setup.py sdist bdist_wheel

