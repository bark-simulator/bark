#!/bin/bash

pkg_name='pip_package'
#pkg_version=0.0.1
workspace_name='bark_project'

# activate virtual environment
source ./bark/python_wrapper/venv/bin/activate 

echo "Building package"
bazel run //bark:$pkg_name

if [ $? -eq 0 ]; then
    echo "Build Suceeded"
else
    echo "Build Fail"
    exit 0
fi

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


#echo "Copying tests  at $build_dir/$workspace_name"
#mkdir -p $build_dir/$workspace_name/tests
#cp bark/*/tests/py*test*.py $build_dir/$workspace_name/tests


echo "Copying manifest from $bazel_manifest to $pip_manifest"
#remove existing manifest
rm $pip_manifest
while IFS=' ' read -r line delim; do

# python files are handled by bazel
if [[ ${line} != *".py"* ]];then
    # remove trailing workspace directory generate in out as package path starts from .bark
    # else to import package we would need from bark_project.bark.examples.etc
    workspace_str_len=${#workspace_name}
    echo "include ${line:$((workspace_str_len+1))}" >> $pip_manifest
fi
done <$bazel_manifest


echo "Moving to build directory"
#move to the directory
cd $build_dir/$workspace_name
# Note:
#plat='any' # for macos update with output of python -c "import distutils.util; print(distutils.util.get_platform())" with all hyphens - and periods . replaced with underscore _
# for example macosx_10_9_x86_64. This does not direct work on linux because of lot of different potential for linux pypi rejects linux_x86_64 tag. so we when a standard wheel
# for linux which will most likely work only on ubuntu or debian based x64 machines. to have support for all versions of linux, manylinux tag is used, which requires that the package
# is built using a standard linux docker provided by https://github.com/pypa/manylinux
python3.7 setup.py clean
python3.7 setup.py sdist bdist_wheel

python3.7 setup.py test

if [ $? -eq 0 ]; then
    echo "Tests Passed!"
else
    echo "Tests Failed!"
    exit 0
fi

echo "Uploading package to PyPi..."
#upload to pypi
python3 -m twine upload --skip-existing dist/*

