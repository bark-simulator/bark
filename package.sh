#!/bin/bash

pkg_name='pip_package'
workspace_name='bark_project'

# activate virtual environment
source ./bark/python_wrapper/venv/bin/activate 

echo "Building package"
bazel run //bark:$pkg_name

if [ $? -eq 0 ]; then
    echo "Build Suceeded!"
else
    echo "Build Failed!"
    exit 0
fi

build_dir=bazel-bin/bark/$pkg_name.runfiles

echo "Copying setup.py to project pirectory at $build_dir/$workspace_name"
#copy setup.py file to build directory
cp setup.py $build_dir/$workspace_name

echo "Copying README.md to project directory at $build_dir/$workspace_name"
#copy READEME.md file to build directory
cp README.md $build_dir/$workspace_name

echo "Copying LICENSE to project directory at $build_dir/$workspace_name"
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
    # remove trailing workspace directory generate in out as package path starts from .bark
    # else to import package we would need from bark_project.bark.examples.etc
    workspace_str_len=${#workspace_name}
    echo "include ${line:$((workspace_str_len+1))}" >> $pip_manifest
fi
done <$bazel_manifest


echo "Moving to build directory"
#move to the directory
cd $build_dir/$workspace_name

# build the wheel
python3.7 setup.py clean
python3.7 setup.py sdist bdist_wheel

#python3.7 setup.py test

#if [ $? -eq 0 ]; then
#    echo "Tests Passed!"
#else
#    echo "Tests Failed!"
#    exit 0
#fi

# check if manylinux argument passed. if so build
# a manylinux wheel. https://github.com/pypa/manylinux
wheeldir='dist'
# alternate: use platform instead of passing variable
#plat=$(python -c "import distutils.util; print(distutils.util.get_platform())")
if [[ $# -gt 0 ]] ; then
    if [[ $1 -eq 'manylinux' ]] ; then
        wheeldir='wheelhouse'
        for whl in dist/*.whl; do
            if ! auditwheel show "$whl"; then
                echo "Skipping non-platform wheel $whl"
            else
                auditwheel repair "$whl"
            fi
            # install the wheel
            /opt/python/cp37-cp37m/bin/pip install $whl
        done
    fi
fi

#echo "Uploading package to PyPi..."
# upload to pypi
#python3.7 -m twine upload --skip-existing $wheeldir/*

