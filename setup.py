from setuptools import setup, find_packages, Extension
import os,sys

with open("README.md", "r") as fh:
    long_description = fh.read()
    

# A dummy native extension to mark module as platform specific
ext_modules= []
try:
    os.mkdir('build')
except FileExistsError:
    # directory already exists - is already created by earlier run
    pass
open('build/temp.c','w').close()
temp_ext = Extension('_temp', sources=['build/temp.c'])
ext_modules.append(temp_ext)

setup(
    name = "bark-simulator",
    version = "1.0.1",
    description = "A tool for Behavior benchmARKing",
    long_description_content_type="text/markdown",
    long_description=long_description,
    classifiers = ["Development Status :: 4 - Beta",
                   "Intended Audience :: Science/Research",
                   "License :: OSI Approved :: MIT License",
                   "Operating System :: OS Independent",
                   "Programming Language :: Python :: 3.7"],
    keywords = "simulator autonomous driving machine learning",
    url = "https://github.com/bark-simulator/bark",
    author = "Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler",
    author_email = "autonomous-driving@fortiss.org",
    license = "MIT",
    packages=find_packages(),
    install_requires=[
      "matplotlib>=3.3.2",
      "numpy>=1.18.1",
      "lxml>=4.4.2",
      "scipy>=1.4.1",
      "sphinx>=2.3.1",
      "sphinx_rtd_theme>=0.4.3",
      "pandas>=0.24.2",
      "autopep8>=1.4.4",
      "cpplint>=1.4.4",
      "pygame>=1.9.6",
      "aabbtree>=2.3.1",
      "ray>=0.8.5",
      "psutil>=5.7.2",
      "notebook>=6.0.3",
      "jupyter>=1.0.0",
      "ipython>=7.13.0"
    ],
    ext_modules=ext_modules,
    test_suite='nose.collector',
    tests_require=['nose'],
    include_package_data=True,
    zip_safe=False,
    python_requires='>=3.7',
)

