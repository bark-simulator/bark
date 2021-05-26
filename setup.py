from setuptools import setup, find_packages, Extension
import os,sys
import os
import shlex
import shutil
import setuptools.command.build_ext
import setuptools.command.build_py
import setuptools.command.install
import setuptools.command.sdist
import setuptools.dist
from setuptools import dist
from setuptools.command.install import install
from distutils.command.build import build

with open("README.md", "r") as fh:
    long_description = fh.read()


class CustomBuild(build):
  def run(self):
    self.build_lib = '_build'

try:
  from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
  class bdist_wheel(_bdist_wheel):
    def finalize_options(self):
      _bdist_wheel.finalize_options(self)
      self.root_is_pure = False
except ImportError:
  bdist_wheel = None

class BinaryDistribution(dist.Distribution):
  def is_pure(self):
    return False
  def has_ext_modules(self):
    return True

class InstallPlatlib(install):
  def finalize_options(self):
    install.finalize_options(self)
    if self.distribution.has_ext_modules():
      self.install_lib = self.install_platlib

class BuildExtCommand(setuptools.command.build_ext.build_ext):
  """Overrides default build_ext command to invoke bazel."""

  def run(self):
    if not self.dry_run:
      prebuilt_path = os.getenv('BARK_PREBUILT_DIR')
      if not prebuilt_path:
        # Ensure python_configure.bzl finds the correct Python verison.
        os.environ['PYTHON_BIN_PATH'] = sys.executable
        bazelisk = os.getenv('BARK_BAZELISK', 'bazelisk.py')
        # Controlled via `setup.py build_ext --debug` flag.
        default_compilation_mode = 'dbg' if self.debug else 'opt'
        compilation_mode = os.getenv('BARK_BAZEL_COMPILATION_MODE',
                                     default_compilation_mode)
        build_command = [sys.executable, '-u', bazelisk] + [
            'build',
            '-c',
            compilation_mode,
            '//bark:pip_package',
            '--verbose_failures'
        ]
        if 'darwin' in sys.platform:
          # Note: Bazel does not use the MACOSX_DEPLOYMENT_TARGET environment
          # variable.
          # build_command += ['--macos_minimum_os=%s' % _macos_deployment_target]
          pass
        if sys.platform == 'win32':
          # Disable newer exception handling from Visual Studio 2019, since it
          # requires a newer C++ runtime than shipped with Python.
          #
          # https://cibuildwheel.readthedocs.io/en/stable/faq/#importerror-dll-load-failed-the-specific-module-could-not-be-found-error-on-windows
          build_command += ['--copt=/d2FH4-']

        self.spawn(build_command)
        suffix = '.pyd' if os.name == 'nt' else '.so'
        built_ext_path = os.path.join(
          'bazel-bin/bark/pip_package.runfiles/bark_project/bark/core' + suffix)

      # TODO: what does this do
      # os.makedirs(os.path.dirname(ext_full_path), exist_ok=True)
      copy_to = os.path.dirname(os.path.abspath(__file__)) + "/bark/core.so"
      copy_to_manifest = os.path.dirname(os.path.abspath(__file__)) + "/MANIFEST.in"
      print('Copying extension %s -> %s' % (
          built_ext_path,
          copy_to
      ))
      shutil.copyfile(built_ext_path, copy_to)

      # new_lines = []
      # with open('bazel-bin/bark/pip_package.runfiles/MANIFEST', 'r') as reader:
      #   line = reader.readline()
      #   while line != '':
      #     line = reader.readline()
      #     line = line.replace("bark_project/", "")
      #     new_lines.append(line)

      # with open(copy_to_manifest, 'w') as fo:
      #   for d in new_lines:
      #     fo.write(str(d))

setup(
    name = "bark-simulator",
    version = "1.0.3",
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
    cmdclass={
      'bdist_wheel': bdist_wheel,
      'build_ext': BuildExtCommand,
      'install': InstallPlatlib,
      'build': CustomBuild
    },
    test_suite='nose.collector',
    tests_require=['nose'],
    include_package_data=True,
    zip_safe=False,
    distclass=BinaryDistribution,
    python_requires='>=3.6',
)

