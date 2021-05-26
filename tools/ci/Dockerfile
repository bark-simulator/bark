# use base manylinux image based on centOs 6
FROM quay.io/pypa/manylinux2014_x86_64
USER root

# setup jdk environment variables
ENV JAVA_HOME="/usr/lib/jvm/java-1.8.0-openjdk-1.8.0*" \
    PLATFORM="el6" \
    PATH=/root/bin:$PATH
ENV BASH_ENV="/usr/bin/scl_enable" \
    ENV="/usr/bin/scl_enable" \
    PROMPT_COMMAND=". /usr/bin/scl_enable"

# install openjdk for bazel
RUN yum install -y centos-release-scl unzip vim wget cmake3 doxygen ccache sudo \
    && yum install -y devtoolset-7 rh-python36 \
    && yum install -y java-1.8.0-openjdk java-1.8.0-openjdk-devel

# install python 3 for compiling bazel. Its different from python
# used for initializing virtual for bark.
RUN source scl_source enable devtoolset-7 rh-python36 \
    && gcc --version \
    && python -V \
    && cmake3 --version \
    && pip install --upgrade pip \
    && pip3 install protobuf

# setup with gcc-7 and py3.6
RUN echo "unset BASH_ENV PROMPT_COMMAND ENV" > /usr/bin/scl_enable \
    && echo "echo 'using user scl_enable script'" >> /usr/bin/scl_enable \
    && echo "source scl_source enable rh-python36 devtoolset-7" >> /usr/bin/scl_enable \
    && echo "JAVA_HOME='/usr/lib/jvm/java-1.8.0-openjdk-1.8.0*'" >> /usr/bin/scl_enable \
    && echo "FULL_JAVA_HOME=$(readlink -f $JAVA_HOME)" >> /usr/bin/scl_enable \
    && echo "export JAVA_HOME=\$FULL_JAVA_HOME" >> /usr/bin/scl_enable
RUN chmod a+x /usr/bin/scl_enable

# build libstacktrace . Required to compile boost a (requirement for bark)
# from source as libstacketrace is not bundled with the devtoolset-7
# toolkit for gcc available in the image.
RUN wget -O libbacktrace-master.zip https://github.com/ianlancetaylor/libbacktrace/archive/master.zip \
    && unzip libbacktrace-master.zip \
    && cd libbacktrace-master \
    && ./configure --prefix=/opt/rh/devtoolset-7/root/usr \
    && make \
    && make install \
    && cd /

# build bazel
ENV BAZEL_VERSION=3.4.1
ENV BAZEL_LINKLIBS="-l%:libstdc++.a"

RUN wget https://github.com/bazelbuild/bazel/releases/download/$BAZEL_VERSION/bazel-$BAZEL_VERSION-dist.zip \
    && unzip bazel-$BAZEL_VERSION-dist.zip -d bazel-$BAZEL_VERSION \
    && cd bazel-$BAZEL_VERSION \
    && ./compile.sh \
    && mv /bazel-$BAZEL_VERSION/output/bazel /usr/bin/bazel

# install virtual env for bark using default python. switch to
# target when container is running.
ENV PATH=/opt/rh/rh-python36/root/usr/bin:$PATH
RUN pip3 install --upgrade pip
RUN pip3 install virtualenv

# Setup Bark as work dir
CMD [ "/bin/bash" ]
WORKDIR /bark
