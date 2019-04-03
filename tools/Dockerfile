FROM openjdk:8

ENV PATH /opt/conda/bin:$PATH
ENV DISPLAY :0


# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_python_venv.sh
RUN apt-get update

# renewe bash
CMD [ "/bin/bash" ]
WORKDIR /bark