# Docker deploy

## Login into Dockerhub

`docker login`

## Build & Deploy

Go in the `/tools` folder and run:
`docker build -t barksim/bark .`
`docker push barksim/bark:latest`

Or for manylinux:
`docker build -t barksim/bazel-manylinux .`
`docker push barksim/bazel-manylinux:latest`