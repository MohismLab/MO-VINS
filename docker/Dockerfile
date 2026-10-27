FROM debian:bookworm
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
 build-essential cmake ninja-build pkg-config libopencv-dev python3 python3-numpy python3-opencv \
 git ca-certificates curl bc bison flex libssl-dev libncurses-dev device-tree-compiler \
 rsync cpio unzip file wget patch perl gawk texinfo automake autoconf libtool \
 libelf-dev locales python3-venv python-is-python3 python3-pyelftools u-boot-tools \
 dosfstools mtools && rm -rf /var/lib/apt/lists/*
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
WORKDIR /work
