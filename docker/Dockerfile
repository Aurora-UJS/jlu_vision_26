# Build environment for jlu_vision_26.
#
# Ubuntu 22.04 on purpose: the upstream README reports gtsam source builds
# breaking on Ubuntu 24, and the host here is 24.04.  Keeping the toolchain in a
# container also means none of this lands in /usr/local on the host.
#
# Scope is trimmed to what the simulator link needs: camera, serial, auto_aim
# and auto_buff.  status_feedback is skipped, which drops open3d and with it
# libc++abi and libglew.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git curl wget ca-certificates pkg-config unzip \
      ninja-build gdb \
      libeigen3-dev libtbb-dev libceres-dev libgoogle-glog-dev \
      libopencv-dev \
      libboost-all-dev \
      libssl-dev libacl1-dev libncurses5-dev \
      python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build parallelism is capped deliberately.  gtsam translation units peak
# around 1-2 GB each, so letting ninja use every core needs far more RAM than
# this host has free and drives it into swap.  Three keeps the peak near 6 GB.
ARG BUILD_JOBS=3

ARG CMAKE_VERSION=3.31.6
RUN wget -q "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh" \
      -O /tmp/cmake.sh \
    && sh /tmp/cmake.sh --skip-license --prefix=/usr/local \
    && rm /tmp/cmake.sh \
    && cmake --version

# iceoryx: the zero-copy transport every node in the framework talks over.
# The build lands in /usr/local/include/iceoryx/v<version>; the framework's
# xmake rule expects a plain ".../iceoryx/v", so link it.
#
# The 2.95 line, not 2.0: the framework includes both the old
# "iceoryx_hoofs/cxx/string.hpp" and the new "iox/string.hpp" /
# "iox/signal_watcher.hpp".  Only the 2.95 transitional releases ship both
# layouts -- on 2.0.8 the iox/ directory does not exist and the build fails at
# the first common_defs header.
ARG ICEORYX_VERSION=v2.95.8
RUN git clone --depth 1 --branch "${ICEORYX_VERSION}" \
      https://github.com/eclipse-iceoryx/iceoryx.git /tmp/iceoryx \
    && cmake -S /tmp/iceoryx/iceoryx_meta -B /tmp/iceoryx/build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
       -DBUILD_SHARED_LIBS=ON -DEXAMPLES=OFF -DBUILD_TEST=OFF \
    && cmake --build /tmp/iceoryx/build --target install -j ${BUILD_JOBS} \
    && ln -sfn "$(ls -d /usr/local/include/iceoryx/v* | head -1)" \
       /usr/local/include/iceoryx/v \
    && ldconfig \
    && rm -rf /tmp/iceoryx

# gtsam 4.3: the factor-graph optimiser behind the observer.  Tests and
# examples are off because they dominate the build and nothing links them.
# System Eigen keeps gtsam and the framework on one Eigen; metis and cephes stay
# internal because the framework links them as metis-gtsam / cephes-gtsam.
ARG GTSAM_VERSION=4.3a2
RUN git clone --depth 1 --branch "${GTSAM_VERSION}" \
      https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
       -DGTSAM_BUILD_TESTS=OFF \
       -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
       -DGTSAM_BUILD_UNSTABLE=OFF \
       -DGTSAM_USE_SYSTEM_EIGEN=ON \
       -DGTSAM_WITH_TBB=ON \
       -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
       -DBUILD_SHARED_LIBS=ON \
    && cmake --build /tmp/gtsam/build --target install -j ${BUILD_JOBS} \
    && ldconfig \
    && rm -rf /tmp/gtsam

# OpenVINO, matching the host's 2024.6.0-17404 but built for jammy.  The host
# copy is the noble build and links against glibc 2.38 symbols
# (__isoc23_strtoul and friends) that do not exist here, so mounting it in
# fails at link time.  Same release, right libc.
ARG OPENVINO_BUILD=2024.6.0.17404.4c0f47d2335
RUN wget -q "https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_${OPENVINO_BUILD}_x86_64.tgz" \
      -O /tmp/openvino.tgz \
    && mkdir -p /opt/intel/openvino_2024 \
    && tar -xzf /tmp/openvino.tgz -C /opt/intel/openvino_2024 --strip-components=1 \
    && rm /tmp/openvino.tgz \
    && ldconfig \
    && test -f /opt/intel/openvino_2024/runtime/lib/intel64/libopenvino.so

# xmake refuses to run as root unless this is acknowledged, and it has to be in
# the environment before the version check below rather than after it.
ENV XMAKE_ROOT=y
# The installer drops the binary here.  Putting it on PATH is more robust than
# symlinking, which depends on the installer's layout staying put.
ENV PATH=/root/.local/bin:$PATH

# xmake drives the framework build and pulls the remaining nine dependencies
# (quill, reflect-cpp, ftxui, ...) itself.  The installer must run under bash:
# a Dockerfile RUN defaults to /bin/sh, where it installs to a different place.
SHELL ["/bin/bash", "-c"]
RUN curl -fsSL https://xmake.io/shget.text | bash \
    && xmake --version

WORKDIR /work
CMD ["/bin/bash"]
