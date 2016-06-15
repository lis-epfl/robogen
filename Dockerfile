FROM ubuntu:14.04
RUN sudo \
      apt-get update && \
      apt-get install -y software-properties-common && \
      add-apt-repository -y ppa:george-edison55/cmake-3.x && \
      apt-get update && \
      apt-get install -y \
        automake \
        cmake build-essential \
        git \
        gnuplot \
        libboost1.54-all-dev \
        libjansson-dev \
        libprotobuf-dev \
        libopenscenegraph-dev \
        libopenscenegraph99 \
        libpng12-dev \
        libssl-dev \
        libtool \
        protobuf-compiler \
        qt5-default \
        qtscript5-dev \
        zlib1g \
        zlib1g-dev
ADD https://bitbucket.org/odedevs/ode/downloads/ode-0.14.tar.gz /deps/ode.tar.gz
RUN tar xf /deps/ode.tar.gz -C /deps && \
      mv /deps/ode-0.14 /deps/ode && \
      rm /deps/ode.tar.gz
WORKDIR /robogen
RUN git submodule update --init --recursive
WORKDIR socket.io-client-cpp
RUN cmake -DBOOST_VER:STRING=1.54 ./ && \
      make install -j$(nproc)
WORKDIR /deps/ode
RUN ./bootstrap && \
      ./configure --enable-double-precision --with-cylinder-cylinder=libccd && \
      make install -j$(nproc)
RUN mkdir /robogen/build
ADD ./src /robogen/src
ADD ./arduino /robogen/arduino
ADD ./build_utils /robogen/build_utils
ADD ./models /robogen/models
WORKDIR /robogen/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ../src -DENABLE_SOCKET_IO=ON  && \
      make -j$(nproc)
ENTRYPOINT ["/robogen/build/robogen-server-sio"]
CMD ["http://robogen.org:3000/app", "public"]
