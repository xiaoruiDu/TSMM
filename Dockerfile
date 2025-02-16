FROM ubuntu:latest

# Install necessary build dependencies
RUN apt-get update && apt-get install -q -y \
  cmake \
  doxygen \
  g++ \
  git \
  graphviz \
  libboost-dev \
  libbz2-dev \
  libexpat1-dev \
  libgdal-dev \
  libgeos++-dev \
  liblz4-dev \
  libproj-dev \
  make \
  ruby \
  ruby-json \
  spatialite-bin \
  zlib1g-dev \
  libprotozero-dev \
  && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

## Clone the TSMM repository
#RUN git clone https://github.com/TSMM-DM/TSMM.git
#
## Move into the project directory
#WORKDIR /app/TSMM
#
## Initialize and update submodules
#RUN git submodule update --init --recursive
#
## Create build directory
#RUN mkdir -p build
#WORKDIR /app/TSMM/build
#
## Build the project
#RUN cmake -DCMAKE_BUILD_TYPE=Release ..
#RUN make -j$(nproc)

# Run the TSMM binary as the container entry point
CMD ["/bin/bash"]