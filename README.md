# TSMM

## 1- Develop Environment

  ```commandline
  sudo apt-get install -q -y \
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
  libprotozero-dev
  ```

## 2- Build

  ```commandline
    git clone https://github.com/osmcode/libosmium.git
    git clone https://github.com/TSMM-DM/TSMM.git
    cd TSMM
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j4
    ./tsmm
    ```