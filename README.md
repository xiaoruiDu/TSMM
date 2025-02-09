# TSMM

## 1- Develop Environment

- 1-1 Installing Dependencies
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

- 2-0 open main.cpp and change the osmPath to your osmPath.
    ```cpp
    
    int main(int argc, char* argv[]) {
    
        std::string osmPath = "path_to_your_osm_file";  /// change it to your osm Path
        OSMManager* osmManager = new OSMManager(osmPath);
        osmManager->initialize();
        delete osmManager;
    }
    
    ```

- 2-1 create a build folder
    ```commandline
    mkdir -p build
    cd build
    
    ```
- 2-2 execute <span style="color:pink">CMake</span> command.
    ```commandline
  cmake -DCMAKE_BUILD_TYPE=DEBUG -S ../ -B ./
  
  ```

- 2-3 run make and wait for the build to finish
    ```commandline
    make -j4
    ```
- 2-4 executable binay file can be found in the build folder
    ```commandline
    ./tsmm
    
    ```

