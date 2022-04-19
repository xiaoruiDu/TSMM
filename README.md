# TSMM

## 1- Develop Environment

- 1-1 Installing Dependencies
  [Osmium](https://osmcode.org/libosmium/manual.html#dependencies)


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

## 3- Output

After running **./tsmm** you can get all intermediate results in output folder, and **12_final__allLayer.osm** is
the final simplified road network. You can find the detailed information on our [paper](paper/paper.pdf).

- 01_link_ways__allLayer.osm
- 02_first_buffer__allLayer.osm
- 03_re_buffer__allLayer.osm
- 05_extend__allLayer.osm
- 07_insert__allLayer.osm
- 08_del_redundant__allLayer.osm
- 09_merge__allLayer.osm
- 10_clean__allLayer.osm
- 12_final__allLayer.osm


## 4- Processing display


![](images/display.gif)