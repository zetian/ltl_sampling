## CMake Project Structure

### Folders

* data: input data for tests and output results
* src: source code

### Modules

* map: code for processing map data 
* h2c: graph related code
* vis: visualization of graph and search results
* apps: applications created from the above libraries

### Workflow

1. Process Map: read map from an image or create map from code; then create a squre grid data structure from the map information
2. Construct Graph: create graph from the squre grid data structure; do additional graph manipulation and search path
3. Visualize graph and search result


### Reference

* http://stackoverflow.com/questions/2275076/is-stdvector-copying-the-objects-with-a-push-back
* http://stackoverflow.com/questions/14700354/how-to-change-compilation-flags-for-a-folder-with-cmake
* http://stackoverflow.com/questions/13638408/cmake-override-compile-flags-for-single-files
* http://stackoverflow.com/questions/16850992/call-a-c-function-from-c-code
* http://stackoverflow.com/questions/1970698/using-malloc-for-allocation-of-multi-dimensional-arrays-with-different-row-lengt
* http://stackoverflow.com/questions/5666214/how-to-free-c-2d-array
* http://stackoverflow.com/questions/3840582/still-reachable-leak-detected-by-valgrind