# Tips

* Make a build directory from where you can run 

```bash
cmake <path/to/cmakelists.txt> 

cmake --build .
```

The first command is to generate naticve build system, the second is the equivqlent of running < make > in linux (cmake generates a makefile)

* To avoid manually making  build directories one can also run the below cmd

```bash

cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release

cmake --build build

```

The -H flags tells cmke where to look for the root CMakeLists.txt, the -B flag provides cmake the path where  the build must be done and the final -D basically sets the env variable CMAKE_BUILD_YPE which sets the compiler flags, in this case I used the Release value so that the executable is of a minimum size, one can also use Debug  so that CMake sets the -g flag to the compiler to ensure we can debug the executable
 

