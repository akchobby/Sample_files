# Notes

## Shared libs
* Called in runtime and handled by OS, hence the links must be properly done

* The linking can be done properly via ldconfig (updates the cache with the newly loaded dynamic library). Check this [link](https://www.cprogramming.com/tutorial/shared-libraries-linux-gcc.html) for more details on linking shared libs

* Shared libraries created are \*.so files 

## Static libs
* Static libraries are basic merged into the final executable
* \*.a files are generated which are then linked to executable by linker(part of the compiler here) -- basically an archive("zip") of .o files and other.[Further details](https://stackoverflow.com/questions/654713/o-files-vs-a-files/654757)

## Compiler vs Linker

* gcc or g++ are not actually the compiler its actually cc1,cc1plus respectively
[Further information](https://stackoverflow.com/questions/5759353/is-g-both-a-c-compiler-and-a-linker)
