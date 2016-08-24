russianaicup2015
================

My solution of Russian AI Cup 2015 -- CodeRacing (http://russianaicup.ru/)

Results:
* Finals ([v10](7df7a90ac86c035f80981d904514755dc04aa585)) -- 42nd place, 39% wins
* Sandbox overall -- 18th place

To compile/run, install [CMake](https://cmake.org) and run:

    mkdir out
    cd out
    cmake ..
    make -j 4
    cd ..
    out/LocalRunner

Pass `-vis` to `LocalRunner` to observe visualization.

To use the renderer plugin, open the IDEA project in `lib/plugins`, invoke Make project and restart the local runner.
