# Crash2Mesh
Read time-variant HDF5 crash test simulation results, extract surfaces as triangle meshes and decimate to acceptable size for rendering purposes.
Code cleanliness is still a work in progress.

# Dependencies:
Most is included as submodules, so you have to clone the repository via .git to be able to build the code.
Simply downloading the source code is not sufficient (except if you download the source code of the correct
branches of all submodules and put it in the extern folder, which is cumbersome).
Other than that you will need to install the HDF5 library binaries matching your build system ([LINK](https://www.hdfgroup.org/downloads/hdf5/)).

# Build:
Via console in main folder (the same folder that contains the top level CMakeLists.txt):
```
mkdir build
cd build
cmake ..
```
Then on UNIX (-j is optional, N is number of launched parallel jobs):
```
make [-jN]
```
OR on Windows using Visual Studio (devenv.exe from Visual Studio installation needs to be in your PATH environment variable):
```
devenv ".\crash2mesh.sln" /Build "Release"
```

Binaries + resources needed for binary are created in bin/Release by default (or bin/Debug for Debug build config).

# Usage:
The GUI should be mostly intuitive.
Hotkeys are listed within a GUI popup.
If something does not work as expected you can check the console logs (if launched from console)
or the logs in folder `logs` (same directory as binary) to see if there was an error or possible bug.
Some invalid decimation configurations are not automatically detected yet and will result
in crashes. This will be fixed as soon as possible.
File export is a bit finicky. You have to manually add the extension (.obj or .c2m) in the
popup window in order to actually save the file in the respective format.
