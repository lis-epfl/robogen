26/03/2013 ilya.loshchilov@gmail.com

Global Picture:
- Download CMake from http://cmake.org/cmake/resources/software.html and install it.
- Downdlod all required libraries given at https://github.com/amaesani/robogen-simulator : Dependencies
- Compile all downloaded libraries (except Robogen) (STEP1 to STEP7)
- Compile Robogen (STEP8 to STEP10)

To compile the source code you will need few gigabytes of disk space. The compilation will take few hours.
 It is important to note that you are free to choose the location of libraries, but it is suggested to use the same directories for all packages as we use
 in this readme. You can use other directories, but please be consistent! 
 When you will be asked to modify CmakeLists.txt file, do that only if you get some errors (e.g., some packages are not found) during the project generation.
 

STEP 1.
  1.1 Download zlib library from http://www.zlib.net/ , create a new folder c:\devel\ and then another folder C:\devel\zlib-1.2.7\ , unzip zlib archive to 
C:\devel\zlib-1.2.7\ such that you will have directories like C:\devel\zlib-1.2.7\amiga etc.
  1.2 Launch CMake program downloaded from http://cmake.org/cmake/resources/software.html, select 'where is the source code' as 'C:/devel/zlib-1.2.7' and 'where to build the binaries' again as 'C:/devel/zlib-1.2.7'.
In this directory you can find CMakeLists.txt where is decribed how to make you project from the source code.
 Click 'Configure', you will see 'Configuration done' message. Then click 'Generate, you will be asked which 
 compiler to use. In this readme we assume that you will use Visual Studio compiler. After the compilation
 you will find new vcproj files in C:\devel\zlib-1.2.7\. Launch zlib.sln, go to Build menu, Rebuild solution.
 To build zlin in Release configuration, go to Build -> Configuration Manager, Active Solution configuration, select release. Build the projec again.
 You will find two new folders - 'debung' and 'release', inside you will find dll libraries.

STEP 2.
  2.1. Download libpng 1.5 from http://libpng.sourceforge.net/index.html, unzip it in C:\devel\libpng-1.5.14. 
  2.2. Open CMake and try to build it in a similar way. You may get some error that zlib is not found. 
Go to C:\devel\libpng-1.5.14, open CMakeLists.txt and add these two lines after the line 'set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)':
set (ZLIB_INCLUDE_DIR "C:/devel/zlib-1.2.7")
set (ZLIB_LIBRARY "C:/devel/zlib-1.2.7/debug/zlibd.lib")
Try to Configure and Generate source code with CMake again. When you get vcproj files, open png15.vcproj and 
png15_static.vcproj, compiler these two both in Debug and Release configurations. Be sure, that debug and release folders 
are generated and there are dll files inside.

STEP 3.
  3.1 Download OpenSceneGraph from http://www.openscenegraph.org/projects/osg/wiki/Downloads
 and Unzip it in C:\OpenSceneGraph-3.0.1
  3.2 Add the follwing text to C:\OpenSceneGraph-3.0.1\CMakeLists.txt after 'set_property(GLOBAL PROPERTY USE_FOLDERS ON) '
set (ZLIB_INCLUDE_DIR "C:/devel/zlib-1.2.7")
set (ZLIB_LIBRARY "C:/devel/zlib-1.2.7/debug/zlibd.lib")
set (PNG_PNG_INCLUDE_DIR "C:/devel/libpng-1.5.14")
set (PNG_LIBRARY "C:/devel/libpng-1.5.14/debug/libpng15d.lib")
  3.3. Generate VisualStudio files and build dll in Debug and Release configurations

STEP 4.
  4.1 Download Google Protobuf from http://code.google.com/p/protobuf/ and Unzip it in C:\google\protobuf such that
you will have C:\google\protobuf\editors
  4.2 Go to C:\google\protobuf\vsprojects and run extract_includes.bat and convert2008to2005.sh that will make vcproj files
  4.3 Compile vcproj in Debug and Release configurations.

STEP 5.
  5.1 Download ODE from http://sourceforge.net/projects/opende/files/ and Unzip it in c:\ode012 such that you will see
c:\ode012\bindings.
  5.2 Go to c:\ode12\build  and run premake4.exe. It will create vs2005 folder where you can find vcproj files which you later should
compile in Debug and Release Configurations.

STEP 6.
  6.1 Download OpenSceneGraph from http://www.openscenegraph.org/projects/osg/wiki/Downloads and unzip it in
C:\OpenSceneGraph-3.0.1
  6.2 Add the following lines in CMakeLists.txt:
set (ZLIB_INCLUDE_DIR "C:/devel/zlib-1.2.7")
set (ZLIB_LIBRARY "C:/devel/zlib-1.2.7/debug/zlibd.lib")
set (PNG_PNG_INCLUDE_DIR "C:/devel/libpng-1.5.14")
set (PNG_LIBRARY "C:/devel/libpng-1.5.14/debug/libpng15d.lib") 
  6.3 Compile the vcproj projects, do not forget to compile plugins projects, it will take some time, but plugins are needed read model components from files.


STEP 7.
  7.1 Download Boost from http://www.boost.org/ and unzip it in C:\boost_1_52_0 
  7.2 Open C:\boost_1_52_0\boost\config\user.hpp and uncomment  #define BOOST_ALL_DYN_LINK to make it link dynamically.
  7.3 create run.bat with the following text (here msvc-8.0 corresponds to visual studio 2005, with version 8.0)
bjam toolset=msvc-8.0 runtime-link=shared threading=multi --build-type=complete stage link=shared define=_SECURE_SCL=0 define=_SECURE_SCL=0 define=_HAS_ITERATOR_DEBUGGING=0
  7.4 run run.bat, the compilation will take several minutes


STEP8
  8.1 Download Robogen files
  8.2 Create a directory called EA (for example C:/Robogen/EA3/EA), 
  8.3 Copy 'src' directory (src of EA module) to EA\src 
  8.4 Download Paradiseo package from (http://paradiseo.gforge.inria.fr/index.php?n=Download.Download) 
and copy 'eo' directory from Paradiseo to EA\eo

STEP 9. compile and build Cmake in C:/Robogen/EA3/EA/src/utils/network
  9.1. Open CMake
  9.2. Setup 'where is the source code..' as C:/Robogen/EA3/EA/src/utils/network 
  9.3. Setup 'where to build..' as C:/Robogen/EA3/EA/src/utils/network
  9.4. Open C:/Robogen/EA3/EA/src/utils/network/CustomPath.cmake
  9.5. Uncomment (delete # symbols) paths for libraries and set them according to your environment. Save the file.
  9.6. In CMake click configure, if there is no error, then click Generate.
  9.7. If there are errors, probably some required libraries are not installed/compiled or the path to these libraries is not correct.
  9.8. If you get the message "Generating done" then VisualStudio project are compiled successfully.
  9.9. Open C:\Robogen\EA3\EA\src\utils\network\NetworkUtils.sln
  9.10. Rebuild solution (all 3 sub-projects)
  9.11. if may have a problem like "Cannot open include file: 'utils/network/TcpSocket.h' .. ", to fix it, right-click on network-utils, then 'Properties', 'Configuration Properties'->'C/C++'->'General'->'Additional include directories'
and add (by clickling on '...') the following directory: ./../../  (do the same for Release configuration).
  9.12. Rebuild solution again.
  9.13. Now in C:\Robogen\EA3\EA\src\utils\network\ you will find these two files: robogen.pb.cc, robogen.pb.h

STEP 10. build and Launch EA
  10.1 Download and copy EA.vcproj to C:\Robogen\EA3\EA\EA.vcproj and launch it
  10.2 Compile EA.vcproj. If some libraries or files are not found, please check project properties:
include additional directories if needed in project property->configuration properties->C/C++->General->Additional include directories.
 for linkage problems see Linker->Input->Additional Dependencies.
  10.3 You can set configuration file with parameters of evolutionary algorithm by right-clicking on EA project in Solution Explorer, 
 then select Properties->Configuration Properties->Debugging->Command Arguments, this can be set as '@EA.in', where EA.in is your file (you can do the same for Release configuration).
  10.4 To run your EA you will need to execute simulator, which should compiled as will be explained in the following step

STEP 11. Build Simulator module:
  11.1 Open CMake, to generate project files you may need to modify CMakeLists.txt is some package are not found.
In this case, go open C:\Robogen\Simulator3\simulator\simulator\src\cmake\CustomPath.cmake and uncomment all
'set ...' lines. 
  11.2 To build the whole simulator takes 10 minutes in Debug configuration
  11.3 To launch robogen-simulator we should copy all linked dll files to src\Debug for Debug configuration
(and to src\Release for Release configuration, respectively)

dll's:
copy from C:\ode012\lib\debugdoubledll\ 
          ode_doubled.dll  
copy from C:\OpenSceneGraph-3.0.1\bin
          osg80-osgd.dll , ot12-OpenThreadsd.dll, osg80-osgDBd.dll , osg80-osgUtild.dll , osg80-osgViewerd.dll
          osg80-osgGAd.dll, osg80-osgTextd.dll , osg80-osgTerraind.dll

copy from from C:\devel\zlib-1.2.7\debug\
          zlibd.dll zlibd.dll

model files
copy the directory 'models'
to src\models

to setup the port on which robogen-server will be waiting solution-packages from evolutionary algorithm,
right-click on robogen-server in Solution Explorer window, then select Properties, then go to Configuration Properties-> Debugging -> Command Arguments and write 8001   (do the same for the Release configuration)

now when you launch robogen-server from Visual Studio you will see the command line with the text 'Waiting for clients'. In order to launch robogen-server and not other project, you can again right-click on robogen-server in Solution Explorer and then click 'Set as Startup Project'.

you should also copy your directory 'models' to Debug and Release directories