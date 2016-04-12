RoboGen -- Robot generation through artificial evolution
=================
Joshua Auerbach (joshua.auerbach@epfl.ch)

Laboratory of Intelligent Systems, EPFL

**Read carefully this document and the online documentation: http://robogen.org/docs/robogen-software-suite/
before starting to do anything!**

**Please check at the end of the file for some tricks you might try in order**
**to improve the stability of the simulator**

-------------
License
-------------

The ROBOGEN Framework is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License (GPL)
as published by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

-------------


## The simulator

The simulator can be compiled on Linux, Windows and MAC OSX. 
The code has matured from last year's course but is still  experimental.

Please, use github as much as possible to report bugs, issues and ideas for improvement.
Whenever you open a ticket for a bug, make sure your description contains:
1) Machine on which the error happened, OS version, installed libraries versions (See dependendencies later)
2) A List of instructions needed to reproduce your bug.

Given the experimental nature of the code, we will take into consideration any effort in helping us to improve the code in the course evaluation!

## Contributions

Here, we will keep track of contributions both for the final evaluation and 
for maintaining your name associated to the project.

LIS members who have made contributions :
* **amaesani** Andrea Maesani, andrea.maesani@epfl.ch
* **loshchil** Ilya Loshchilov, ilya.loshchilov@gmail.com
* **GregoireH** Grégoire Heitz, gregoire.heitz@gmail.com
* **daydin** Deniz Aydin, deniz.aydin@epfl.ch
* Przemek Kornatowski, przemyslaw.kornatowski@epfl.ch
* Ludovic Daler, ludovic.daler@epfl.ch
* And, LIS alumnus Pradeep Ruben Fernando



Thanks to past students
* **Mikaz-fr** (Roy Michaël, michael.roy@epfl.ch) contributed instructions to compile the code 
 on MAC OS X and adapted part of the CMAKE code for MAC OS. 
* **tcies** (Titus Cieslewski, titus.cieslewski@epfl.ch) who did a lot of work to improve the code base and documentation

## Git Repository

To obtain a version of the simulator code, and pull future updates you need to install
a client for GIT. Please quickly go through the GitHub help to get a grasp of how GIT works 
(https://help.github.com/articles/set-up-git).

Please request push rights and clear any code modifications with the assistants before pushing!
Remember **PULL is OK**, **PUSH is dangerous!!** and MUST be discussed with us.

## Requirements

You need to download and install Cmake (http://www.cmake.org/), which
will aid you in compiling the source code and/or automatically create a project for your favorite IDE.

Before starting to compile RoboGen you will need to download and 
make available to your compiler includes and library paths to 
* zLib (http://www.zlib.net/)
* Boost (http://www.boost.org/)
* Google Protobuf (http://code.google.com/p/protobuf/)
* Gnuplot (http://www.gnuplot.info/download.html)
* OpenSceneGraph (http://www.openscenegraph.org/projects/osg/wiki/Downloads) 
* Cmake (https://cmake.org/download/)
* Build-essential (packages.ubuntu.com/precise/build-essential)
* libtool (https://www.gnu.org/software/libtool/)
* Automake (https://www.gnu.org/software/automake/)
* libpng 1.5 (http://libpng.sourceforge.net/index.html)
* Jansson 2.4+ (http://www.digip.org/jansson/)
* ODE (http://sourceforge.net/projects/opende/files/) 

ODE must be compiled in double precision mode (dDOUBLE preprocessor flag)

### Linux 

On Linux we advise to install all the needed dependencies from available repositories, except for ODE 
which has to be built in double precision mode.
For example, in Ubuntu Linux 12.04 you can type
	
    sudo apt-get install zlib1g zlib1g-dev libboost1.54-all-dev libprotobuf-dev protobuf-compiler \
    gnuplot libopenscenegraph-dev libopenscenegraph99 cmake build-essential \
    libtool automake libpng12-dev libjansson-dev git
    

to install the required dependencies.

Note: If you are using a different version of Ubuntu replace libboost1.54-all-dev and libopenscenegraph99 with the versions that are available in your repositories (tested with Boost 1.48 or newer and libopenscenegraph80)

Note: If you wish to generate files to use with the WebGL Visualizer, we recommend installing the latest version of jansson from source.  This will allow you create significantly smaller simulation files.

Note: We also recommend installing nautilus-open-terminal (restart with ‘nautilus -q’ afterwards) so that you can open the terminal in a given directory with a right-click. For other distributions, package names may vary. 

Note: if you would like to be able to use scriptable scenarios for writing custom fitness functions and scenario configurations without needing to modify the code, then you will additionally need Qt5, since this functionality relies on QtScript. This can be done as follows.  (Without this, RoboGen will still build, but this functionality will be disabled).

    sudo apt-get install qt5-default qtscript5-dev

Next, download the Open Dynamics Engine version 0.14 from https://bitbucket.org/odedevs/ode/downloads. Follow the instructions found under ‘BUILDING WITH AUTOTOOLS’ in INSTALL.txt to install it. For the configuration step and the build, use:

    ./configure --enable-double-precision --with-cylinder-cylinder=libccd
    sudo make install -j


### Windows

To assist the compilation of ROBOGEN on Windows, we preship an archive containing all the already compiled 
dependencies (x86, 32bits, available http://robogen.org/docs/get-started/robogen-windows/). 
To avoid problems, download and install Visual Studio Express 2010 (http://www.microsoft.com/visualstudio/eng/downloads).

### MAC OS X

Some help can be found on the [dedicated wiki page](https://github.com/jauerb/robogen/wiki/Building-robogen-simulator-on-Mac).

## RoboGen Build

1) Once the required dependencies are satisfied (all code compiled successfully), clone this repository
to get all the needed source code of the simulator

    git clone https://github.com/lis-epfl/robogen
    
2) Modify the paths in src/cmake/CustomPath.cmake pointing them to the correct libraries/include directories.
Note that you need to modify only those paths pointing to non-standard locations, i.e. if you installed a library under Linux under a standard path (e.g. /usr/lib) you do not need to update the corresponding line in CustomPath.cmake

On Linux, you'll probably not modify anything on this file, except if you have the ODE library installed in non-default paths. 
In this case, you will have to modify the lines related to the ODE installation paths in CustomPath.cmake (Look for the Linux section).

On Windows, you need to download and extract the content of the precompiled libraries archive (http://robogen.org/docs/get-started/robogen-windows/) and uncomment the fist line of CustomPath.cmake pointing it to the utils/ folder contained
in the archive. Watch out that when you extract the archive you might generate a containing folder named as the archive.
For example you might end up having a structure like:

     + Robogen-dev-Windows << DO NOT use this path...
         |-- utils_debug << ...instead CustomPath.cmake should point to this folder
         |-- run
     
3) We will call the location you have cloned this repository to as ROBOGEN_HOME.
This contains the source code (robogen/src) and a build directory.
At this point you should have something like this:

      + robogen
         |-- build
         |-- src

Then depending on your OS the next steps might change.
On Linux, MAC OS X and Windows/MinGW, from a terminal, enter ROBOGEN_HOME/build and run 

    cd ROBOGEN_HOME/build
    cmake -DCMAKE_BUILD_TYPE=Release -G"Unix Makefiles" ../src/
    make -j3
    
Note: The -j flag of make enables parallel build. We have observed that fully parallelizing can make the system lag, so we recommend using one instance less, which is 3 for a four-core system.

Note: On Mac 10.6 Snow Leopard you’ll need to go into src/cmake/CustomPath.cmake and change set (PNG_LIBRARY “${UTILS_PATH}/lib/libpng.a”) to set (PNG_LIBRARY “${UTILS_PATH}/opt/libpng/lib/libpng.a”)

If everything compiles you can continue to usage examples (http://robogen.org/docs/usage-examples/) to start using RoboGen.

If anything goes wrong, it is possible that your dependencies are installed in different directories than ours. The first place to look for solutions is robogen/src/cmake/CustomPath.cmake, which specifies the locations of various dependencies.  If you can’t figure it out on your own, head on over to our Google Group (https://groups.google.com/forum/#!forum/robogen-users) and ask for help.

On Windows + Visual Studio, open CMAKE GUI, enter the ROBOGEN_HOME/src in "Where to find the source" and ROBOGEN_HOME/build
in "Where to build the source". Click on Configure and when done, on Generate to create a solution for Visual Studio.
Then, from Visual Studio open the solution "RoboGen.sln" that can be found in ROBOGEN_HOME/build.

### Generating projects for your IDE (Eclipse, Visual Studio, etc.)

Using CMAKE you can also generate projects for different IDEs. 
For example, to generate a project that can be imported in eclipse

    cd ROBOGEN_HOME/build
    cmake -DCMAKE_BUILD_TYPE=Debug -G"Eclipse CDT4 - Unix Makefiles" ../src

Please check the CMAKE manual for the correct command to generate projects for other IDEs.

## Executables

The most important executables are:
* robogen-file-viewer, a small utility to visualize the robot structures.
* robogen-evolver, the main evolutionary algorithm software.  Runs all components of the evolutionary algorithm, sends robots to
  robogen-server to be evaluated.
* robogen-server, the main simulator software, to be used as a server software. Listens for connections from robogen-evolver
  evaluates robots and returns their fitness to the EA.

You can run them, launching them from terminal with the following parameters:

    robogen-file-viewer ROBOT_FILE CONF_FILE
    
where ROBOT_FILE is the robot structure and configuration file is a file containing the ROBOGEN simulator configuration.
Once the robogen-file-viewer is open, you can pause/unpause the simulation pressing "P" on your keyboard.

    robogen-evolver SEED OUTPUT_DIRECTORY EVOLUTION_CONF_FILE
    
where SEED is an integer specifying the seed for the pseudo-random number generator (used for making runs reproduceable),
OUTPUT_DIRECTORY specifies where OUTPUT should be written and EVOLUTION_CONF_FILE is an evolutionary conf file for defining
your run.  

    robogen-server PORT
    
where PORT is the port on which the server will listen for connections (we recommend 8001).

See http://robogen.org/docs/usage-examples/ for more details.
    
### Configuration file

See http://robogen.org/docs/evolution-configuration/ for in depth documentation.

## Running the Robogen Simulator

Check in the ROBOGEN_HOME/examples/ folder for sample configuration files and example robots that can be visualized.

On Windows, you need to copy your executables and model folder in the run/ folder contained in the utils.zip archive
that you downloaded from http://robogen.org/docs/get-started/robogen-windows/

## Scenario descriptions

[WARNING] As the rugged terrain loading from images is broken, you might want to use many obstacles in a scenario 
to create rugged terrains. To do that, specify the terrain as flat, and insert more obstacles in the obstacles
configuration file.

### Racing

The aim of a robot evolved under this scenario is to cover the largest distance as possible in the given amount of time.
The fitness is computed as the Euclidean distance between the center of mass of the robot at the beginning and the end of the 
simulation time.

### Chasing

A light source is created at the center of the area (0, 0, 10cm). Robots must get as close as possible to this light source,
independently from the specified starting position.
Make sure to specify starting position at a certain distance from the light source, otherwise robots might
penetrate the light source and the simulator will fail.
The fitness is computed as the sum of the distances between the center of mass of the robot and the light source over all
the simulation time.

## Making the simulation more stable

If you have stability problems when simulating your robot you might try to modify the ERP and CFM parameters, 
described [here](http://ode-wiki.org/wiki/index.php?title=Manual:_All&printable=yes#Joint_error_and_the_Error_Reduction_Parameter_.28ERP.29) to modify the way ODE internally solves constraints and forces.

Mikaz-fr reported some success modifying ERP and CFM (remember to change both of them in FileViewer.cpp, RobogenServer.cpp,
and of course recompile the simulator code) to the following values.
However, the choice of values is strictly dependent on the structure being simulated, so try to play around with them.

    dWorldSetERP(odeWorld, 0.8);        // Error correction parameter
    dWorldSetCFM(odeWorld, 0.5*10e-3);      // Constraint force mixing

