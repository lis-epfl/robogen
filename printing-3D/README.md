robogen-3DPrint
===============
Part of the Robogen Course project, LIS Laboratory, EPFL.

Modified by Deniz Aydin <deniz.aydin@epfl.ch>, Nov 5, 2013.

Original by Gregoire Heitz <gregoire.heitz@epfl.ch>, April 26th, 2013.

This repository contains software to produce STL files needed to 3D print parametric components like the wheels, whegs, and parametric bricks.
The STL files will be generated based on the parameters discovered by the Evolutionary Algorithm.

This ReadMe file is divided into three parts:

1) Description of the structure of the robogen-3DPrint folder;

2) Installation of FreeCAD software;

3) Description of how to produce the STL files for the different body components in preparation for 3D printing.

-----------------------------------------------------------------------------------
-----------------------------------------------------------------------------------

Structure of robogen-3DPrint folder

This folder contains 3 sub-folders:
- FreeCAD_Modules contains the scripts to produce the STL files for the parametric components, calling FreeCAD appropriately;
- STL_Files contains the generated STL files;
- utils contains the python scripts needed to orchestrate the production of the final STL files.

This folder also contains the following files:
- someBody.txt is an example of what is produced by the Evolutionary Algorithm. 
- getParametricValue_LINUX.sh (which works for MAC too) and getParametricValue_WINDOWS.bat scripts to parse YOUR bets robot txt file.

-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------

How to install FreeCAD

- First go to http://sourceforge.net/projects/free-cad/
And Download the FreeCAD software.

It is a cross-Platform, open source software to design 2D and 3D models with an integrated python interpreter. 
We will use its capability to design each required parametric component, by simply executing the right scripts in FreeCAD.

-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------

What to do

- From robogen/python run:
	python json_converter.py ../results/YOUR_RESULTS_FOLDER/GenerationBest-XXX.json someBody.txt
to convert your evolved robot back into the simple txt format.
- Cut and paste someBody.txt into robogen/printing-3D
- Then open a terminal and go to the robogen/printing-3D folder. From there execute the batch script corresponding to your platform (getParametricValues_* batch file).
  The script will produce a Readme.txt file where you can see how many parametric components should be generated,
  and a python script for each parametric component in the FreeCAD_Modules folder.
- Open FreeCAD software. Open all Call*.py scripts, located in FreeCAD_Modules folder, and run each of them separately.
For a some reason, the wheel takes quite long to generate in FreeCAD (about 3min) so just let the software run.
You should check the "report view" in the bottom left of the window to make sure that no errors occured during part generation.
- The scripts will export the generated STL files in the STL_Files folder.
For Windows : If there is an error in FreeCad (e.g. "Module XYZ cannot be found"), it might be because of the paths. You can manually edit the paths by changing '\' to '/'. 

-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------

If you encounter some issues, we will do our best to help you resolve them. Feel free to ask any question.
We hope that you will enjoy printing your homegrown robots!

Your friendly neighbourhood TAs,

Gregoire Heitz, <gregoire.heitz@epfl.ch>, Deniz Aydin <deniz.aydin@epfl.ch>, Nov 5th 2013. 
