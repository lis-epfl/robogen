#!/bin/bash
#Author Gregoire Heitz <gregoire.heitz@epfl.ch>
# modified last on Feb 14 2014 by Deniz Aydin deniz.aydin@epfl.ch
#Goal : Parse the bodyRepresentation.txt file to get the parametric Part Parameters
# For the robogen course project, LIS EPFL.

# Create those 3 empty files, even if they alrady exist
> utils/intermediate.h
> utils/intermediatebis.h
> utils/intermediateTer.py
> Readme.txt

#User information on what is going on
echo please wait until the body has been parsed

#global variables declaration
let "passivewheel=0"
let "activewheg=0"
let "parametricbrick=0"
let "activewheel=0"

CurrentDir=$PWD

PreviousType=
PreviousRadius=
PreviousLength=
PreviousInclAngle=
PreviousRotAngle=

echo "sys.path.append("\"$CurrentDir/utils/Mod"\")" > utils/intermediateTer.py

#file name to be parsed.
FILENAME="my_robot.txt"

# read file, line by line, and get all of the elements.

while read f1 f2 f3 f4 f5 f6 f7
do
      #Counter number of passivewheels (W), activewhegs (G) and parametricbricks aka parametric bar joints (B), and activewheels(J)
	if [[ "$f1" == "" ]]; then
		break	
	elif [[ "$f2" == "PassiveWheel" ]]; then
		let "passivewheel+=1"
		PreviousType="$f2"
		PreviousRadius="$f5"
	elif [[ "$f2" == "ActiveWheg" ]]; then
		let "activewheg+=1"
		PreviousType="$f2"
		PreviousRadius="$f5"
	elif [[ "$f2" == "ParametricJoint" ]]; then
		let "parametricbrick+=1"
		PreviousType="$f2"
		PreviousLength="$f5"
		PreviousInclAngle="$f6"
		PreviousRotAngle="$f7"
	elif [[ "$f2" == "ActiveWheel" ]]; then
		let "activewheel+=1"
		PreviousType="$f2"
		PreviousRadius="$f5"
	fi
	
	#Set which parameter will be discribed there : radius, length, inclinationangle, rotationangle
	#if [ "$f1" = "paramName:" ]; then
	#	PreviousParam="$f2"
	#fi
	

	#Produce the scripts for each Parametric Part
	
	if [[ "$PreviousType" == "PassiveWheel" ]]; then
		#Fill the parameter value needed in python script to be executed in FreeCAD
		echo "radiusExtern = $PreviousRadius" > utils/intermediate.py
		echo Path="\"$CurrentDir/STL_Files/PassiveWheel$passivewheel.stl"\" >> utils/intermediate.py
		> FreeCAD_Modules/CallWheel$passivewheel.py
		#concatenate CallWheelPART1.py+utils/intermediate.py+CallWheelPART2.py in one file : the CallWheel python script which will be executed by FreeCAD to generate Paramteric Parts
		cat utils/Header.py utils/intermediateTer.py utils/CallWheelPart1.py utils/intermediate.py utils/CallWheelPart2.py > FreeCAD_Modules/CallWheel$passivewheel.py

	elif [[ "$PreviousType" == "ActiveWheel" ]]; then
		#Fill the parameter value needed in python script to be executed in FreeCAD
		echo "radiusExtern = $PreviousRadius" > utils/intermediate.py
		echo Path="\"$CurrentDir/STL_Files/ActiveWheel$activewheel.stl"\" >> utils/intermediate.py
		> FreeCAD_Modules/CallWheel$activewheel.py
		#concatenate CallWheelPART1.py+utils/intermediate.py+CallWheelPART2.py in one file : the CallWheel python script which will be executed by FreeCAD to generate Paramteric Parts
		cat utils/Header.py utils/intermediateTer.py utils/CallWheelPart1.py utils/intermediate.py utils/CallWheelPart2.py > FreeCAD_Modules/CallWheel$activewheel.py

	elif [[ "$PreviousType" == "ActiveWheg" ]]; then
		#Fill the parameter value needed in python script to be executed in FreeCAD
		echo "radiusExtern = $PreviousRadius" > utils/intermediate.py
		echo Path="\"$CurrentDir/STL_Files/ActiveWheg$activewheg.stl"\" >> utils/intermediate.py
		> FreeCAD_Modules/CallWheg$activewheg.py
		#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallWheg python script which will be executed by FreeCAD to generate Paramteric Parts
		cat utils/Header.py utils/intermediateTer.py utils/CallWhegPart1.py utils/intermediate.py utils/CallWhegPart2.py > FreeCAD_Modules/CallWheg$activewheg.py

	elif [[ "$PreviousType" == "ParametricJoint" ]]; then
		#Fill the parameter value needed in python script to be executed in FreeCAD
		echo "heightJoin = $PreviousLength" > utils/intermediate.py
		echo "inclAngle = $PreviousInclAngle" > utils/intermediatebis.py
		echo Path="\"$CurrentDir/STL_Files/ParametricJoinPartB$parametricbrick.stl"\" >> utils/intermediatebis.py
		> FreeCAD_Modules/CallJoinB$parametricbrick.py
		#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartA python script which will be executed by FreeCAD to generate Paramteric Parts
		cat utils/Header.py utils/intermediateTer.py utils/CallJoinBPart1.py utils/intermediatebis.py utils/CallJoinBPart2.py > FreeCAD_Modules/CallJoinB$parametricbrick.py
		
		#Fill the parameter value needed in python script to be executed in FreeCAD
		echo "angle = $PreviousRotAngle" >> utils/intermediate.py
		echo Path="\"$CurrentDir/STL_Files/ParametricJoinPartA$parametricbrick.stl"\" >> utils/intermediate.py
		> FreeCAD_Modules/CallJoinA$parametricbrick.py
		#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartB python script which will be executed by FreeCAD to generate Paramteric Parts
		cat utils/Header.py utils/intermediateTer.py utils/CallJoinAPart1.py utils/intermediate.py utils/CallJoinAPart2.py > FreeCAD_Modules/CallJoinA$parametricbrick.py
		
	fi
	
	
#end of Read line loop, go back to the loop begining until the EOF of FILENAME.
done < $FILENAME

#show numbers of each Parametric Part, in Readme.txt file. we may add Parameters for each part in the future...
echo Readme file >> Readme.txt
echo Here we parse the someBody.txt file to get the values of each parametric part >> Readme.txt

#Test whether there are some passivewheel
if [ "$passivewheel" = "0" ]; then
	echo There is No passivewheel >> Readme.txt
else
	echo "There is/are $passivewheel passiveWheels" >> Readme.txt
fi

#Test whether there are some activewheg
if [ "$activewheg" = "0" ]; then
	echo There is No activewheg >> Readme.txt
else
	echo "There is/are $activewheg activeWHEG" >> Readme.txt
fi

#Test whether there are some parametricbrick
if [ "$parametricbrick" = "0" ]; then
	echo There is No parametricbrick >> Readme.txt
else
	echo "There is/are $parametricbrick parametricBRICK" >> Readme.txt
fi

#Test whether there are some activewheel
if [ "$activewheel" = "0" ]; then
	echo There is No activewheel >> Readme.txt
else
	echo "There is/are $activewheel activeWheels" >> Readme.txt
fi

echo Finish.
echo

