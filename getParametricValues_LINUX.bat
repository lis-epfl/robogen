#!/bin/bash
#Author Gregoire Heitz <gregoire.heitz@epfl.ch>
#Goal : Parsed the finalBestInd_GP.txt file to get the parametric Part Parameters
# For the robogen course project, LIS EPFL.

# Create those 3 empty files, even if they alrady existed
> utils/intermediate.h
> utils/intermediatebis.h
> Readme.txt

#User information on what is going on
echo please wait until the body has been parsed

#global variables declaration
let "passivewheel=0"
let "activewheg=0"
let "parametricbrick=0"

CurrentDir=$PWD

PreviousType=
PreviousParam=

#file name to be parsed.
FILENAME="finalBestInd_GP.txt"

#show numbers of each Parametric Part, in Readme.txt file. we may add Parameters for each part in the future...
echo Readme file >> Readme.txt
echo Here we parse the finalBestInd_GP.txt file to get the values of each parametric part >> Readme.txt

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

#read file, line by line, and get the first two elements.
while read f1 f2
do
      #Counter number of passivewheels, activewhegs and parametricbricks
	if [[ "$f2" == *"passivewheel"* ]]; then
		let "passivewheel+=1"
		PreviousType="$f2"
	elif [[ "$f2" == *"activewheg"* ]]; then
		let "activewheg+=1"
		PreviousType="$f2"
	elif [[ "$f2" == *"parametricbrick"* ]]; then
		let "parametricbrick+=1"
		PreviousType="$f2"
	fi

	#Set which parameter will be discribed there : radius, length, inclinationangle, rotationangle
	if [ "$f1" = "paramName:" ]; then
		PreviousParam="$f2"
	fi

	#Produce the scripts for each Parametric Part
	if [ "$f1" = "paramValue:" ]; then
		if [[ "$PreviousType" == *"passivewheel"* ]]; then
			#Fill the parameter value needed in python script to be executed in FreeCAD
			echo "radiusExtern = $f2" > utils/intermediate.py
			echo Path="\"$CurrentDir/STL_Files/PassiveWheel$passivewheel.stl"\" >> utils/intermediate.py
			> FreeCAD_Modules/CallWheel$passivewheel.py
			#concatenate CallWheelPART1.py+utils/intermediate.py+CallWheelPART2.py in one file : the CallWheel python script which will be executed by FreeCAD to generate Paramteric Parts
			cat utils/CallWheelPart1.py utils/intermediate.py utils/CallWheelPart2.py > FreeCAD_Modules/CallWheel$passivewheel.py

		elif [[ "$PreviousType" == *"activewheg"* ]]; then
			#Fill the parameter value needed in python script to be executed in FreeCAD
			echo "radiusExtern = $f2" > utils/intermediate.py
			echo Path="\"$CurrentDir/STL_Files/ActiveWheg$activewheg.stl"\" >> utils/intermediate.py
			> FreeCAD_Modules/CallWheg$activewheg.py
			#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallWheg python script which will be executed by FreeCAD to generate Paramteric Parts
			cat utils/CallWhegPart1.py utils/intermediate.py utils/CallWhegPart2.py > FreeCAD_Modules/CallWheg$activewheg.py

		elif [[ "$PreviousType" == *"parametricbrick"* ]]; then
			if [[ "$PreviousParam" == *"length"* ]]; then
				#Fill the parameter value needed in python script to be executed in FreeCAD
				echo "heightJoin = $f2" > utils/intermediate.py 
			elif [[ "$PreviousParam" == *"inclinationangle"* ]]; then
				#Fill the parameter value needed in python script to be executed in FreeCAD
				echo "angle = $f2" > utils/intermediatebis.py
				echo Path="\"$CurrentDir/STL_Files/ParametricJoinPartB$parametricbrick.stl"\" >> utils/intermediatebis.py
				> FreeCAD_Modules/CallJoinB$parametricbrick.py
				#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartA python script which will be executed by FreeCAD to generate Paramteric Parts
				cat utils/CallJoinBPart1.py utils/intermediatebis.py utils/CallJoinBPart2.py > FreeCAD_Modules/CallJoinB$parametricbrick.py
			elif [[ "$PreviousParam" == *"rotationangle"* ]]; then
				#Fill the parameter value needed in python script to be executed in FreeCAD
				echo "angle = $f2" >> utils/intermediate.py
				echo Path="\"$CurrentDir/STL_Files/ParametricJoinPartA$parametricbrick.stl"\" >> utils/intermediate.py
				> FreeCAD_Modules/CallJoinA$parametricbrick.py
				#concatenate CallWhegPART1.py+utils/intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartB python script which will be executed by FreeCAD to generate Paramteric Parts
				cat utils/CallJoinAPart1.py utils/intermediate.py utils/CallJoinAPart2.py > FreeCAD_Modules/CallJoinA$parametricbrick.py
			fi
		fi
	fi
	
#end of Read line loop, go back to the loop begining until the EOF of FILENAME.
done < $FILENAME

echo Finish.
echo

