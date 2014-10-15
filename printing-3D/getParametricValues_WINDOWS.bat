



@echo off

::Create 3 files. If they already exist, it will delete them and create new empty ones.
copy /y nul "utils\intermediate.py"
copy /y nul "utils\intermediatebis.py"
copy /y nul "utils\intermediateTer.py"
copy /y nul "Readme.txt"

::write a message so that user knows what is going on, until the end of the script.
echo please wait until the brain has been parsed


setlocal
::variables declaration
set /a passivewheel=0
set /a activewheel=0
set /a activewheg=0
set /a parametricbrick=0

set CurrentDir=%CD%

::We need to store what was the previous Type and parameter that has been parsed from finalBestInd_GP.txt
set PreviousType=
set PreviousRadius=
set PreviousLength=
set PreviousInclAngle=
set PreviousRotAngle=

echo sys.path.append("%CurrentDir%\utils\Mod") > utils\intermediateTer.py

::loop for each Lines of the someBody1.txt and get the two first part of that Line to be analysed in 'process' function
for /F "tokens=1-7" %%i in (someBody1.txt) do call :process %%i %%j %%k %%l %%m %%n %%o

::show numbers of each Parametric Part, in Readme.txt file. we may add Parameters for each part in the future...
echo Readme file >> Readme.txt
echo Here we parse the someBody1.txt file to get the values of each parametric part >> Readme.txt

::Test whether there are some passivewheel
if "%passivewheel%"=="0" ( 
	echo There is No passivewheel >> Readme.txt
) else (
	echo There is/are %passivewheel% passiveWheels >> Readme.txt )

::Test whether there are some activewheel
if "%passivewheel%"=="0" ( 
	echo There is No activewheel >> Readme.txt
) else (
	echo There is/are %activewheel% activeWheels >> Readme.txt )	
	
::Test whether there are some activewheg
if "%activewheg%"=="0" ( 
	echo There is No activewheg >> Readme.txt
) else (
	echo There is/are %activewheg% activeWHEG >> Readme.txt )

::Test whether there are some parametricbrick
if "%parametricbrick%"=="0" ( 
	echo There is No parametricbrick >> Readme.txt
) else (
	echo There is/are %parametricbrick% parametricBRICK >> Readme.txt )

endlocal

:process
set VAR1=%1
set VAR2=%2
set VAR5=%5
set VAR6=%6
set VAR7=%7

::Counter number of passivewheels, activewheels, activewhegs and parametricbricks
if "%VAR2%"=="PassiveWheel" (
	set /a passivewheel=%passivewheel%+1
	set PreviousType=%VAR2:~0,1%
	set PreviousRadius=%VAR5%
) else if "%VAR2%"=="ActiveWheel" (
	set /a activewheel=%activewheel%+1
	set PreviousType=%VAR2:~0,1%
	set PreviousRadius=%VAR5%
) else if "%VAR2%"=="ActiveWheg" (
	set /a activewheg=%activewheg%+1
	set PreviousType=%VAR2:~0,1%
	set PreviousRadius=%VAR5%
) else if "%VAR2%"=="ParametricJoint" (
	set /a parametricbrick=%parametricbrick%+1
	set PreviousType=%VAR2:~0,1%
	set PreviousLength=%VAR5%
	set PreviousInclAngle=%VAR6%
	set PreviousRotAngle=%VAR7%
)

::Produce the scripts for each Parametric Part

if "%PreviousType%"=="PassiveWheel" (
	::Fill the parameter value needed in python script to be executed in FreeCAD
	copy /y nul "FreeCAD_Modules\CallPassiveWheel%passivewheel%.py"
	echo radiusExtern = %PreviousRadius% > utils\intermediate.py
	echo Path="%CurrentDir%\..\PassiveWheel%passivewheel%.stl" >> utils\intermediate.py
	copy utils\Header.py+utils\intermediateTer.py+utils\CallWheelPart1.py+utils\intermediate.py+utils\CallWheelPart2.py FreeCAD_Modules\CallPassiveWheel%passivewheel%.py

) else if "%PreviousType%"=="ActiveWheel" (
	::Fill the parameter value needed in python script to be executed in FreeCAD
	copy /y nul "FreeCAD_Modules\CallActiveWheel%activewheel%.py"
	echo radiusExtern = %PreviousRadius% > utils\intermediate.py
	echo Path="%CurrentDir%\..\ActiveWheel%activewheel%.stl" >> utils\intermediate.py
	copy utils\Header.py+utils\intermediateTer.py+utils\CallWheelPart1.py+utils\intermediate.py+utils\CallWheelPart2.py FreeCAD_Modules\CallActiveWheel%activewheel%.py

) else if "%PreviousType%"=="ActiveWheg" (
	::Fill the parameter value needed in python script to be executed in FreeCAD
	copy /y nul "FreeCAD_Modules\CallWheg%activewheg%.py"
	::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallWheg python script which will be executed by FreeCAD to generate Paramteric Parts
	echo radiusExtern = %PreviousRadius% > utils\intermediate.py
	echo Path="%CurrentDir%\..\ActiveWheg%activewheg%.stl" >> utils\intermediate.py
	copy utils\Header.py+utils\intermediateTer.py+utils\CallWhegPart1.py+utils\intermediate.py+utils\CallWhegPart2.py FreeCAD_Modules\CallWheg%activewheg%.py

) else if "%PreviousType%"=="ParametricJoint" (
	::Fill the parameter value needed in python script to be executed in FreeCAD
	echo heightJoin = %PreviousLength% > utils\intermediate.py 
	::Fill the parameter value needed in python script to be executed in FreeCAD
	echo inclAngle = %PreviousInclAngle% > utils\intermediatebis.py
	echo Path="%CurrentDir%\..\ParametricJoinPartB%parametricbrick%.stl" >> utils\intermediatebis.py
	copy /y nul "FreeCAD_Modules\CallJoinB%parametricbrick%.py"
	::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartA python script which will be executed by FreeCAD to generate Paramteric Parts
	copy utils\Header.py+utils\intermediateTer.py+utils\CallJoinBPart1.py+utils\intermediatebis.py+utils\CallJoinBPart2.py FreeCAD_Modules\CallJoinB%parametricbrick%.py
	::Fill the parameter value needed in python script to be executed in FreeCAD
	echo angle = %PreviousRotAngle% >> utils\intermediate.py
	echo Path="%CurrentDir%\..\ParametricJoinPartA%parametricbrick%.stl" >> utils\intermediate.py
	copy /y nul "FreeCAD_Modules\CallJoinA%parametricbrick%.py"
	::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartB python script which will be executed by FreeCAD to generate Paramteric Parts
	copy utils\Header.py+utils\intermediateTer.py+utils\CallJoinAPart1.py+utils\intermediate.py+utils\CallJoinAPart2.py FreeCAD_Modules\CallJoinA%parametricbrick%.py
)


goto :EOF
