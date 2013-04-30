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
set /a activewheg=0
set /a parametricbrick=0

set CurrentDir=%CD%

::We need to store what was the previous Type and parameter that has been parsed from finalBestInd_GP.txt
set PreviousType=
set PreviousParam=

echo sys.path.append("%CurrentDir%\utils\Mod") > utils\intermediateTer.py

::loop for each Lines of the finalBestInd_GP.txt and get the two first part of that Line to be analysed in 'process' function
for /F "tokens=1,2" %%i in (finalBestInd_GP.txt) do call :process %%i %%j

::show numbers of each Parametric Part, in Readme.txt file. we may add Parameters for each part in the future...
echo Readme file >> Readme.txt
echo Here we parse the finalBestInd_GP.txt file to get the values of each parametric part >> Readme.txt

::Test whether there are some passivewheel
if "%passivewheel%"=="0" ( 
	echo There is No passivewheel >> Readme.txt
) else (
	echo There is/are %passivewheel% passiveWheels >> Readme.txt )

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

::Counter number of passivewheels, activewhegs and parametricbricks
if "%VAR2%"==""passivewheel"" (
	set /a passivewheel=%passivewheel%+1
	set PreviousType=%VAR2:~1,12%
) else if "%VAR2%"==""activewheg"" (
	set /a activewheg=%activewheg%+1
	set PreviousType=%VAR2:~1,10%
) else if "%VAR2%"==""parametricbrick"" (
	set /a parametricbrick=%parametricbrick%+1
	set PreviousType=%VAR2:~1,15%
)

::Set which parameter will be discribed there : radius, length, inclinationangle, rotationangle
if "%VAR1%"=="paramName:" (
	set PreviousParam=
	set PreviousParam=%VAR2:~1,6%
)

::Produce the scripts for each Parametric Part
if "%VAR1%"=="paramValue:" (
	if "%PreviousType%"=="passivewheel" (
		::Fill the parameter value needed in python script to be executed in FreeCAD
		copy /y nul "FreeCAD_Modules\CallWheel%passivewheel%.py"
		echo radiusExtern = %VAR2% > utils\intermediate.py
		echo Path="%CurrentDir%\STL_Files\PassiveWheel%passivewheel%.stl" >> utils\intermediate.py
		copy utils\Header.py+utils\intermediateTer.py+utils\CallWheelPart1.py+utils\intermediateTer.py+utils\intermediate.py+utils\CallWheelPart2.py FreeCAD_Modules\CallWheel%passivewheel%.py

	) else if "%PreviousType%"=="activewheg" (
		::Fill the parameter value needed in python script to be executed in FreeCAD
		copy /y nul "FreeCAD_Modules\CallWheg%activewheg%.py"
		::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallWheg python script which will be executed by FreeCAD to generate Paramteric Parts
		echo radiusExtern = %VAR2% > utils\intermediate.py
		echo Path="%CurrentDir%\STL_Files\ActiveWheg%activewheg%.stl" >> utils\intermediate.py
		copy utils\Header.py+utils\intermediateTer.py+utils\CallWhegPart1.py+utils\intermediate.py+utils\CallWhegPart2.py FreeCAD_Modules\CallWheg%activewheg%.py

	) else if "%PreviousType%"=="parametricbrick" (
		if "%PreviousParam%"=="length" (
		::Fill the parameter value needed in python script to be executed in FreeCAD
		echo heightJoin = %VAR2% > utils\intermediate.py 
		) else if "%PreviousParam%"=="inclin" (
		::Fill the parameter value needed in python script to be executed in FreeCAD
		echo angle = %VAR2% > utils\intermediatebis.py
		echo Path="%CurrentDir%\STL_Files\ParametricJoinPartB%parametricbrick%.stl" >> utils\intermediatebis.py
		copy /y nul "FreeCAD_Modules\CallJoinB%parametricbrick%.py"
		::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartA python script which will be executed by FreeCAD to generate Paramteric Parts
		copy utils\Header.py+utils\intermediateTer.py+utils\CallJoinBPart1.py+utils\intermediatebis.py+utils\CallJoinBPart2.py FreeCAD_Modules\CallJoinB%parametricbrick%.py
		) else if "%PreviousParam%"=="rotati" (
		::Fill the parameter value needed in python script to be executed in FreeCAD
		echo angle = %VAR2% >> utils\intermediate.py
		echo Path="%CurrentDir%\STL_Files\ParametricJoinPartA%parametricbrick%.stl" >> utils\intermediate.py
		copy /y nul "FreeCAD_Modules\CallJoinA%parametricbrick%.py"
		::concatenate CallWhegPART1.py+utils\intermediate.py+CallWhegPART2.py in one file : the CallParametricJointPartB python script which will be executed by FreeCAD to generate Paramteric Parts
		copy utils\Header.py+utils\intermediateTer.py+utils\CallJoinAPart1.py+utils\intermediate.py+utils\CallJoinAPart2.py FreeCAD_Modules\CallJoinA%parametricbrick%.py
		)
	)
)

goto :EOF
