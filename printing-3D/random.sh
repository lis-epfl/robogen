#!/bin/bash
FILENAME="someBody.txt"

# read file, line by line, and get all of the elements.

while read f1 f2 f3 f4 f5 f6 f7 #read as many elements as there are - how ?
do
      #Counter number of passivewheels (W), activewhegs (G) and parametricbricks aka parametric bar joints (B), and activewheels(J)
	if [[ "$f1" == "" ]]; then
		break
	elif [[ "$f2" == "W" ]]; then
		let "passivewheel+=1"
		PreviousType="$f2"
		radius="$f5"
		echo "$f2 and radius = $radius"
	elif [[ "$f2" == "G" ]]; then
		let "activewheg+=1"
		PreviousType="$f2"
		radius="$f5"
		echo "$f2 and radius = $radius"		
	elif [[ "$f2" == "B" ]]; then
		let "parametricbrick+=1"
		PreviousType="$f2"
		len="$f5"
		inclAngle="$f6"	
		rotAngle="$f7"
		echo "$f2 and length = $len, inclination=$inclAngle, rot=$rotAngle"
	elif [[ "$f2" == "J" ]]; then
		let "activewheel+=1"
		PreviousType="$f2"
		radius="$f5"
		echo "$f2 and radius = $radius"	
	fi		

	
#end of Read line loop, go back to the loop begining until the EOF of FILENAME.
done < $FILENAME
