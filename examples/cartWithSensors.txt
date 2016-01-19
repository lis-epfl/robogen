0 CoreComponent Core 0
	0 FixedBrick A1 0
		0 ActiveWheel Wheel1 0 0.04
		1 ParametricJoint J1 0 0.02 -45 0
			0 IrSensor S1 0
	1 FixedBrick A2 0
		2 ParametricJoint J2 0 0.02 45 0
			0 IrSensor S2 0
		0 ActiveWheel Wheel2 0 0.04
	2 FixedBrick Tail1 1
		0 FixedBrick Tail2 1
			0 FixedBrick Tail3 1
				1 PassiveWheel Wheel3 0 0.04
				2 PassiveWheel Wheel4 0 0.04
				0 IrSensor S4 0
	3 ParametricJoint J3 0 0.02 0 0
		0 IrSensor S3 0
