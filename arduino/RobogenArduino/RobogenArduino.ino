/*
 * @(#) RobogenArduino.ino  1.0   Sept 14, 2013
 *
 * Grégoire Heitz (gregoire.heitz@epfl.ch),
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Grégoire Heitz, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2011-10-07 - initial release
// 2015-11-06 - modified by Alice for the IR sensors

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
 */


// comment out to prevent serial communication
#define USE_SERIAL
#define USE_SERIAL1

//#define SENSOR_NOISE_LEVEL 0.1
//#define MOTOR_NOISE_LEVEL 0.0

// uncomment to check memory usage
//#define CHECK_MEMORY

// uncomment to perform motor calibration test
//#define CALIBRATE_SERVOS

// uncomment to perform imu calibration
//#define CALIBRATE_SENSOR_OFFSET

// put in values for your Arduino after doing calibration
#define ACCEL_OFFSET_X (1353)
#define ACCEL_OFFSET_Y (895)
#define ACCEL_OFFSET_Z (1596)
#define GYRO_OFFSET_X (138)
#define GYRO_OFFSET_Y (-110)
#define GYRO_OFFSET_Z (133)

#define PI 3.14159265358979323846f

#define LIGHT_SENSOR_THRESHOLD (0)

#include <Servo.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#define D9 (9)
#define D10 (10)
#define D5 (5)
#define D6 (6)
#define D11 (11)
#define D13 (13)
#define ROLL (16)
#define PITCH (14)
#define YAW (15)
#define AUX1 (8)
#define D7 (7)
#define D4 (4)

#define NONE (-1)

//NeuralNetwork
#include "NeuralNetwork.h"

#ifdef CHECK_MEMORY
#include "MemoryFree.h"
#endif

#include "IMU.h"
#include "quaternions.h"

// Include the Library for the IR sensors
#include "SparkFun_VL6180X.h"

/* Define Neural network*/
NeuralNetwork network;
float networkInput[NB_INPUTS];
float networkOutputs[NB_OUTPUTS];

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelGyro;

/* Define Servos Motors*/
Servo myservo0,myservo1,myservo2,myservo3,myservo4,myservo5,myservo6,myservo7;
Servo myservo[]={myservo0,myservo1,myservo2,myservo3,myservo4,myservo5,myservo6,myservo7};
float servoOffsets[] = {0,5,3,3,0,0,0,0};

float servoPosition, servoSpeed;
int lightInput;

/* Define IR sensors */
//To begin with, they all have the same address then it will be changed in setup()
VL6180x sensor0(0x29),sensor1(0x29),sensor2(0x29),sensor3(0x29),sensor4(0x29),sensor5(0x29),sensor6(0x29),sensor7(0x29);
VL6180x irSensors[]={sensor0,sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7};
int sensorAdresses[] = {0x28,0x27,0x26,0x25,0x24,0x23,0x22,0x21};


/* Keep elapsed time */
unsigned long t = 0;


quat_t q_rot;
IMU imu;

bool started=false;
int startTime;

void setup() {

	started = false;
	t = 0;

#ifdef USE_SERIAL
	Serial.begin(9600);
#endif
#ifdef USE_SERIAL1
	Serial1.begin(115200);
#endif
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();


	/* neural network initialization : */
	initNetwork(&network, NB_INPUTS, NB_OUTPUTS, NB_HIDDEN, NULL, NULL, NULL);

	/* Define light and touch sensor pins as input*/
	for(int i=0;i<NB_INPUTS;i++)
	{
		// Initialize pin for all sensors except IMU and IR (which has no pin)
		if(inputTab[i][1] != 2 && inputTab[i][1] != 3)
			pinMode(inputTab[i][0], INPUT);
	}

	/* Define the IR sensor pins as output*/

	for(int i=0;i<NB_INPUTS;i++)
	{
		// The pins are used to enable/disable the IR sensor for initialization. They are set to OUTPUT.
		if(inputTab[i][1] == 3)
		{
			pinMode(inputTab[i][0], OUTPUT);
			digitalWrite(inputTab[i][0], LOW);//i++;
#ifdef USE_SERIAL
			Serial.print("LOW ");
			Serial.println(irIndices[i]);
#endif
		}
	}

	delay(10);

	/* Initialize IR sensors : */
	// Set each one to HIGH in turn and change the address.
	// WARNING!!!!
	// Do not put a sensor pin back to LOW, this will reinitialize
	// it and return the address to default 0x29
	for(int i=0;i<NB_INPUTS;i++)
	{
		if(inputTab[i][1] == 3)
		{
#ifdef USE_SERIAL
			Serial.print("Init IR sensor ");
			Serial.println(irIndices[i]);
#endif
			for(int j=0;j<NB_INPUTS;j++){
				if(i==j){
					digitalWrite(inputTab[j][0],HIGH);
#ifdef USE_SERIAL
					Serial.print("HIGH ");
					Serial.println(irIndices[j]);
#endif
				}
			}
			delay(10); // This is required to let the chip realise that it is enabled !!
#ifdef USE_SERIAL
			if(irSensors[irIndices[i]].VL6180xInit() != 0)
			{
				Serial.print("FAILED TO INITIALIZE SENSOR "); //Initialize device and check for errors
				Serial.println(irIndices[i]);
			}
			else
			{
				Serial.print("INITIALIZED SENSOR ");
				Serial.println(irIndices[i]);
			}
#endif
#ifdef USE_SERIAL
			Serial.println(irSensors[irIndices[i]]._i2caddress);
#endif
			irSensors[irIndices[i]].changeAddress(0x29,sensorAdresses[irIndices[i]]);
#ifdef USE_SERIAL
			Serial.println(irSensors[irIndices[i]]._i2caddress);
#endif
			irSensors[irIndices[i]].VL6180xDefautSettings(); //Load default settings to get started.

		}
	}

	//All sensors are on HIGH. Readjust a few parameters.
	for(int i=0;i<NB_INPUTS;i++)
	{
		if(inputTab[i][1] == 3)
		{
			irSensors[irIndices[i]].VL6180xReadjustParameters();//i++; // Readjust the parameters of the sensors
		}
	}

	/* Assigns servos motors port and set initial command to activate full rotation motors */
	for(int i=0;i<NB_OUTPUTS;i++)
	{
		if (outputTab[i][1] == 0) {
			myservo[i].attach(outputTab[i][0]);
			myservo[i].write(90 + servoOffsets[i]);
		}
	}

	delay(4000);

	/* initialize acceleromoter and gyroscope */
#ifdef USE_SERIAL
	Serial.println(F("Initializing I2C devices..."));
#endif
	accelGyro.initialize();

	/* verify connection */
	bool connectionSuccess = accelGyro.testConnection();
#ifdef USE_SERIAL
	Serial.println(F("Testing device connections..."));
	Serial.println(connectionSuccess ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

	//set full Scale sensor range to its lowest resolution
	accelGyro.setFullScaleGyroRange(0); // 3 = +/- 250 degrees/sec
	accelGyro.setFullScaleAccelRange(3); // 3 = +/- 16g
#ifdef USE_SERIAL
	Serial.print(F("Full Scale :  gyro_scale = "));
	Serial.print(accelGyro.getFullScaleGyroRange());
	Serial.print(F("\t  acc_scale = "));
	Serial.print(accelGyro.getFullScaleAccelRange());
	Serial.print(F("\n"));
#endif

	//set to Low pass filter mode 3 for a max freq of 44Hz (refer uint8_t MPU6050::getDLPFMode() )
#ifdef USE_SERIAL
	Serial.print(F("DLPG : "));
	Serial.print(accelGyro.getDLPFMode());
	Serial.print(F("\n"));
#endif

	accelGyro.setDLPFMode(6);

#ifdef USE_SERIAL
	Serial.print(F("DLPG : "));
	Serial.print(accelGyro.getDLPFMode());
	Serial.print(F("\n"));
#endif


	// use the code below to change accel/gyro offset values
#ifdef USE_SERIAL
	Serial.println(F("Updating internal sensor offsets..."));
#endif
	// If you want to calibrate the accelero offset, ensure the Arduino board in perfectly leveled (=FLAT)
#ifdef CALIBRATE_SENSOR_OFFSET
	// read raw accel/gyro measurements from device


	for(int i= 0; i <100 ; i++)
	{
		delay(50);
		imu.update(&accelGyro);
		accelGyro.setXAccelOffset((int16_t)(accelGyro.getXAccelOffset() - imu.rawAccel[0]));
		accelGyro.setYAccelOffset((int16_t)(accelGyro.getYAccelOffset() - imu.rawAccel[1]));
		accelGyro.setZAccelOffset((int16_t)(accelGyro.getZAccelOffset() - (imu.rawAccel[2] - 2100)));

		imu.gyroOffset[0] = -imu.rawGyro[0];
		imu.gyroOffset[1] = -imu.rawGyro[1];
		imu.gyroOffset[2] = -imu.rawGyro[2];



#ifdef USE_SERIAL
delay(50);
imu.update(&accelGyro);
Serial.print(F("Accelero offsets\n"));
Serial.print(accelGyro.getXAccelOffset()); Serial.print(F("\t"));
Serial.print(accelGyro.getYAccelOffset()); Serial.print(F("\t"));
Serial.print(accelGyro.getZAccelOffset()); Serial.print(F("\n"));
Serial.println(F("Accelero values, should be close to (0, 0, 0)"));
Serial.print(imu.rawAccel[0]); Serial.print(F("\t")); //should be close to 0
Serial.print(imu.rawAccel[1]); Serial.print(F("\t")); //should be close to 0
Serial.print(imu.rawAccel[2] - 2100); Serial.print(F("\n")); //should be close to 0


Serial.print(F("Gyro offsets\n"));
Serial.print(imu.gyroOffset[0]); Serial.print(F("\t"));
Serial.print(imu.gyroOffset[1]); Serial.print(F("\t"));
Serial.print(imu.gyroOffset[2]); Serial.print(F("\n"));

Serial.println(F("Gyro values, should be close to (0, 0, 0)"));
Serial.print(imu.rawGyro[0] + imu.gyroOffset[0]); Serial.print(F("\t")); //should be close to 0
Serial.print(imu.rawGyro[1] + imu.gyroOffset[1]); Serial.print(F("\t")); //should be close to 0
Serial.print(imu.rawGyro[2] + imu.gyroOffset[2]); Serial.print(F("\n")); //should be close to 0

#endif
	}
#else
	// set these macros above to proper values
	accelGyro.setXAccelOffset((int16_t)(ACCEL_OFFSET_X));
	accelGyro.setYAccelOffset((int16_t)(ACCEL_OFFSET_Y));
	accelGyro.setZAccelOffset((int16_t)(ACCEL_OFFSET_Z));

	imu.gyroOffset[0] = (int16_t)(GYRO_OFFSET_X);
	imu.gyroOffset[1] = (int16_t)(GYRO_OFFSET_Y);
	imu.gyroOffset[2] = (int16_t)(GYRO_OFFSET_Z);
#endif

//update imu in order to get rotation of gravity vector
	for(int i=0; i< 20; i++)
	{
		delay(50);

		//update imu
		imu.update(&accelGyro);
	}
	float up[3] = {0.0, 0.0, 1.0};
	float out[3];
	vectors_cross_product(imu.scaledAccel,up,out);
	//initial rotation, to relevel imu
	q_rot.s = sqrt((vectors_norm_sqr(imu.scaledAccel)) * (vectors_norm_sqr(up))) + vectors_scalar_product(imu.scaledAccel, up);
	q_rot.v[0] = out[0];
	q_rot.v[1] = out[1];
	q_rot.v[2] = out[2];
	q_rot = quaternions_normalise(q_rot);


#ifdef USE_SERIAL
#ifdef CALIBRATE_SERVOS

	Serial.print(F("\n"));
	Serial.print(F("\n"));
	Serial.print(F("\n"));
	servoOffsets[0] = -10;
	while(servoOffsets[0] < 10) {
		delay(500);
		myservo[0].write(90 + servoOffsets[0]);

		Serial.print(servoOffsets[0]);
		Serial.print(F("\n"));
		servoOffsets[0] += 0.2;

	}
#endif
#endif


}

double rand01()
{
	return random(0,2147483647L)/double(2147483647L);
}

double randn (double mu, double sigma)
{
	double U1, U2, W, mult;

	do
	{
		U1 = -1 + rand01() * 2;
		U2 = -1 + rand01() * 2;
		W = U1 * U1 + U2 * U2;
	}
	while (W >= 1 || W == 0);

	mult = sqrt ((-2 * log (W)) / W);
	double X1 = U1 * mult;

	return (mu + sigma * (double) X1);
}

void loop() {


	unsigned long elapsedTime = millis() - t;
	if ( (!started) || (elapsedTime >= ACTUATION_PERIOD)) {
		if(!started) {
			started = true;
			startTime = millis();
		}
		t = millis();

		/* launch IR sensor measure */
		for(int i=0;i<NB_INPUTS;i++)
		{
			if(inputTab[i][1] == 3)
			{
				irSensors[irIndices[i]].setSingleRange();
			}
		}

		/* read sensors */
		for(int i=0;i<NB_INPUTS;i++)
		{
			if(inputTab[i][1]==0)//Type lightSensor
			{
				//To comply with the simulator we cast this sensor output into a float between 0.0 and 1. with 1 = maxLight
				analogRead(inputTab[i][0]);
				//In order to properly read value need to delay 1 ms and read again
				delay(1);
				lightInput = analogRead(inputTab[i][0]);

				//you can set a certain threshold
				if(lightInput > (LIGHT_SENSOR_THRESHOLD))
					networkInput[i] = float(lightInput)/1000.0;
				else
					networkInput[i] = 0.0;
			}
			else if(inputTab[i][1]==1) { // Type touchSensor
				networkInput[i] = !digitalRead(inputTab[i][0]);
			}
			else if(inputTab[i][1]==2) // Type is accelerometer and gyroscope
			{
				//update imu, scaling and low pass filtering
				imu.update(&accelGyro);
				quat_t temp = quaternions_create_from_vector(imu.scaledAccel);
				float norm = vectors_norm(imu.scaledAccel);
				temp = quaternions_multiply(quaternions_multiply(q_rot, temp),quaternions_inverse(q_rot));
				temp = quaternions_normalise(temp);
				temp.v[0] *= norm;
				temp.v[1] *= norm;
				temp.v[2] *= norm;

				//fill in neuralNetwork
				networkInput[i] = temp.v[0]; i++;
				networkInput[i] = temp.v[1]; i++;
				networkInput[i] = temp.v[2]; i++;

				networkInput[i] = imu.scaledGyro[0]; i++;
				networkInput[i] = imu.scaledGyro[1]; i++;
				networkInput[i] = imu.scaledGyro[2];
			}
			else if(inputTab[i][1]==3) // Type is IR sensor
				//
			{
				//irSensors[irIndices[i]].setSingleShot();
				int distance = irSensors[irIndices[i]].getDistance2();
				networkInput[i] = 1.0 - (distance/255.0); //i++;
				//networkInput[i] = 0.0; // give light value in [0,1]
			}
		}
#ifdef USE_SERIAL
		Serial.println();
#endif

		// add noise
#ifdef SENSOR_NOISE_LEVEL
		// Add sensor noise: Gaussian with std dev of
		// SENSOR_NOISE_LEVEL * actualValue
		for(int i=0;i<NB_INPUTS;i++) {
			networkInput[i] += (randn(0,1) * SENSOR_NOISE_LEVEL * networkInput[i]);
		}
#endif

#ifdef USE_SERIAL
		for(int j=0; j<NB_INPUTS; j++) {
			Serial.print(networkInput[j]); Serial.print(F("\t"));
		}
		Serial.print(elapsedTime);
		Serial.print(F("\n"));
#ifdef CHECK_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.print(freeMemory()); Serial.print(F("\n"));
#endif
#endif



#ifdef USE_SERIAL1
		Serial1.print((millis() - startTime)); Serial1.print(F("\t"));
		for(int i=0;i<NB_INPUTS;i++)
		{
			Serial1.print(networkInput[i]);
			Serial1.print(F("\t"));
		}
#endif




		/* Feed neural network with sensors values as input of the neural network*/
		feed(&network, &networkInput[0]);

		/* Step the neural network*/
		step(&network, (millis() - startTime)/1000.0);

		/* Fetch the neural network ouputs */
		fetch(&network, &networkOutputs[0]);

		/* set Servos Motors according to the Fetch of the neural network outputs above*/
		for(int i=0;i<NB_OUTPUTS;i++)
		{
			if(outputTab[i][1] == 0) //servoMotors => set poisition
			{
				servoPosition = networkOutputs[i]  * 100 + 40;; //goes from 40 degrees to 140 degrees
				myservo[i].write(servoPosition + servoOffsets[i]);
#ifdef USE_SERIAL
				Serial.print(servoPosition); Serial.print(F("\t"));
#endif
			}
			else if(outputTab[i][1] == 1)  // full rotation motors => set speed
			{
				//if( networkOutputs[i] >= 90)//one rotation direction only => discard other rotation direction
				//  servoSpeed = networkOutputs[i] - 30; //we
				//else
				//  servoSpeed = 0;
				servoSpeed = networkOutputs[i] * 255;
				//myservo[i].write(servoSpeed);
#ifdef USE_SERIAL
				Serial.print(servoSpeed); Serial.print(F("\t"));
#endif


				analogWrite(outputTab[i][0], servoSpeed);

			}

		}
#ifdef NEUTRAL_PIN
		analogWrite(NEUTRAL_PIN, 127.5); // neutre
#endif

#ifdef USE_SERIAL
		Serial.print(F("\n"));
#endif

#ifdef USE_SERIAL1
		for(int i=0;i<NB_OUTPUTS; i++) {
			Serial1.print(networkOutputs[i]);
			Serial1.print((i==NB_OUTPUTS-1)?F("\n"):F("\t"));
		}
#endif

		//slow the main loop frequency, not to burn servoMotors too quickly
		//delay(200);
	}
}

//hack for Arduino to be able to play with function that have struct as parameters.



void initNetwork(NeuralNetwork* network, unsigned int nInputs, unsigned int nOutputs, 
                  unsigned int nHidden, const float *weights, const float* params,
                  const unsigned int *types) {

	unsigned int i = 0;

	network->nNonInputs = nOutputs + nHidden;

	/* Initialize states */
	for (i = 0; i < network->nNonInputs; ++i) {
		network->state[i] = 0.0;
	}

	/* Initialize inputs */
	for (i = 0; i < nInputs; ++i) {
		network->input[i] = 0.0;
	}

	network->nInputs = nInputs;
	network->nOutputs = nOutputs;
	network->nHidden = nHidden;


}

void feed(NeuralNetwork* network, const float *input) {

	unsigned int i = 0;
	for (i = 0; i < network->nInputs; ++i) {
		network->input[i] = input[i];
	}

}

float getWeight(int index) {
	return pgm_read_float_near(EAWeight + index);
	//return EAWeight[index];
}

float getParam(int index) {
	return pgm_read_float_near(EAParams + index);
	//return EAParams[index];
}

void step(NeuralNetwork* network, float time) {

	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int baseIndexOutputWeigths =
			network->nNonInputs * network->nInputs;

	if (network->nOutputs == 0) {
		return;
	}

	/* For each hidden and output neuron, sum the state of all the incoming connection */

	for (i = 0; i < network->nNonInputs; ++i) {

		network->activations[i] = 0;

		for (j = 0; j < network->nInputs; ++j) {
			network->activations[i] += getWeight(network->nNonInputs * j + i)
							* network->input[j];
		}

		for (j = 0; j < network->nNonInputs; ++j) {
			network->activations[i] += getWeight(baseIndexOutputWeigths
					+ network->nNonInputs * j + i)
					* network->state[j];
		}
	}

	/* Now add in biases and calculate new network state (or do appropriate operation for neuron type*/
	for (i = 0; i < network->nNonInputs; ++i) {

		/* Save next state */
		if (EATypes[i] == SIGMOID) {
			/* params are bias, gain */
			network->activations[i] -= getParam(MAX_PARAMS*i);
			network->state[i] = 1.0
					/ (1.0 + exp(-getParam(MAX_PARAMS*i+1) *
							network->activations[i]));
		} else if (EATypes[i] == SIMPLE) {
			/* linear, params are bias, gain */

			network->activations[i] -= getParam(MAX_PARAMS*i);
			network->state[i] = getParam(MAX_PARAMS*i+1) *
					network->activations[i];

		} else if (EATypes[i] == OSCILLATOR) {
			/* TODO should this consider inputs too?? */
			/* params are period, phase offset, gain (amplitude) */


			float period = getParam(MAX_PARAMS*i);
			float phaseOffset = getParam(MAX_PARAMS*i + 1);
			float gain = getParam(MAX_PARAMS*i + 2);
			network->state[i] = ((sin( (2.0*PI/period) *
					(time - period * phaseOffset))) + 1.0) / 2.0;

			/* set output to be in [0.5 - gain/2, 0.5 + gain/2] */
			network->state[i] = (0.5 - (gain/2.0) +
					network->state[i] * gain);

		}
	}

}

/**
 * Read the output of the neural network
 * @param network the neural network
 * @param output the output of the neural network, must point to an area of memory of at least size n
 * the output is the network state multiplied by 180 to map a 0 to 1 command into a 0° to 180° servo command
 */
void fetch(const NeuralNetwork* network, float *output) {

	unsigned int i = 0;
	for (i = 0; i < network->nOutputs; ++i) {
		float rawOutput = network->state[i];
#ifdef MOTOR_NOISE_LEVEL
		// Add motor noise:
		// uniform in range +/- MOTOR_NOISE_LEVEL * actualValue
		rawOutput += ( ((rand01() * 2.0 * MOTOR_NOISE_LEVEL) - MOTOR_NOISE_LEVEL) * rawOutput);
#endif
		output[i] = rawOutput;
	}

}

