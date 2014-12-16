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

// uncomment to check memory usage
//#define CHECK_MEMORY

// uncomment to perform motor calibration test
//#define CALIBRATION

#include <Servo.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

//NeuralNetwork
#include "NeuralNetwork.h"

#ifdef CHECK_MEMORY
#include "MemoryFree.h"
#endif

#define PI 3.14159265358979323846


/* Define Neural network*/
NeuralNetwork network;
unsigned int nInputs, nOutputs, nHidden;
//const float *weights, *params;
//const unsigned int *types;
float networkInput[NB_INPUTS];
float networkOutputs[NB_OUTPUTS];
int nbLightSensors, nbTouchSensors;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//float accelX, accelY, accelZ;
//float gyroX, gyroY, gyroZ;
//float xAccelOffset = 0.0;
//float yAccelOffset = 0.0; 
//float zAccelOffset = 2500;//-700;//2500;
const float ACCEL_SCALE_FACTOR = (9.81/2100); //(9.81/16500.0);
const float GYRO_SCALE_FACTOR = (1.0/131.0) * (PI/180.0);


int16_t ax, ay, az, gx, gy, gz;
int16_t ax_, ay_, az_, gx_, gy_, gz_;

float xGyroOffset = 0;
float yGyroOffset = 0;
float zGyroOffset = 0;

const float ALPHA = 0.0;


/* Sensor pin allocation*/
const int lightSensorTab[4] = {A0,A1,A2,A3};
const int touchSensorTab[6] = {15,8,7,4};//{"YAW", "AUX1", "D7", "D4"};

/* double dimension Tab 
 * inputTab[i][0] is the value of the input port 
 * inputTab[i][1] is the type of the input : 0 for lightSensor, 1 for Touch sensor and 2 for Accelerometer and Gyroscope */
int inputTab[10][2]; 

/* Define Servos Motors*/
Servo myservo0,myservo1,myservo2,myservo3,myservo4,myservo5,myservo6,myservo7;
/* Servopin allocation*/
int outputTab[8]={9,10,5,6,11,13,16,14}; //{"D9", "D10", "D5", "D6", "D11", "D13", "ROLL", "PITCH"}
Servo myservo[8]={myservo0,myservo1,myservo2,myservo3,myservo4,myservo5,myservo6,myservo7};

float servoOffsets[NB_SERVOS_MOTORS] = {4.0,0.0};

float servoPosition, servoSpeed, lightInput;
const int threshold = 400;

/* Keep elapsed time */
unsigned long t = 0;



float gyroIntegrate = 0;

void setup() {

  #ifdef USE_SERIAL
  Serial.begin(9600);
  #endif
  #ifdef USE_SERIAL1
  Serial1.begin(115200);
  #endif
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  

  /* neural network initialization : */  
  nbLightSensors = NB_LIGHTSENSORS;
  nbTouchSensors = NB_TOUCH_SENSORS*2; //two inputs per touch_sensor

  nInputs = NB_INPUTS;//NB_LIGHTSENSORS+NB_TOUCH_SENSORS*2+NB_ACC_GYRO_SENSORS;//MAX_INPUT_NEURONS;
  nOutputs = NB_OUTPUTS;//NB_SERVOS_MOTORS;
  nHidden = NB_HIDDEN;
  //weights = EAWeight;
  //params = EAParams;
  //types = EATypes;
  
  initNetwork(&network, nInputs, nOutputs, nHidden, NULL, NULL, NULL);
  
  /* Feed inputTab according to the mapping of light sensors and touch sensors
  * We need to separate them because light sensors use analog inputs and touch sensors digital inputs
  */
  int sensorCounter = 0;
  int lightSensorCounter = 0;
  int TouchSensorCounter = 0;
  for(int i=0;i<NB_LIGHTSENSORS+NB_TOUCH_SENSORS*2+NB_ACC_GYRO_SENSORS ;i++)
  {
     if(input[i] == 0)//Type = lightSensor
     {  
       inputTab[i][0]=lightSensorTab[i-sensorCounter+lightSensorCounter];
       inputTab[i][1]=0; //Type = lightSensor
       lightSensorCounter++;
       sensorCounter++;
     }
     else if(input[i] == 1)//Type = touchSensor
     {
       inputTab[i][0]=touchSensorTab[i-sensorCounter+TouchSensorCounter]; //i-j because the index should begin to zero and i did not begin at zero at this step but at nbLightSensor, which has been added previously
       inputTab[i][1]=1; //Type = touchSensor
       TouchSensorCounter++;
       sensorCounter++;
     }
     else if(input[i] == 2)//Type = Accelerometer&Gyroscope
     {
       inputTab[i][1]=2; //Type = Accelerometer&Gyroscope
       sensorCounter++;
     }
  }
  
  /* Define sensors as input*/
  for(int i=0;i<NB_LIGHTSENSORS+NB_TOUCH_SENSORS*2+NB_ACC_GYRO_SENSORS;i++)
  {  
    if(inputTab[i][1] != 2)
      pinMode(inputTab[i][0], INPUT);
  }
  
  /* Assigns servos motors port and set initial command to activate full rotation motors */
  for(int i=0;i<nOutputs;i++)
  {
    myservo[i].attach(outputTab[i]);
    if(motor[i] == 1)
      myservo[i].write(0);
    else
      myservo[i].write(90 + servoOffsets[i]);    
  } 
  
  delay(5000);

 /* initialize acceleromoter and gyroscope */
  #ifdef USE_SERIAL
  Serial.println(F("Initializing I2C devices..."));
  #endif
  accelgyro.initialize();
  
  /* verify connection */
  bool connectionSuccess = accelgyro.testConnection();
  #ifdef USE_SERIAL
  Serial.println(F("Testing device connections..."));
  Serial.println(connectionSuccess ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  #endif

  //set full Scale sensor range to its lowest resolution
  accelgyro.setFullScaleGyroRange(0); // 3 = +/- 250 degrees/sec
  accelgyro.setFullScaleAccelRange(3); // 3 = +/- 16g
  #ifdef USE_SERIAL
  Serial.print(F(" Full Scale :  gyro_scale = "));
  Serial.print(accelgyro.getFullScaleGyroRange());
  Serial.print(F("\t  acc_scale = "));
  Serial.print(accelgyro.getFullScaleAccelRange());
  Serial.print(F("\n"));
  #endif
  
  //set to Low pass filter mode 3 for a max freq of 44Hz (refer uint8_t MPU6050::getDLPFMode() )
  #ifdef USE_SERIAL
  Serial.print(F(" DLPG : "));
  Serial.print(accelgyro.getDLPFMode());
  Serial.print(F("\n"));
  #endif
  
  accelgyro.setDLPFMode(6);
  
  #ifdef USE_SERIAL
  Serial.print(F(" DLPG : "));
  Serial.print(accelgyro.getDLPFMode());
  Serial.print(F("\n"));  
  #endif
  
  // use the code below to change accel/gyro offset values
  #ifdef USE_SERIAL
  Serial.println("Updating internal sensor offsets...");
  #endif
  // read raw accel/gyro measurements from device
  for(int i= 0; i <2 ; i++)
  {
    delay(50);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    #ifdef USE_SERIAL
    Serial.print(F("a/g:\t"));
    Serial.print(ax); Serial.print(F("\t"));
    Serial.print(ay); Serial.print(F("\t"));
    Serial.print(az); Serial.print(F("\t"));
    Serial.print(gx); Serial.print(F("\t"));
    Serial.print(gy); Serial.print(F("\t"));
    Serial.print(gz); Serial.print(F("\t"));
    Serial.print(F("\n"));
    Serial.print(accelgyro.getXAccelOffset()); Serial.print(F("\t")); 
    Serial.print(accelgyro.getYAccelOffset()); Serial.print(F("\t")); 
    Serial.print(accelgyro.getZAccelOffset()); Serial.print(F("\t")); 
    Serial.print(accelgyro.getXGyroOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getYGyroOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getZGyroOffset()); Serial.print(F("\t"));
    Serial.print(F("\n"));
    #endif
    
    accelgyro.setXAccelOffset((int16_t)(accelgyro.getXAccelOffset() - ax)); //-50
    accelgyro.setYAccelOffset((int16_t)(accelgyro.getYAccelOffset() - ay)); //30 
    accelgyro.setZAccelOffset((int16_t)(accelgyro.getZAccelOffset() - (az - 2100))); //300
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    
    xGyroOffset = - gx;
    yGyroOffset = - gy;
    zGyroOffset = - gz;
    
    
    
    #ifdef USE_SERIAL
    Serial.print(accelgyro.getXAccelOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getYAccelOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getZAccelOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getXGyroOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getYGyroOffset()); Serial.print(F("\t"));
    Serial.print(accelgyro.getZGyroOffset()); Serial.print(F("\t"));
    Serial.print(F("\n"));
    #endif
  }
  

  #ifdef USE_SERIAL
  #ifdef CALIBRATION
  
  delay(5000);
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

bool started=false;
int startTime;
void loop() {
  
  
  unsigned long elapsedTime = millis() - t;
  if ( (!started) || (elapsedTime >= ACTUATION_PERIOD)) {
    if(!started) {
      started = true;
      startTime = millis();
    }
    t = millis();
    
    /* read sensors */
    for(int i=0;i<nInputs;i++)
    {  
      if(inputTab[i][1]==0)//Type lightSensor
      {
        //To comply with the simulator we cast this sensor output into a float between 0.1 and 1. with 1 = maxLight
        lightInput = float(analogRead(inputTab[i][0]))/1000;
        //Serial.print(lightInput); Serial.print(F("\t"));  
        //you can set a certain threshold
        if(analogRead(inputTab[i][0]) > threshold) //you 
          networkInput[i] = lightInput;
        else
          networkInput[i] = 0.1;
      }
      else if(inputTab[i][1]==1)//Type touchSensor
        networkInput[i] = digitalRead(inputTab[i][0]);
      else if(inputTab[i][1]==2)//Type is accelerometer and gyroscope
      {
        accelgyro.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
        
        gyroIntegrate += ((gy_ + yGyroOffset) * GYRO_SCALE_FACTOR)*(elapsedTime)/1000;
        
        #ifdef USE_SERIAL
        Serial.print(F("integrated gyro z \t"));
        Serial.println(gyroIntegrate);
        #endif
        
        #ifdef USE_SERIAL1
        Serial1.print(gyroIntegrate); Serial1.print(F("\t"));
        #endif
        
        ax = ax * ALPHA + ax_ * (1-ALPHA);
        ay = ay * ALPHA + ay_ * (1-ALPHA);
        az = az * ALPHA + az_ * (1-ALPHA);
        gx = gx * ALPHA + gx_ * (1-ALPHA);
        gy = gy * ALPHA + gy_ * (1-ALPHA);
        gz = gz * ALPHA + gz_ * (1-ALPHA);
        
        
        //normalize accel values from 16500 to 9.81 (1g)
        //accelX = (float)(ax+xAccelOffset)*ACCEL_SCALE_FACTOR;
        //accelY = (float)(ay+yAccelOffset)*ACCEL_SCALE_FACTOR;
        //accelZ = (float)(az+zAccelOffset)*ACCEL_SCALE_FACTOR;

        //fill in neuralNetwork
        networkInput[i] = ax * ACCEL_SCALE_FACTOR; i++;  //0; i++; //
        networkInput[i] = ay * ACCEL_SCALE_FACTOR; i++; //0; i++; //
        networkInput[i] = az * ACCEL_SCALE_FACTOR; i++; //0; i++; //

        networkInput[i] = gx * GYRO_SCALE_FACTOR; i++;  
        networkInput[i] = gy * GYRO_SCALE_FACTOR; i++; 
        networkInput[i] = gz * GYRO_SCALE_FACTOR; 

        #ifdef USE_SERIAL
        //Serial.print(xAccelOffset); Serial.print(F("\t"));
        //Serial.print(yAccelOffset); Serial.print(F("\t"));
        //Serial.print(zAccelOffset); Serial.print(F("\t"));
        Serial.print(ax * ACCEL_SCALE_FACTOR); Serial.print(F("\t"));
        Serial.print(ay * ACCEL_SCALE_FACTOR); Serial.print(F("\t"));
        Serial.print(az * ACCEL_SCALE_FACTOR); Serial.print(F("\t"));
        Serial.print(gx * GYRO_SCALE_FACTOR); Serial.print(F("\t"));
        Serial.print(gy * GYRO_SCALE_FACTOR); Serial.print(F("\t")); 
        Serial.print(gz * GYRO_SCALE_FACTOR); Serial.print(F("\t")); 
        #endif
        
      }
    }
    #ifdef USE_SERIAL
    Serial.print(elapsedTime);
    Serial.print(F("\n"));
    #ifdef CHECK_MEMORY
      Serial.print(F("freeMemory()="));
      Serial.print(freeMemory()); Serial.print(F("\n"));    
    #endif
    #endif
    
    #ifdef USE_SERIAL1
    Serial1.print((millis() - startTime)); Serial1.print(F("\t"));
    for(int i=0;i<nInputs;i++)
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
    for(int i=0;i<nOutputs;i++)
    {
      if(motor[i] == 0) //servoMotors => set poisition 
      {
        servoPosition = networkOutputs[i]; //goes from 0 degrees to 180 degrees 
        myservo[i].write(servoPosition + servoOffsets[i]);
        #ifdef USE_SERIAL
        Serial.print(servoPosition); Serial.print(F("\t"));
        #endif
      }
      else if(motor[i] == 1)  // full rotation motors => set speed
      {
        if( networkOutputs[i] >= 90)//one rotation direction only => discard other rotation direction
          servoSpeed = networkOutputs[i] - 30; //we  
        else
          servoSpeed = 0;
          
        myservo[i].write(servoSpeed);
        #ifdef USE_SERIAL
        Serial.print(servoSpeed); Serial.print(F("\t"));
        #endif
      }
      
    }
    #ifdef USE_SERIAL
    Serial.print(F("\n"));
    #endif
    
    #ifdef USE_SERIAL1
    for(int i=0;i<nOutputs; i++) {
      Serial1.print(networkOutputs[i]); 
      Serial1.print((i==nOutputs-1)?F("\n"):F("\t"));
    }
    #endif
    
    //slow the main loop frequency, not to burn servoMotors too quickly
    //delay(200);  
  }
}

//hack for Arduino to be able to play with function that have struct as parameters.



void initNetwork(NeuralNetwork* network, unsigned int nInputs,
		unsigned int nOutputs, unsigned int nHidden,
		const float *weights, const float* params,
		const unsigned int *types) {

	unsigned int i = 0;
        
	network->nNonInputs = nOutputs + nHidden;

	/* Initialize states */
	for (i = 0; i < network->nNonInputs * 2; ++i) {
		network->state[i] = 0.0;
	}

	/* Initialize inputs */
	for (i = 0; i < nInputs; ++i) {
		network->input[i] = 0.0;
	}

	network->nInputs = nInputs;
	network->nOutputs = nOutputs;
	network->nHidden = nHidden;

	network->curStateStart = 0;
        

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
        unsigned int nextState;
	unsigned int i = 0;
	unsigned int j = 0;

	if (network->nOutputs == 0) {
		return;
	}

	/* For each output neuron, sum the state of all the incoming connection */
	nextState = (network->curStateStart + network->nNonInputs)
			% network->nNonInputs;

 
        


	for (i = 0; i < network->nNonInputs; ++i) { /*testing just updating state of outputs*/


		float curNeuronActivation = 0;
		unsigned int baseIndexOutputWeigths = -1;
                

		for (j = 0; j < network->nInputs; ++j) {
                        curNeuronActivation += getWeight(network->nNonInputs * j + i)
					* network->input[j];
		}
		baseIndexOutputWeigths = network->nNonInputs * network->nInputs;
		for (j = 0; j < network->nNonInputs; ++j) {
			
  
                    curNeuronActivation += getWeight(baseIndexOutputWeigths
					+ network->nNonInputs * j + i)
					* network->state[network->curStateStart + j];
		}

		/* Save next state */
		if (EATypes[i] == SIGMOID) {
			/* params are bias, gain */
			curNeuronActivation -= getParam(MAX_PARAMS*i);
			network->state[nextState + i] = 1.0
				/ (1.0 + exp(-getParam(MAX_PARAMS*i+1) *
						curNeuronActivation));
		} else if (EATypes[i] == SIMPLE) {
			/* linear, params are bias, gain */

			curNeuronActivation -= getParam(MAX_PARAMS*i);
			network->state[nextState + i] = getParam(MAX_PARAMS*i+1) *
					curNeuronActivation;

		} else if (EATypes[i] == OSCILLATOR) {
			/* TODO should this consider inputs too?? */
			/* params are period, phase offset, gain (amplitude) */


			float period = getParam(MAX_PARAMS*i);
			float phaseOffset = getParam(MAX_PARAMS*i + 1);
			float gain = getParam(MAX_PARAMS*i + 2);
			network->state[nextState + i] = ((sin( (2.0*PI/period) *
				 (time - period * phaseOffset))) + 1.0) / 2.0;

			/* set output to be in [0.5 - gain/2, 0.5 + gain/2] */
			network->state[nextState + i] = (0.5 - (gain/2.0) +
					network->state[nextState + i] * gain);

		}
	}
        
	network->curStateStart = nextState;
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
            output[i] = network->state[network->curStateStart + i] * 100 + 40;
	}

}
