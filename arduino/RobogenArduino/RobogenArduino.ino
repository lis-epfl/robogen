//Greg first Step with arduino programming

#include <Servo.h>

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

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

//NeuralNetwork
#include "NeuralNetwork.h"

/* Define Neural network*/
NeuralNetwork network;
unsigned int nInputs, nOutputs;
const float *weights, *bias, *gain;
float networkInput[MAX_INPUT_NEURONS];
float networkOutputs[MAX_OUTPUT_NEURONS];
int nbLightSensors, nbTouchSensors;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
float accelX, accelY, accelZ;
float xAccelOffset = 0.0;
float yAccelOffset = 0.0; 
float zAccelOffset = 2500;
float accelScaleFactor = (9.81/16500.0);

int16_t ax, ay, az;
int16_t gx, gy, gz;

/* Sensor pin allocation*/
int lightSensorTab[4] = {A0,A1,A2,A3};
int touchSensorTab[6] = {16,15,14,8,7,4};

/* double dimension Tab 
 * inputTab[i][0] is the value of the input port 
 * inputTab[i][1] is the type of the input : 0 for lightSensor, 1 for Touch sensor and 2 for Accelerometer and Gyroscope */
int inputTab[10][2]; 

/* Define Servos Motors*/
Servo myservo0,myservo1,myservo2,myservo3,myservo4,myservo5;
/* Servopin allocation*/
int outputTab[6]={9,10,5,6,11,13};
Servo myservo[6]={myservo0,myservo1,myservo2,myservo3,myservo4,myservo5};
int initMotorValue = 0;

float servoPosition, servoSpeed, lightInput;
int threshold = 400;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  /* neural network initialization : */  
  nbLightSensors = NB_LIGHTSENSORS;
  nbTouchSensors = NB_TOUCH_SENSORS*2; //two inputs per touch_sensor
  nInputs = NB_LIGHTSENSORS+NB_TOUCH_SENSORS*2+NB_ACC_GYRO_SENSORS;//MAX_INPUT_NEURONS;
  nOutputs = NB_SERVOS_MOTORS;
  weights = EAWeight;
  bias = EABiasWeight;
  gain = EAGain;
  initNetwork(&network, nInputs, nOutputs, weights, bias, gain);
  
 /* initialize acceleromoter and gyroscope */
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  /* verify connection */
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  /*accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(2500);
  
  xAccelOffset = accelgyro.getXAccelOffset();
  yAccelOffset = accelgyro.getYAccelOffset();
  zAccelOffset = accelgyro.getZAccelOffset();
  */
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
      myservo[i].write(initMotorValue);
  } 
   
}

void loop() {
  
  /* read sensors */
  for(int i=0;i<nInputs;i++)
  {  
    if(inputTab[i][1]==0)//Type lightSensor
    {
      //To comply with the simulator we cast this sensor output into a float between 0.1 and 1. with 1 = maxLight
      lightInput = float(analogRead(inputTab[i][0]))/1000;
      //Serial.print(lightInput); Serial.print("\t");  
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
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //normalize accel values from 16500 to 9.81 (1g)
      accelX = (float)(ax+xAccelOffset)*accelScaleFactor;Serial.print(xAccelOffset); Serial.print("\t");
      accelY = (float)(ay+yAccelOffset)*accelScaleFactor;Serial.print(yAccelOffset); Serial.print("\t");
      accelZ = (float)(az+zAccelOffset)*accelScaleFactor;Serial.print(zAccelOffset); Serial.print("\t");
      //fill in neuralNetwork
      networkInput[i] = accelX; i++; Serial.print(accelX); Serial.print("\t"); //0; i++; //
      networkInput[i] = accelY; i++; Serial.print(accelY); Serial.print("\t");//0; i++; //
      networkInput[i] = accelZ; i++; Serial.print(accelZ); Serial.print("\t");//0; i++; //
      //we add some noise to the gyros input here as the simulation and reality does not match yet
      networkInput[i] = 1.38778e-20; i++; //gx; i++; Serial.print(gx); Serial.print("\t"); 
      networkInput[i] = 1.38778e-20; i++; //gy; i++; Serial.print(gy); Serial.print("\t"); 
      networkInput[i] = 1.38778e-20; //gz; Serial.print(gz); Serial.print("\n"); 
    }
  }
  Serial.print("\n");
  
  /* Feed neural network with sensors values as input of the neural network*/
  feed(&network, &networkInput[0]);
  
  /* Step the neural network*/
  step(&network);
  
  /* Fetch the neural network ouputs */
  fetch(&network, &networkOutputs[0]);
  
  /* set Servos Motors according to the Fetch of the neural network outputs above*/
  for(int i=0;i<nOutputs;i++)
  {
    if(motor[i] == 0) //servoMotors => set poisition 
    {
      servoPosition = networkOutputs[i]; //goes from 0 degrees to 180 degrees 
      myservo[i].write(servoPosition);Serial.print(servoPosition); Serial.print("\t");
    }
    else if(motor[i] == 1)  // full rotation motors => set speed
    {
      if( networkOutputs[i] >= 90)//one rotation direction only => discard other rotation direction
        servoSpeed = networkOutputs[i] - 30; //we  
      else
        servoSpeed = 0;
        
      myservo[i].write(servoSpeed);Serial.print(servoSpeed); Serial.print("\t");
    }
    
  }
  Serial.print("\n");
  
  //slow the main loop frequency, not to burn servoMotors too quickly
  delay(500);
}

//hack for Arduino to be able to play with function that have struct as parameters.

/**
 * Initializes a NeuralNetwork data structure
 * @param network the neural network
 * @param nInputs the number of inputs of the neural network
 * @param nOutputs the number of outputs of the neural network
 * @param weight weights of the neural network. Weights must be provided in the same order as
 *               specified in the NeuralNetwork structure
 * @param bias the bias of each output neuron
 * @param gain the gain of each output neuron
 */
void initNetwork(NeuralNetwork* network, unsigned int nInputs, unsigned int nOutputs,
		const float *weights, const float* bias, const float* gain) {

	unsigned int i = 0;

	/* Copy weights, bias and gains */
	memcpy(network->weight, weights,
			sizeof(float) * (nInputs * nOutputs + nOutputs * nOutputs));
	memcpy(network->bias, bias,
			sizeof(float) * (nOutputs));
	memcpy(network->gain, gain,
			sizeof(float) * (nOutputs));

	/* Initialize states */
	for (i = 0; i < nOutputs * 2; ++i) {
		network->state[i] = 0.0;
	}

	/* Initialize inputs */
	for (i = 0; i < nInputs; ++i) {
		network->input[i] = 0.0;
	}

	network->nInputs = nInputs;
	network->nOutputs = nOutputs;

	network->curStateStart = 0;

}

/**
 * Feed the neural network with input values
 * @param network the neural network
 * @param input the input values, must be an array of m inputs
 */
void feed(NeuralNetwork* network, const float *input) {

	unsigned int i = 0;
	for (i = 0; i < network->nInputs; ++i) {
		network->input[i] = input[i];
	}

}

/**
 * Step the neural network of 1 timestep
 * @param network the neural network
 */
void step(NeuralNetwork* network) {

	unsigned int nextState;
	unsigned int i = 0;
	unsigned int j = 0;

	if (network->nOutputs == 0) {
		return;
	}

	/* For each output neuron, sum the state of all the incoming connection */
	nextState = (network->curStateStart + network->nOutputs)
			% network->nOutputs;

	for (i = 0; i < network->nOutputs; ++i) {

		float curNeuronActivation = 0;
		unsigned int baseIndexOutputWeigths = -1;

		for (j = 0; j < network->nInputs; ++j) {
			curNeuronActivation += network->weight[network->nOutputs * j + i]
					* network->input[j];
		}
		baseIndexOutputWeigths = network->nOutputs * network->nInputs;
		for (j = 0; j < network->nOutputs; ++j) {
			curNeuronActivation += network->weight[baseIndexOutputWeigths
					+ network->nOutputs * j + i]
					* network->state[network->curStateStart + j];
		}

		/* Save next state */
		curNeuronActivation -= network->bias[i];
		network->state[nextState + i] = 1.0
				/ (1.0 + exp(-network->gain[i] * curNeuronActivation));
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
		output[i] = network->state[network->curStateStart + i]*180;
	}

}

