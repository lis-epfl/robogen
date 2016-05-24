/* Written for the  Sparkfun sensors but works the same for the pololu ones although the pololus have their own library.
 * Final version integrated with the vectors as they are used in the Robogen Arduino code. 
 * /!\ Some modfications were made to the library!!!
 * 
 * The Github version is up-to-date.
 */
 
#include <Servo.h>
#include <Wire.h>
#include "SparkFun_VL6180X.h"

VL6180x sensor0(0x29),sensor1(0x29),sensor2(0x29),sensor3(0x29);
VL6180x irSensors[]={sensor0,sensor1,sensor2,sensor3};
int sensorAdresses[] = {0x28,0x27,0x26,0x25};

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

const int inputTab[][2] = { {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {A0, 3}, {A1, 3}, {A2, 3}, {A3, 3} };
const int irIndices[] = { -1, -1, -1, -1, -1, -1, 0, 1, 2, 3 };
const int outputTab[][2] = { {D9, 0}, {D10, 0}, {D5, 0}, {D6, 0}, {D11, 0}, {D13, 0}, {ROLL, 0}, {PITCH, 0} };

#define NB_INPUTS 10
#define NB_OUTPUTS 8

#define ACTUATION_PERIOD 40
/* Keep elapsed time */
unsigned long t = 0;

bool started=false;
int startTime;

void setup() {
  
  delay(1000);
  Serial.begin(115200); // Start Serial at 115200bps
  //Wire.begin(); // Start I2C library
  //delay(7000); // delay 7s

  /* Define the IR sensor pins as output*/
    for(int i=0;i<NB_INPUTS;i++)
  {  
    // The pins are used to enable/disable the IR sensor for initialization. They are set to OUTPUT. 
    if(inputTab[i][1] == 3)
    {
      pinMode(inputTab[i][0], OUTPUT);
      digitalWrite(inputTab[i][0], LOW);
    }
  }
  
  delay(1000);
  /* Initialize IR sensors : */
  //set each one to HIGH in turn and change the address.
  for(int i=0;i<NB_INPUTS;i++)
  {
    if(inputTab[i][1] == 3) 
    {
      Serial.print("Init IR sensor");
      Serial.println(irIndices[i]);
        for(int j=0;j<NB_INPUTS;j++){
          if(i==j){
            digitalWrite(inputTab[j][0],HIGH);
          }
        }
        delay(10); // This is required to let the chip realise that it is enabled !!
  
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
     
    Serial.println(irSensors[irIndices[i]]._i2caddress);
    irSensors[irIndices[i]].changeAddress(0x29,sensorAdresses[irIndices[i]]);
    Serial.println(irSensors[irIndices[i]]._i2caddress);
    irSensors[irIndices[i]].VL6180xDefautSettings(); //Load default settings to get started.
    }
  }

  delay(1000);

  //All sensors are on HIGH. Readjust a few parameters.
  for(int i=0;i<NB_INPUTS;i++)
  {
    if(inputTab[i][1] == 3)
    {
      irSensors[irIndices[i]].VL6180xReadjustParameters(); // Readjust the parameters of the sensors
      Serial.print("sensor ");
      Serial.print(irIndices[i]);
      Serial.println(" readjust");
    }
  }
  delay(1000);

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
    delay(10);

    /* read sensors */
    for(int i=0;i<NB_INPUTS;i++)
    {
      if(inputTab[i][1]==3) // Type is IR sensor
        //
      {
        //irSensors[irIndices[i]].setSingleShot();
        int distance = irSensors[irIndices[i]].getDistance2();

        Serial.print("\t R ");
        Serial.print(distance);

        //networkInput[i] = 1.0 - (distance/255.0); //i++;
        //networkInput[i] = 0.0; // give light value in [0,1]
      }
    }
    Serial.println();
  }
};


