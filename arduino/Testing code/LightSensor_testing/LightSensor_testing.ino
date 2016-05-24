/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  delay(20);
  int sensor1Value = analogRead(A0);
  delay(20);        // delay in between reads for stability
  sensor1Value = analogRead(A0);
  // print out the value you read:
  Serial.print(sensor1Value);
  Serial.print("\t");

  delay(20);
  int sensor2Value = analogRead(A1);
  delay(20);        // delay in between reads for stability
  sensor2Value = analogRead(A1);
  // print out the value you read:
  Serial.print(sensor2Value);
  Serial.print("\t");

  delay(20);
  int sensor3Value = analogRead(A2);
  delay(20);        // delay in between reads for stability
  sensor3Value = analogRead(A2);
  // print out the value you read:
  Serial.print(sensor3Value);

  delay(20);
  int sensor4Value = analogRead(A3);
  Serial.print("\t");
  delay(20);        // delay in between reads for stability
  sensor4Value = analogRead(A3);
  // print out the value you read:
  Serial.println(sensor4Value);
}
