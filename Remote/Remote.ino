#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define VX A1
#define VY A2
#define CLK 5
#define DT 4
// #define Button 2

RF24 radio(9, 10);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "00001";

const int XDZ = 100;
const int YDZ = 100;

int calibx, caliby;
int deltax, deltay;
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";


struct Data_frame{
  int XPOS;
  int YPOS;
  int MODE;
};

void setup() {
  pinMode(VX, INPUT);
  pinMode(VY, INPUT);
  pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
  // pinMode(Button, INPUT_PULLUP);
  
  calibx = map(analogRead(VX), 0, 1023, 0, 255);
  caliby = map(analogRead(VY), 0, 1023, 0, 255);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  Serial.begin(19200);
  lastStateCLK = digitalRead(CLK);
}

void loop() {

  Data_frame msg;
  int x = map(analogRead(VX), 0, 1023, 0, 255);
  int y = map(analogRead(VY), 0, 1023, 0, 255);
  msg.XPOS = -(calibx - x);
  msg.YPOS = -(caliby - y);
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(DT) != currentStateCLK) {
			counter --;
			currentDir ="CCW";
		} else {
			// Encoder is rotating CW so increment
			counter ++;
			currentDir ="CW";
		}
    if (counter > 2){
      counter = 0;
    }else if(counter < 0){
      counter = 2;
    }
    Serial.print("Direction: ");
		Serial.print(currentDir);
		Serial.print(" | Counter: ");
		Serial.println(counter);
	}
  lastStateCLK = currentStateCLK;
  msg.MODE = counter;
  radio.write(&msg, sizeof(Data_frame));
  delay(10);
}
