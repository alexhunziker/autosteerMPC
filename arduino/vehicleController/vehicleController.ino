const byte numChars = 8;
char recievedChars[numChars];
char input[4];

char serdata[8] = "0000000";
boolean newData = false;

#include <Servo.h>

Servo steering;
Servo break1;
Servo break2;
Servo break3;
Servo break4;

#define enA 3
#define in1 A1
#define in2 A2

void setup() {
  // Attach servos to pins
  steering.attach(2);
  break1.attach(5);
  break2.attach(6);
  break3.attach(7);
  break4.attach(9);

  // Calibration
  steering.write(90);
  break1.write(150);
  break2.write(150);
  break3.write(35);
  break4.write(44);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Serial.begin(9600);
  Serial.println("INFO: Arduino initialized");
}

void loop() {
  char startMarker = '<';
  char endMarker = '>';
  char currentChar;
  static byte index = 0;
  static boolean isControlStream = false;
  char steeringSequence[3] = "00";

  while (Serial.available() > 0 && newData == false) {
    currentChar = Serial.read();

    if(isControlStream){
      if(currentChar == endMarker){
        recievedChars[index] = '\0';  //Temination symbol
        isControlStream = false;
        index = 0;
        newData = true;
      } else {
        recievedChars[index] = currentChar;
        serdata[index] = currentChar;
        index++;
        if(index >= numChars) {
          index = numChars - 1;
        }
      }
    }
    else if (currentChar == startMarker) {
      isControlStream = true;
    }
  }

  if (newData == true) {
    steeringSequence[0] = recievedChars[0];
    steeringSequence[1] = recievedChars[1];

    steering.write(atoi(steeringSequence)+40);
    analogWrite(enA, int(recievedChars[2]-'0')*25);

    break1.write(187-int(recievedChars[3]-'0')*2+44);
    break2.write(187-int(recievedChars[3]-'0')*2+44);
    break3.write(int(recievedChars[3]-'0')*2+35);
    break4.write(int(recievedChars[3]-'0')*2+45);

    Serial.println("DEBUG: Control signals updated");
    newData = false;
  }
}
