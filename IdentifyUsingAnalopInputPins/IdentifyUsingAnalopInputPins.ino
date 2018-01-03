unsigned int DEVICE_IDENTITY=0;
boolean MASTER_MODE=false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000); while (!Serial); //delay for Leonardo
  identifySelf();
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    Serial.read();
    identifySelf();
  }
}

#define pinMasterMode  A5

#define pinLedRed      A0
#define pinLedGreen    A1
#define pinLedBlue     A2
#define pinLedYellow   A3

#define VALUE_TO_CHECK LOW // Set this to HIGH if you are using a common cathode LED

void identifySelf(){
  //to identify the MASTER_MODE or SLAVE_MODE, configure pin as an input and enable the internal pull-up resistor
  pinMode(pinMasterMode, INPUT_PULLUP);
  //to identify the DEVICE configure pins as input and enable the internal pull-up resistor
  pinMode(pinLedRed, INPUT_PULLUP);
  pinMode(pinLedGreen, INPUT_PULLUP);
  pinMode(pinLedBlue, INPUT_PULLUP);
  pinMode(pinLedYellow, INPUT_PULLUP);
  // identify the self device id and the mode of operation
  int sensorVal = HIGH;
  //read the master mode pin, print out the mode
  sensorVal = digitalRead(pinMasterMode);
  if(sensorVal == LOW) {MASTER_MODE = true;}
  else {MASTER_MODE = false;}
  //read the pin values into a variable
  sensorVal = digitalRead(pinLedRed);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0008; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFF7; }
  sensorVal = digitalRead(pinLedGreen);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0004; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFB; }
  sensorVal = digitalRead(pinLedBlue);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0002; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFD; }
  sensorVal = digitalRead(pinLedYellow);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0001; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFE; }
  printIdentity();
}

void printIdentity(){
  if(MASTER_MODE) Serial.print(F("MASTER")); else Serial.print(F("SLAVE"));
  Serial.print(" 0x");Serial.print(DEVICE_IDENTITY,HEX); Serial.print(" Color ");
  if (DEVICE_IDENTITY & 0x0008) Serial.print("R"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0004) Serial.print("G"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0002) Serial.print("B"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0001) Serial.println("Y"); else Serial.println("-");  
}

