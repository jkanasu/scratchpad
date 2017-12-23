/* send.ino Example sketch for IRLib2
 *  Illustrates how to send a code.
 */
#include <IRLibDecodeBase.h>
#include <IRLibSendBase.h>    // First include the send base
//Now include only the protocols you wish to actually use.
//The lowest numbered protocol should be first but remainder 
//can be any order.
#include <IRLib_P03_RC5.h>   
#include <IRLibCombo.h>     // After all protocols, include this
// All of the above automatically creates a universal sending
// class called "IRsend" containing only the protocols you want.
// Now declare an instance of that sender.
#include <IRLibRecv.h>
#include <IRLibRecvPCI.h>

IRrecvPCI myReceiver(2); //create receiver and pass pin number
IRdecode myDecoder;   //create decoder
//Create a buffer that we will use for decoding one stream while
//the receiver is using the default buffer "recvGlobal.recvBuffer"
// NOTE this eats away 100 x 16 bits -- i.e. 200 bytes
//uint16_t myBuffer[RECV_BUF_LENGTH];

IRsend mySender;

void setup() {
  Serial.begin(9600);
  delay(2000); while (!Serial); //delay for Leonardo
  Serial.println(F("Every time you press a key is a serial monitor we will send."));
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  Serial.println(F("Ready to receive IR signals"));
}

void loop() {
  if (Serial.read() != -1) {
    //send a code every time a character is received from the 
    // serial port. You could modify this sketch to send when you
    // push a button connected to an digital input pin.
    sendSignal();
  }
  receiveSignal();
}

int numOfSignalsSent = 0;
void sendSignal(){
  Serial.println(F("Sending signal"));
  numOfSignalsSent++;
  //Substitute values and protocols in the following statement
  // for device you have available.
  //mySender.send(SONY,0xa8bca, 20);//Sony DVD power A8BCA, 20 bits
  //mySender.send(NEC,0x61a0f00f,0);//NEC TV power button=0x61a0f00f
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  receiveSignal();
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  Serial.print(F("Sent signal "));  Serial.println(numOfSignalsSent, DEC);
}

int numOfSignalsReceived = 0;
void receiveSignal() {
  //Serial.println(F("Checking signal"));
  //Continue looping until you get a complete signal received
  if (myReceiver.getResults()) {
    Serial.println(F("decoding signal"));
    myDecoder.decode();           //Decode it
    myDecoder.dumpResults(true);  //Now print results. Use false for less detail
    myReceiver.enableIRIn();      //Restart receiver
    Serial.println(F("enabled receiver"));
    numOfSignalsReceived++;
    Serial.print(F("numOfSignalsReceived is ")); Serial.println(numOfSignalsReceived, DEC);
  }
}

