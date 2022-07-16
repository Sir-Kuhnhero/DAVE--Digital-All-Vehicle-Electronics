#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";


bool reciveData(){
  bool recived = false;
  long time = millis();

  while(!recived)
  {
    if (radio.available()) {
      char text[32] = "";
      radio.read(&text, sizeof(text));
      Serial.println(text);

      recived = true;
    }

    if (millis() - time > 1000)
      return false;
  }

  return true;
}


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (!reciveData())
    Serial.println("no data connection!!!");
  
  delay(10);
}