//Ada_remoteFXTrigger_RX_motor
//Remote Effects Trigger Box Receiver
//by John Park
//for Adafruit Industries
//
// Button box receiver with motor FeatherWing
//
//
//MIT License


#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


/********** Motor Setup **************/
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
// Select which 'port' M1, M2, M3 or M4. In this case, M2
Adafruit_DCMotor *myDCMotor = AFMS.getMotor(1);//for a pump

 #define LED 13
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

bool oldState = HIGH;
int showType = 0;

void setup() {
  
  delay(500);
  Serial.begin(115200);
  
  //while (!Serial) { delay(1); } // wait until serial console is open, 
  //remove if not tethered to computer

  //Motor setup
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  myMotor->setSpeed(20);  // 10 rpm   
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myDCMotor->setSpeed(150);
  myDCMotor->run(FORWARD);
  // turn on motor
  myDCMotor->run(RELEASE);
  

  pinMode(LED, OUTPUT);  
     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Motor");

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  delay(500);
}


void loop(){
    
  if (rf69.waitAvailableTimeout(100)) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (! rf69.recv(buf, &len)) {
      Serial.println("Receive failed");
      return;
    }
    
    digitalWrite(LED, HIGH);
   
    rf69.printBuffer("Received: ", buf, len);
    buf[len] = 0;
    
    Serial.print("RX Got: "); 
    Serial.println((char*)buf);
    Serial.print("RSSI: "); 
    Serial.println(rf69.lastRssi(), DEC);

    
    char radiopacket[20] = "Button #";//prep reply message to send

    
    
     if (buf[0]=='H'){ //Card motor up
        myMotor->step(300, FORWARD, MICROSTEP); 
       radiopacket[8] = 'H';    
      }
     
     else if (buf[0]=='I'){ //Card motor down
        myMotor->step(300, BACKWARD, MICROSTEP); 
       radiopacket[8] = 'I';    
      }

     else if (buf[0]=='J'){ //Pump
      myDCMotor->run(FORWARD);
           uint8_t i;
      for (i=0; i<255; i++) {
        myDCMotor->setSpeed(i);  
      delay(10);
      }
        myDCMotor->run(RELEASE);
       radiopacket[8] = 'J';    
      }
     radiopacket[9] = 0;

    //Serial.print("Sending "); Serial.println(radiopacket);
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();

    digitalWrite(LED, LOW);
  }
}


