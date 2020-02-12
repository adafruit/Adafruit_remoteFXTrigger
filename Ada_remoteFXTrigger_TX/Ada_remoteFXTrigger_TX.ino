//Ada_remoteFXTrigger_TX
//Remote Effects Trigger Box Transmitter
//by John Park
//for Adafruit Industries
//
// General purpose button box
// for triggering remote effects
// using packet radio Feather boards
//
//
//MIT License


#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_Trellis.h"
#include <Encoder.h>

/********* Encoder Setup ***************/
#define PIN_ENCODER_SWITCH 11
Encoder knob(10, 12);
uint8_t activeRow = 0;
long pos = -999;
long newpos;
int prevButtonState = HIGH;
bool needsRefresh = true;
bool advanced = false;
unsigned long startTime;


/********* Trellis Setup ***************/
#define MOMENTARY 0
#define LATCHING 1
#define MODE LATCHING //all Trellis buttons in latching mode
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0);
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
#define INTPIN A2

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();
#if defined(ESP8266)
  #define BUTTON_A 0
  #define BUTTON_B 16
  #define BUTTON_C 2
  #define LED      0
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define LED      13
#elif defined(ARDUINO_STM32F2_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
  #define LED PB5
#elif defined(TEENSYDUINO)
  #define BUTTON_A 4
  #define BUTTON_B 3
  #define BUTTON_C 8
  #define LED 13
#elif defined(ARDUINO_FEATHER52)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
  #define LED 17
#else // 32u4, M0, and 328p
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
  #define LED      13
#endif


/************ Radio Setup ***************/
// Can be changed to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
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

int lastButton=17; //last button pressed for Trellis logic

int menuList[8]={1,2,3,4,5,6,7,8}; //for rotary encoder choices
int m = 0; //variable to increment through menu list
int lastTB[8] = {16, 16, 16, 16, 16, 16, 16, 16}; //array to store per-menu Trellis button

/*******************SETUP************/
void setup() {
  delay(500);
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, 
  //remove if not tethered to computer


  // INT pin on Trellis requires a pullup
  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);
  trellis.begin(0x70);  

  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);//set encoder push switch pin to input pullup
  
  digitalPinToInterrupt(10); //on M0, Encoder library doesn't auto set these as interrupts
  digitalPinToInterrupt(12);
  
  // Initialize OLED display
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.setTextWrap(false);
  oled.display();
  delay(500);
  oled.clearDisplay();
  oled.display();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

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

  oled.setCursor(0,0);
  oled.println("RFM69 @ ");
  oled.print((int)RF69_FREQ);
  oled.println(" MHz");
  oled.display();
  delay(1200); //pause to let freq message be read by a human

  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.println("REMOTE FX");
  oled.setCursor(0,16);
  oled.println("TRIGGER");  
  oled.display();

  // light up all the LEDs in order
  for (uint8_t i=0; i<numKeys; i++) {
    trellis.setLED(i);
    trellis.writeDisplay();    
    delay(30);
  }
  // then turn them off
  for (uint8_t i=0; i<numKeys; i++) {
    trellis.clrLED(i);
    trellis.writeDisplay();    
    delay(30);
  }
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
void loop() {
  delay(30); // 30ms delay is required, dont remove me! (Trellis)

    /*************Rotary Encoder Menu***********/

    //check the encoder knob, set the current position as origin
    long newpos = knob.read() / 4;//divide for encoder detents
    
    /* // for debugging
     Serial.print("pos=");
     Serial.print(pos);
     Serial.print(", newpos=");
     Serial.println(newpos);
    */

    if(newpos != pos){
      int diff = newpos - pos;//check the different between old and new position
      if(diff>=1){
        m++; 
        m = (m+8) % 8;//modulo to roll over the m variable through the list size
       }

      if(diff==-1){ //rotating backwards
         m--;
         m = (m+8) % 8;
       }
      /* //uncomment for debugging or general curiosity
      Serial.print("Diff = ");
      Serial.print(diff);
      Serial.print("  pos= ");
      Serial.print(pos);
      Serial.print(", newpos=");
      Serial.println(newpos);
      Serial.println(menuList[m]);
      */

      pos = newpos;

      // Serial.print("m is: ");
      //Serial.println(m);


      //clear Trellis lights 
      for(int t=0;t<=16;t++){
        trellis.clrLED(t);
        trellis.writeDisplay();
      }
      //light last saved light for current menu
        trellis.clrLED(lastTB[m]);
        trellis.setLED(lastTB[m]);
        trellis.writeDisplay();
      
      //write to the display
      oled.setCursor(0,3);
      oled.clearDisplay();

      int p; //for drawing bullet point menu location pixels
      int q;

      if (m==0){
        for(p=0;p<4;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Sharks");
      }
      if (m==1){
        for(p=4;p<8;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" NeoPixels");
      }
      if (m==2){
        for(p=8;p<12;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Motors");
      }
      if (m==3){
        for(p=12;p<16;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Lamps");
      }
      if (m==4){
        for(p=16;p<20;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Traps");
      }
      if (m==5){
        for(p=20;p<24;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Radio");
      }
      if (m==6){
        for(p=24;p<28;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Doors");
      }
      if (m==7){
        for(p=28;p<32;p++){
          for(q=0;q<4;q++){
            oled.drawPixel(q,p,WHITE);
          }
        }
        oled.print(" Pyro");
      }
      
      oled.display();
    }

// remember that the switch is active low
    int buttonState = digitalRead(PIN_ENCODER_SWITCH);
    if (buttonState == LOW) {
        unsigned long now = millis();
        if (prevButtonState == HIGH) {
            prevButtonState = buttonState;
            startTime = now;
           // Serial.println("button pressed");
            trellis.clrLED(lastTB[m]);
            trellis.writeDisplay();    
            lastTB[m]=17;//set this above the physical range so 
            //next button press works properly
        } 
    } 
    else if (buttonState == HIGH && prevButtonState == LOW) {
      //Serial.println("button released!");
      prevButtonState = buttonState;
    }

  /*************Trellis Button Presses***********/
  if (MODE == MOMENTARY) {
    if (trellis.readSwitches()) { // If a button was just pressed or released...
      for (uint8_t i=0; i<numKeys; i++) { // go through every button
        if (trellis.justPressed(i)) { // if it was pressed, turn it on
        //Serial.print("v"); Serial.println(i);
          trellis.setLED(i);
        } 
        if (trellis.justReleased(i)) { // if it was released, turn it off
          //Serial.print("^"); Serial.println(i);
          trellis.clrLED(i);
        }
      }
      trellis.writeDisplay(); // tell the trellis to set the LEDs we requested
    }
  }

  if (MODE == LATCHING) {
    if (trellis.readSwitches()) { // If a button was just pressed or released...
      for (uint8_t i=0; i<numKeys; i++) { // go through every button
        if (trellis.justPressed(i)) { // if it was pressed...
         //Serial.print("v"); Serial.println(i);

         // Alternate the LED unless the same button is pressed again
         //if(i!=lastButton){
         if(i!=lastTB[m]){ 
          if (trellis.isLED(i)){
              trellis.clrLED(i);
              lastTB[m]=i; //set the stored value for menu changes
          }
          else{
            trellis.setLED(i);
            //trellis.clrLED(lastButton);//turn off last one
            trellis.clrLED(lastTB[m]);
            lastTB[m]=i; //set the stored value for menu changes
          }
          trellis.writeDisplay();
        }
            char radiopacket[20];
            
        /**************SHARKS**************/
        //check the rotary encoder menu choice
        if(m==0){//first menu item
            if (i==0){ //button 0 sends button A command
              radiopacket[0] = 'A';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Sharks");
              oled.setCursor(0,16);
              oled.print("Kill....ON");
              oled.display();  
            }
            if (i==1){ //button 1 sends button B command
              radiopacket[0] = 'B';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Sharks");
              oled.setCursor(0,16);
              oled.print("Kill...OFF");
              oled.display(); 
            }
            if (i==4){ //
              radiopacket[0] = 'C';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Sharks");
              oled.setCursor(0,16);
              oled.print("Sleep....ON");
              oled.display(); 
            } 
            if (i==5){ //
              radiopacket[0] = 'D';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Sharks");
              oled.setCursor(0,16);
              oled.print("Sleep...OFF");
              oled.display(); 
            } 
        }
        /**************NeoPixels**************/
        if(m==1){//next menu item
            if (i==0){ //button 0 sends button A command
              radiopacket[0] = 'E';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("NeoPixels");
              oled.setCursor(75,16);
              oled.print("RED");
              oled.display(); 
            }
            if (i==1){ //button 1 sends button B command
              radiopacket[0] = 'F';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("NeoPixels");
              oled.setCursor(70,16);
              oled.print("GREEN");
              oled.display(); 
            }
            if (i==2){ //button 4 sends button C command
              radiopacket[0] = 'G';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("NeoPixels");
              oled.setCursor(75,16);
              oled.print("BLUE");
              oled.display();  
            } 

            if(i>=3 && i<=15){
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("NeoPixels");
              oled.display();  
            }  
        }
        /**************Motor Props**************/
        if(m==2){//next menu item
            if (i==0){ //button 0 sends button D command CARD UP
              radiopacket[0] = 'H';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Motors");
              oled.setCursor(0,16);
              oled.print("Card....UP");
              oled.display();   
            }
            if (i==1){ //button 1 sends button E command CARD DOWN
              radiopacket[0] = 'I';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Motors");
              oled.setCursor(0,16);
              oled.print("Card..DOWN");
              oled.display();
            }
            if (i==4){ //button 4 sends button F command PUMP RUN temp
              radiopacket[0] = 'J';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Motors");
              oled.setCursor(0,16);
              oled.print("Pump...RUN");
              oled.display(); 
            }
            if(i>=5 && i<=15){
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Motors");

              oled.display();  
            }   
        }
        /**************Lamps**************/
        if(m==3){//next menu item
            if (i==0){ 
              radiopacket[0] = 'K';
             oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Lamp");
              oled.setCursor(0,16);
              oled.print("Spot1...ON");
              oled.display();   
            }
            if (i==1){ 
              radiopacket[0] = 'L';
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Lamp");
              oled.setCursor(0,16);
              oled.print("Spot1..Off");
              oled.display();   
            }
            if(i>=2 && i<=15){
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("Lamp");
              oled.display();  
            }   
        }
            
          Serial.print("Sending "); 
          Serial.println(radiopacket[0]);

          rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
          rf69.waitPacketSent(); 
          //reset packet so unassigned buttons don't send last command
          radiopacket[0]='Z'; //also being used to turn off NeoPixels 
          //from any unused button

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
            
            //Serial.print("TX Got: "); 
            //Serial.println((char*)buf);
            Serial.print("RSSI: "); 
            Serial.println(rf69.lastRssi(), DEC);

            //delay(1000);//chill for a moment before returning the message to RX unit

            /*************Reply message from RX unit***********/
            //oled.clearDisplay();
            //oled.print((char*)buf[0]);
            //oled.print("RSSI: "); oled.print(rf69.lastRssi());
            //oled.display(); 
            
            
            digitalWrite(LED, LOW);
          }

          //lastButton=i;//set for next pass through to turn this one off
        } 
      }
      // tell the trellis to set the LEDs we requested
      trellis.writeDisplay();
    }
  }
}
