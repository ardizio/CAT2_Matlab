// LIBRARIES
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

// Scheda AMT22 Arduino SPI (Serial Peripheral Interface)
// ENCODER SETTINGS VARIABLES
#define AMT22_NOP 0x00    //Get position   nei bytes [0x00, 0x00]
#define AMT22_RESET 0x60  //reset Encoder  nei bytes [0x00, 0x60]
#define AMT22_ZERO 0x70   //Set Zero       nei bytes [0x00, 0x70]

//Serial and SPI data transfer speeds
#define BAUDRATE 57600    //Used in serial monitor
#define BAUDRATE_2 19200  //baud
#define MAXSPS 8000       //12500 //the maximum clock speed 

// ENCODER RESOLUTION COSTANTS
// AMT22 is a 12 and 14 bit encoder
#define RES12 12          //definig for 12 bit encoder
#define RES14 14          //definig for 14 bit encoder

// ENCODER PINS
#define ENC 10            //ENCODER definition
#define SPI_MOSI 51       //Master Out <- Slave In
#define SPI_MISO 50       //Master In  -> Slave Out
#define SPI_SCLK 52       //Serial Clock

// SWITCH PINS
#define SWITCH_BLACK A3   // Switch Black  [analog3]
#define SWITCH_WHITE A1   // Switch White  [analog1]
#define VCC_PIN 31        // Power supply
#define AnalogThreshold 600   // GIVEN Threshold

#define CONST 0.0208      // per la conversione da angoli in cm
#define OFFSET_ANGLE 10   // da 0 a 4096 come da  0° a 360°  // da ricalibrare // forse di 0.6° [15]
#define SAMPLE_TIME 6     //milliseconds GIVEN tempo di campionamento

// CONTROL VARIABLES
#define MAX_ERROR 30      // GIVEN D? TBD
#define MIN_ERROR 3       // GIVEN D? TBD

#define REF_ANGLE 180     // angolo di riferimento
#define INIT_POSE 70      // GIVEN 70cm from start
#define REF_POSE 70       // added by me

#define QUANTIZER 0.00019175

// Casistiche possibili
// MACHINE STATES
typedef enum { INIT,
               RESET,
               REACH_POSE_BLACK,
               REACH_POSE_WHITE,
               WAIT,
               CTRL
               } machineState;
               
// SERIAL COMMUNICATION WITH MOTOR CONTROL ARDUINO
char bufferRX[16];
char ctrlAction_str[16];

 

int16_t encodedAngle_1 = 0, encodedAngle_2 = 0;      // 2 angoli
uint32_t msTime_Current = 0, msTime_Past = 0;        // 2 tempi
double deltaTime = 0;                                // differenza di tempo
double Angle1_pendulumMinusOffset = 0.0, Angle2_Pendulum = 0.0, newAngle3_fromEncoder = 0.0;  // 3 angoli



char c;         //lettura porta seriale
uint8_t bidx = 0;   // bidx or zero
int readVal = 0;    // atoi BufferRx[id]



double ang_error = 0.0;    //TBD
double pose_error = 0.0;   //TBD
long ctrlSpeed = 0;        //TBD
double intAngle = 0.0, previousAngle_fromEncoder = 0.0, cart_Position = 0.0;
bool sw_w_on = false, sw_b_on = false;


// Definizione dello stato iniziale del sistema
machineState state = INIT;       // INITIAL STATE
machineState lastState = INIT;   // LASTSTATE STATE


/////////////////////////////
// REGULATOR VECTORS DEFINITION///////

// ANGLE
double u_angle[] = {0, 0, 0, 0, 0};		  // Regulator error control input
double e_angle[] = {0, 0, 0, 0, 0};		  // Regulator error vector 
// POSITION
double u_position[] = {0, 0, 0, 0, 0};	// Regulator error control input
double e_position[] = {0, 0, 0, 0, 0};		// Regulator error vector 

double v = 0.0;        // MOTOR SPEED
double ang_radiants = 0;

//////////////////////////////

//Standard Setup Arduino SPI [is Standard]
void setup() {
  Serial.begin(BAUDRATE);
  Serial2.begin(BAUDRATE);
  Serial3.begin(BAUDRATE_2);

  SPI.begin();
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC, OUTPUT);
  digitalWrite(ENC, HIGH);

  SPI.setClockDivider(SPI_CLOCK_DIV32);

  pinMode(VCC_PIN, OUTPUT);
  digitalWrite(VCC_PIN, HIGH);
}


//MY APPLICATION
void loop() {

  int value_Switch_Black = analogRead(SWITCH_BLACK);
  int value_Switch_White = analogRead(SWITCH_WHITE);


  // [ 1 ]
  //Controllo che i valori degli switch rientrino all'inteno del limite imposto AnalogThreshold
  if (value_Switch_Black > AnalogThreshold){ sw_b_on = true;} 
  else{ sw_b_on = false; }

  if (value_Switch_White > AnalogThreshold){ sw_w_on = true; } 
  else{ sw_w_on = false; }



  // [ 2 ]
  // READ CART POSITION and ANGLE from Encoder SPI
  if (state != 0) {
    //uint8_t attempts = 0;

    // encoderPosition è definito da 0 a 4096 come da 0° a 360°
    uint16_t encoderPosition = getPositionSPI(ENC, RES12);  

    //if(encoderPosition != 0xFFFF) [where hex: 0xFFFF=4095]
    if (encoderPosition != 0xFFFF) {
      // trasformo encoderPosition da [0-4095} in gradi [0-360]
      newAngle3_fromEncoder = encoderPosition / 4096.0 * 360.0;  
    }

    //Genrero l'angolo iniziale
    if ((previousAngle_fromEncoder - newAngle3_fromEncoder) > 300) {
      intAngle = intAngle + 360;
    } else if ((previousAngle_fromEncoder - newAngle3_fromEncoder) < -300) {
      intAngle = intAngle - 360;
    }

    previousAngle_fromEncoder = newAngle3_fromEncoder;
    cart_Position = (intAngle + newAngle3_fromEncoder) * CONST;  //Trasformo gradi in  cm
    //would like top print

    //for nextRun: previousAngle_fromEncoder
    //for thisRun: cart_Position && intAngle
  }
  



  // [ 3 ]
  // READ PENDULUM ANGLE from Serial2
  while (Serial2.available()) {                          // Se Serial2 è disponibile
    c = Serial2.read();                       // Read Serial2 Value
    if (c == 13 || c == 10) {      // Se è 10 o 13 (perchè a noi interessano 13, 10?)
      if (bidx > 0) {
        bufferRX[bidx] = 0;                       // CLEAR bufferRX[bidx] 
        readVal = atoi(bufferRX);                 // SET readVal as 
                                                         // atoi(String to integer) Demo[tring:"300" -> Int:300]
      }
      bidx = 0;                                   // SET bidx at 0
    } 
    else {                                               // FIRST ITERATION and IF Serial2 != 10 or 13
      bufferRX[bidx] = c;              // SET bufferRX[bidx] as serial input
      bidx++;                                     // increment bidx by 1
      bidx = bidx & 0x07;                  // SET check if (bidx reached 7 [0-7]) : bool
    }
  }
  //Diff
  if (readVal > 5000) {                           // se readVal > 5000
    encodedAngle_2 = readVal - 0x4000;            // encodedAngle_2 = readVal - 16384(as hex)
    Angle2_Pendulum = encodedAngle_2 / 4096.0 * 360.0;
  }
  else {
      //MEH, Where should i Use?
    encodedAngle_1 = readVal - OFFSET_ANGLE;     //errore angolo SECONDO ME
    Angle1_pendulumMinusOffset = (encodedAngle_1 / 4096.0) * 360.0;
  }



  // [ 4 ]
  // TIME HANDLING
  msTime_Current = micros();                   //GET microseconds since Boot
  deltaTime = (msTime_Current - msTime_Past) / 1000000.0;
  msTime_Past = msTime_Current;
  
  

  // [ 5 ]
  // TIME HANDLING
  switch (state) {

    // Go Toward BLACK Switch
    case INIT:{  
      // if(lastState != INIT){
      //   Serial.println("INIT");
      //   lastState = INIT;
      // }
      Serial.println("INIT");
      ctrlSpeed = 2500;
      if (sw_b_on) {          // if cart at "Home"
        ctrlSpeed = 0;                      // SET ctrlSpeed at 0
        state = RESET;                      // NEXT phase: .RESET
      }
      break;
    }


    // Reset Encoder & Cart Position
    case RESET:{  
      Serial.println("Reset State");
      setZeroSPI(ENC);                        // SET zero and wait reset time
      intAngle = 0.0;                         // RESET intAngle
      cart_Position = 0.0;                    // RESET cart_Position
      state = REACH_POSE_BLACK;               // NEXT phase: .REACH_POSE_BLACK
      break;
    }
    


    //To Reach initial Position
    case REACH_POSE_BLACK:{
      Serial.println("REACH_POSE_BLACK");
      ctrlSpeed = -2500;                    // SET ctrlSpeed at -2500
      if (cart_Position > INIT_POSE) {      // if cart > "INIT_POSE"
        ctrlSpeed = 0;                      // RESET ctrlSpeed at 0 [don't push] ho sorpassato, credo
        state = WAIT;                       // NEXT phase: .WAIT
      }                                     // else NEXT phase still: .REACH_POSE_BLACK
      break;
    }
     

    //To Reach initial Position
    case REACH_POSE_WHITE: {
      Serial.println("REACH_POSE_WHITE");
      ctrlSpeed = 2500;                     // SET ctrlSpeed at 2500
      if (cart_Position < INIT_POSE) {      // if cart < "INIT_POSE"
        ctrlSpeed = 0;                      // RESET ctrlSpeed at 0 [don't push] ho sorpassato, credo
        state = WAIT;                       // NEXT phase: .WAIT
      }                                     // else NEXT phase still: .REACH_POSE_WHITE
      break;
    }
     

    //Waiting to reach the upside position IDLE
    case WAIT: {
      Serial.println("WAIT");
      ctrlSpeed = 0;
      ang_error = 180 - Angle1_pendulumMinusOffset;

      if(abs(ang_error) < MIN_ERROR){
        state = CTRL;
         // Serial.print("{");
        // Serial.print(Angle1_pendulumMinusOffset);
        // Serial.print(", ");
        // Serial.print(ang_error);
        // Serial.print(", ");
        // Serial.print(cart_Position);
        // Serial.print("}");
        // Serial.print("\n");
        Serial.print("from WAIT to CONTROL");
      }

      v=0;
      
      e_angle[4] = 0;
      e_angle[3] = 0;
      e_angle[2] = 0;
      e_angle[1] = 0;
      e_angle[0] = 0;

      u_angle[4] = 0;
      u_angle[3] = 0;
      u_angle[2] = 0;
      u_angle[1] = 0;
      u_angle[0] = 0;
    
      break;
    }
      
    
    
    // Regulator Control State
    case CTRL:{
      Serial.println("CONTROL");

      ang_error = 180 - Angle1_pendulumMinusOffset;
      ang_radiants = ang_error * 3.14 / 180; 
      
      e_angle[4] = e_angle[3];
      e_angle[3] = e_angle[2];
      e_angle[2] = e_angle[1];
      e_angle[1] = e_angle[0];
      e_angle[0] = - ang_radiants;
    
      u_angle[4] = u_angle[3];
      u_angle[3] = u_angle[2];
      u_angle[2] = u_angle[1];
      u_angle[1] = u_angle[0];
      //TBD IMPLEMENT YOUR CONTROLLER HERE
      u_angle[0]=290*u_angle[1]-577.3355*u_angle[2]+287.3397*u_angle[3]-1*e_angle[0]+1.9139*e_angle[1]-0.91393*e_angle[2];
      

      //Speed settings
      v += u_angle[0] / 0.7 * (SAMPLE_TIME / 1000.0);  // FORCE/VELOCITY conversion

      if (v > 0.9){
        v = 0.9;
      }
      if (v < -0.9){
        v = -0.9;
      }
        
      //Conversion Speed/rpm
      ctrlSpeed = v / 0.000109375;

 
      Serial.print("v: ");
      Serial.print(v);
      Serial.print(" | ");
      Serial.print("ctrlSpeed: ");
      Serial.print(ctrlSpeed);
      Serial.print(" | ");
      Serial.print("ang_error: ");
      Serial.print(ang_error);
      Serial.print(" | ");
      Serial.print("u_angle[0]: ");
      Serial.print(u_angle[0]);
      Serial.print("\n");
      
      if (abs(ang_error) > MAX_ERROR || sw_b_on || sw_w_on) { 
        Serial.println("INIT from CONTROL");

        v = 0;
        ang_error = 0;
        ctrlSpeed = 0;

        if (sw_b_on) {
          state = REACH_POSE_BLACK;
        }
        if (sw_w_on) {
          state = REACH_POSE_WHITE;
        }
        if (!sw_b_on && !sw_w_on) {
          state = REACH_POSE_WHITE;
        }
        lastState = CTRL;
      }
      break;
    }   
  }

  // Saturate Control Action
  if (ctrlSpeed > MAXSPS){
    ctrlSpeed = MAXSPS;
    Serial.print("[M] SATURATION");
  }
  if (ctrlSpeed < -MAXSPS){
    ctrlSpeed = -MAXSPS;
    Serial.print("[m] SATURATION");
  }

  ltoa(ctrlSpeed, ctrlAction_str, 10);
  Serial3.println(ctrlAction_str);

  Serial.print(ang_radiants);
  Serial.print(" - ");
  Serial.print(u_angle[0]);
  Serial.print(" - ");
  Serial.println(v);

  //Delay
  // 2millis, tempo imposto per la discretizzazione del controllo
  delay(SAMPLE_TIME);  
}


/************* Helper Functions ************/
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution) {
  /*
    * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
    * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
    * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
    * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
    * This function takes the pin number of the desired device as an input
    * This funciton expects res12 or res14 to properly format position responses.
    * Error values are returned as 0xFFFF
  */
  uint16_t currentPosition;
  bool binaryArray[16];

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for (int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
      && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0]))) {
    //we got back a good position, so just mask away the checkbits
    currentPosition &= 0x3FFF;
  } else {
    currentPosition = 0xFFFF;  //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine) {
  /*
  * This function does the SPI transfer. sendByte is the byte to transmit. 
  * Use releaseLine to let the spiWriteRead function know if it should release
  * the chip select line after transfer.  
  * This function takes the pin number of the desired device as an input
  * The received data is returned.
  */


  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder, LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command
  data = SPI.transfer(sendByte);
  delayMicroseconds(3);             //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine);  //if releaseLine is high set it high else it stays low

  return data;
}
void setCSLine(uint8_t encoder, uint8_t csLine) {
  /*
  * This function sets the State of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
  * This function takes the pin number of the desired device as an input
  */
  digitalWrite(encoder, csLine);
}
void setZeroSPI(uint8_t encoder) {
    /*
  * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
  * second byte is the command.  
  * This function takes the pin number of the desired device as an input
  */

  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250);  //250 second delay to allow the encoder to reset
}
void resetAMT22(uint8_t encoder) {

  /*
    * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
    * second byte is the command.  
    * This function takes the pin number of the desired device as an input
    */

  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  spiWriteRead(AMT22_RESET, encoder, true);

  delay(250);  //250 second delay to allow the encoder to start back up
}