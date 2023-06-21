// LIBRARIES
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
// ENCODER SETTINGS VARIABLES
#define AMT22_NOP 0x00
#define AMT22_RESET 0x60
#define AMT22_ZERO 0x70
//Serial and SPI data transfer speeds
#define BAUDRATE 57600
#define BAUDRATE_2 19200
#define MAXSPS 8000  //12500
// ENCODER RESOLUTION COSTANTS
#define RES12 12
#define RES14 14
// ENCODER PINS
#define ENC 10
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
// SWITCH PINS
#define SW_B A3     // Switch Black
#define SW_W A1     // Switch White
#define VCC_PIN 31  // Power supply
#define THR 600

#define CONST 0.0208     // per la conversione da angoli in cm
#define OFFSET_ANGLE 10  // da 0 a 4096 come da  0° a 360°  // da ricalibrare // forse di 0.6°
#define SAMPLE_TIME 2    //milliseconds

// CONTROL VARIABLES
#define MAX_ERROR 25
#define MIN_ERROR 5

#define INIT_POSE 70

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

int16_t encAngle_1 = 0, encAngle_2 = 0;
uint32_t ttNow = 0, tt = 0;
double deltaT = 0;
double ang_1 = 0.0, ang_2 = 0.0, ang_3 = 0.0;

char c;
uint8_t bidx = 0;
int readVal = 0;

double ang_error = 0.0;
double pose_error = 0.0;
long ctrlSpeed = 0;
double intAngle = 0.0, prev_angle = 0.0, cart_position = 0.0;
bool sw_w_on = false, sw_b_on = false;

machineState state = INIT;  // INITIAL STATE

/////////////////////////////
// REGULATOR VECTORS DEFINITION///////
double u = 0.0;          // Control input


//
//aggiungere variabili di stato
//

double v = 0.0;  // MOTOR SPEED


// ANGLE
double e_angle[] = {0, 0, 0, 0, 0};     // Regulator error control input
double u_angle[] = {0, 0, 0, 0, 0};		  // Regulator error vector 

double v = 0;
double ang_rad_1 = 0;

//////////////////////////////

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

void loop() {

  int val_w = analogRead(SW_W);
  int val_b = analogRead(SW_B);

  if (val_b > THR)
    sw_b_on = true;
  else
    sw_b_on = false;

  if (val_w > THR)
    sw_w_on = true;
  else
    sw_w_on = false;
  /****************************************************/
  /********** Read cart position from Encoder**********/
  if (state != 0) {
    uint8_t attempts = 0;
    uint16_t encoderPosition = getPositionSPI(ENC, RES12);  // encoderPosition è definito da 0 a 4096 come da 0° a 360°

    if (encoderPosition != 0xFFFF) {
      ang_3 = encoderPosition / 4096.0 * 360.0;  // trasformo in gradi
    }

    if ((prev_angle - ang_3) > 300) {
      intAngle = intAngle + 360;
    } else if ((prev_angle - ang_3) < -300) {
      intAngle = intAngle - 360;
    }

    prev_angle = ang_3;
    cart_position = (intAngle + ang_3) * CONST;  //Trasformo gradi in  cm
  }
  /****************************************************/
  /********** Read pendulum angle from Serial2 ********/
  while (Serial2.available()) {
    c = Serial2.read();
    if (c == 13 || c == 10) {
      if (bidx > 0) {
        bufferRX[bidx] = 0;
        readVal = atoi(bufferRX);
      }
      bidx = 0;

    } else {
      bufferRX[bidx] = c;
      bidx++;
      bidx = bidx & 0x07;
    }
  }

  if (readVal > 5000) {
    encAngle_2 = readVal - 0x4000;
    ang_2 = encAngle_2 / 4096.0 * 360.0;

  } else {
    encAngle_1 = readVal - OFFSET_ANGLE;
    ang_1 = (encAngle_1 / 4096.0) * 360.0;
  }
  //ang_error = 180 - ang_1;
  /***************************************************/

  /****************** Time Handling ******************/
  ttNow = micros();
  deltaT = (ttNow - tt) / 1000000.0;
  tt = ttNow;
  /***************************************************/

  /****************** State Machine ******************/
  switch (state) {
    case INIT:  // Go Toward BLACK Switch
      Serial.println("Init: Homing");
      ctrlSpeed = 2500;
      if (sw_b_on) {
        ctrlSpeed = 0;
        state = RESET;
      }
      break;

    case RESET:  // Reset Encoder & Cart Position
      Serial.println("Reset state");
      setZeroSPI(ENC);
      intAngle = 0.0;
      cart_position = 0.0;
      state = REACH_POSE_BLACK;
      break;

    case REACH_POSE_BLACK:  //To Reach initial Position
      Serial.println("Reaching Initial Position");
      ctrlSpeed = -2500;

      if (cart_position > INIT_POSE) {
        ctrlSpeed = 0;
        state = WAIT;
      }
      break;

    case REACH_POSE_WHITE:  //To Reach initial Position
      Serial.println("Reaching Initial Position");
      ctrlSpeed = 2500;
      if (cart_position < INIT_POSE) {
        ctrlSpeed = 0;
        state = WAIT;
      }
      break;

    case WAIT:  //Waiting to reach the upside position
      Serial.println("WAITING");
      ctrlSpeed = 0;
      ang_error = REF_ANGLE - ang_1;
      if(abs(ang_error) < MIN_ERROR){
        state = CTRL;
      }
      v=0;
      e[4] = 0;
      e[3] = 0;
      e[2] = 0;
      e[1] = 0;
      e[0] = 0;

      u[4] = 0;
      u[3] = 0;
      u[2] = 0;
      u[1] = 0;
      u[0] = 0;
      break;

    case CTRL:  // Regulator Control state
    
      //
      //
      //Inserire il regolatore
      //
      //
      e[4]=e[3];
      e[3]=e[2];
      e[2]=e[1];
      e[1]=e[0];
      e[0]=-ang_rad_1;
      u[4]=u[3];
      u[3]=u[2];
      u[2]=u[1];
      u[1]=u[0];

      u[0]=1000*u[1]-1991.292*u[2]+991.3057*u[3]-1*e[0]+1.8187*e[1]-0.81873*e[2];


      /***********************************/
      /***********Speed setting***********/
      v += u / 0.7 * (SAMPLE_TIME / 1000.0);  // FORCE/VELOCITY conversion
      if (v > 0.9)
        v = 0.9;
      if (v < -0.9)
        v = -0.9;

      /***********************************/
      /********Speed/rpm conversion*****/
      ctrlSpeed = v / 0.000109375;

      /***********************************/

      //ctrlAccel = controlAction; //round(controlAction);
      //ctrlSpeed += ctrlAccel*deltaT;

      if (abs(ang_error) > MAX_ERROR || sw_b_on || sw_w_on) {
        //Serial.println("Sono qui");
        pose_error = 0;
        ang_error = 0;
        v = 0;
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
      }
      break;
  }

  // Saturate Control Action
  if (ctrlSpeed > MAXSPS)
    ctrlSpeed = MAXSPS;

  if (ctrlSpeed < -MAXSPS)
    ctrlSpeed = -MAXSPS;



  ltoa(ctrlSpeed, ctrlAction_str, 10);
  Serial3.println(ctrlAction_str);

  delay(SAMPLE_TIME);  // tempo imposto per la discretizzazione del controllo
}


/************* Helper Functions ************/

/*
   This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
   for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
   For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
   is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
   This function takes the pin number of the desired device as an input
   This funciton expects res12 or res14 to properly format position responses.
   Error values are returned as 0xFFFF
*/
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution) {
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

/*
   This function does the SPI transfer. sendByte is the byte to transmit.
   Use releaseLine to let the spiWriteRead function know if it should release
   the chip select line after transfer.
   This function takes the pin number of the desired device as an input
   The received data is returned.
*/
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine) {
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

/*
   This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere
   This function takes the pin number of the desired device as an input
*/
void setCSLine(uint8_t encoder, uint8_t csLine) {
  digitalWrite(encoder, csLine);
}

/*
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
*/
void setZeroSPI(uint8_t encoder) {
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250);  //250 second delay to allow the encoder to reset
}

/*
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
*/
void resetAMT22(uint8_t encoder) {
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  spiWriteRead(AMT22_RESET, encoder, true);

  delay(250);  //250 second delay to allow the encoder to start back up
}
