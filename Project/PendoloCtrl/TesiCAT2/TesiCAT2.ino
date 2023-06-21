// LIBRARIES
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
// ENCODER SETTINGS VARIABLES
#define AMT22_NOP 0x00    //Get position   nei bytes [0x00, 0x00]
#define AMT22_RESET 0x60  //reset Encoder  nei bytes [0x00, 0x60]
#define AMT22_ZERO 0x70   //Set Zero       nei bytes [0x00, 0x70]
//Serial and SPI data transfer speeds
#define BAUDRATE 57600    //Used in serial monitor
#define BAUDRATE_2 19200  //not used
#define MAXSPS 8000       //12500 //the maximum clock speed 
// ENCODER RESOLUTION COSTANTS
#define RES12 12          //definig for 12 bit encoder
#define RES14 14          //definig for 14 bit encoder
// ENCODER PINS
#define ENC 10            //ENCODER definition
#define SPI_MOSI 51       //Master Out <- Slave In
#define SPI_MISO 50       //Master In  -> Slave Out
#define SPI_SCLK 52       //Serial Clock
// SWITCH PINS
#define SW_B A3           // Switch Black [analog3]
#define SW_W A1           // Switch White [analog1]
#define VCC_PIN 31        // Power supply
#define THR 600           // GIVEN Threshold

#define CONST 0.0208      // per la conversione da angoli in cm
#define OFFSET_ANGLE 10   // da 0 a 4096 come da  0° a 360°  // da ricalibrare // forse di 0.6°
#define SAMPLE_TIME 2     // milliseconds GIVEN tempo di campionamento [CBE]

// CONTROL VARIABLES
// -- angle --
#define MAX_ANGLE_ERROR 25 //Effettuiamo il controllo se l'angolo 
#define MIN_ANGLE_ERROR 5  //è compreso tra [-5 e 5] e controlliamo tra [-25 e 25] gradi
#define REF_ANGLE_POSITION 180
// -- cart --
#define REF_CART_POSITION 75  //la rotaia è lunga 150, setto 75 come metà


// MACHINE STATES
typedef enum { INIT,
               RESET,
               REACH_POSE_BLACK,
               REACH_POSE_WHITE,
               WAIT,
               CTRL
             } machineState;

machineState state = INIT;  // INITIAL STATE



// SERIAL COMMUNICATION WITH MOTOR CONTROL ARDUINO
char bufferRX[16];
char ctrlAction_str[16];

int16_t encAngle_1 = 0, encAngle_2 = 0;
uint32_t msTime_Now = 0, msTime_Past = 0;
double deltaTime = 0;
double ang_1 = 0.0, ang_2 = 0.0, ang_3 = 0.0;

char c;            //lettura porta seriale
uint8_t bidx = 0;  // bidx or zero
int readVal = 0;   // converts the string argument str to an integer BufferRx[id]

double angle_error = 0.0;
double angle_rad_1 = 0;
double position_error = 0.0;
long ctrlSpeed = 0;
double intAngle = 0.0, prev_angle = 0.0, cart_position = 0.0;
bool sw_w_on = false, sw_b_on = false;



/////////////////////////////
// REGULATOR VECTORS DEFINITION///////
double u = 0.0;          // Control input


// -- angle --
double e_a[] = {0, 0, 0, 0, 0};	  // Regulator error vector 
double u_a[] = {0, 0, 0, 0, 0};   // Regulator error control input
// -- position --
double u_p[] = {0, 0, 0, 0, 0};	  // Regulator error control input
double e_p[] = {0, 0, 0, 0, 0};		// Regulator error vector 

double v = 0.0;  // MOTOR SPEED

//////////////////////////////


//Standard Setup Arduino SPI
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



  /****************************************************/
  // [ 1 ]
  //Controllo che i valori degli switch rientrino all'inteno del limite imposto AnalogThreshold
  if (val_b > THR){sw_b_on = true;}
  else{sw_b_on = false;}
  if (val_w > THR){sw_w_on = true;}
  else{ sw_w_on = false;}
  /****************************************************/
   


  /****************************************************/
  // [ 2 ]
  // READ CART POSITION and ANGLE from Encoder SPI
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
  position_error = REF_CART_POSITION - cart_position;
  /****************************************************/



  /****************************************************/
  // [ 3 ]
  // READ PENDULUM ANGLE from Serial2
  while (Serial2.available()) {               // Se Serial2 è disponibile
    c = Serial2.read();                       // Read Serial2 Value
    if (c == 13 || c == 10) {                 // Se è 10 o 13 (perchè a noi interessano 13, 10?)
      if (bidx > 0) {
        bufferRX[bidx] = 0;;                  // CLEAR bufferRX[bidx] 
        readVal = atoi(bufferRX);             // SET readVal (String to integer) Demo[String:"300" -> Int:300]
      }
      bidx = 0;                               // SET bidx at 0
    } 
    else {                                    // FIRST ITERATION and IF Serial2 != 10 or 13
      bufferRX[bidx] = c;                     // SET bufferRX[bidx] as serial input
      bidx++;                                 // increment bidx by 1
      bidx = bidx & 0x07;                     // SET check if (bidx reached 7 [0-7]) : bool
    }
  }

  if (readVal > 5000) {                       // se readVal > 5000
    encAngle_2 = readVal - 0x4000;            // encodedAngle_2 = readVal - 16384(as hex)
    ang_2 = encAngle_2 / 4096.0 * 360.0;      // valore angolo in gradi
  } 
  else {
    encAngle_1 = readVal - OFFSET_ANGLE;
    ang_1 = (encAngle_1 / 4096.0) * 360.0;      // valore angolo in gradi
  }
  angle_error = REF_ANGLE_POSITION - ang_1;
  /***************************************************/



  /***************************************************/
  // [ 4 ]
  // TIME HANDLING
  msTime_Now = micros();                         //GET microseconds since Boot
  deltaTime = (msTime_Now - msTime_Past) / 1000000.0;
  msTime_Past = msTime_Now;
  /***************************************************/



  /***************************************************/
  // [ 5 ]
  // Macchina degli Stati
  switch (state) {

    case INIT:{  // Go Toward BLACK Switch
      Serial.println("INIT | Homing");
      ctrlSpeed = 2500;
      if (sw_b_on) {
        ctrlSpeed = 0;
        state = RESET;
      }
      break;
    }

    case RESET:{  // Reset Encoder & Cart Position
      Serial.println("RESET | Resetting");
      setZeroSPI(ENC);
      intAngle = 0.0;
      cart_position = 0.0;
      state = REACH_POSE_BLACK;
      break;
    }

    case REACH_POSE_BLACK:{  //To Reach initial Position
      Serial.println("REACH_POSE_BLACK | Reaching Initial Position");
      ctrlSpeed = -2500;

      if (cart_position > REF_CART_POSITION) {
        ctrlSpeed = 0;
        state = WAIT;
      }
      break;
    }

    case REACH_POSE_WHITE:{  //To Reach initial Position
      Serial.println("REACH_POSE_WHITE | Reaching Initial Position");
      ctrlSpeed = 2500;
      if (cart_position < REF_CART_POSITION) {
        ctrlSpeed = 0;
        state = WAIT;
      }
      break;
    }

    case WAIT:{  //Waiting to reach the upside position
      Serial.println("WAIT | Waiting angle to be between -5 and 5");
      ctrlSpeed = 0;
      v = 0;

      // -- angle --
      e_a[] = {0, 0, 0, 0, 0};
      u_a[] = {0, 0, 0, 0, 0};
      // -- position --
      e_p[] = {0, 0, 0, 0, 0};
      u_p[] = {0, 0, 0, 0, 0};

      if (abs(angle_error) < MIN_ANGLE_ERROR) {
        state = CTRL;
        Serial.println("WAIT | to CTRL, init states");        
      }
      break;
    }

    case CTRL:{  // Regulator Control state
      Serial.println("CTRL | Executing control algos");
      angle_rad_1 = angle_error * 3.14 / 180; // converto l'angolo da gradi a radianti

    
      // -- angle --
      e_a[2] = e_a[1]
      e_a[1] = e_a[0]
      //e_a[2] = e_a[1]
      u_a[2] = u_a[1]
      u_a[1] = u_a[0]

      // -- position --
      e_p[2] = e_p[1]
      e_p[1] = e_p[0]
      e_p[0] = - position_error

      u_p[2] = u_p[1]
      u_p[1] = u_p[0]

      //u_p[0]= 1 * (e_p[0] + e_p[1] + e_p[2] - u_p[0]D - u_p[1]D - u_p[2]D);
      u_p[0] = (0 * e_p[0]) + (0.0006768664772887434 * e_p[1]) - (-0.0006763522535967913 * e_p[2]) - (0 * u_p[0]) - (-1.990028938436608 * u_p[1]) - (0.990049833749168 * u_p[2]);

      e_a[0] = u_p[0] - angle_rad_1;

      //u_a[0]= 1 * (e_a[0]N + e_a[1]N + e_a[2]N - u_a[0]D - u_a[1]D - u_a[2]D);
      u_a[0] = (-357 * e_a[0]) + (707.3636501260482 * e_a[1]) + (-350.3918502248686 * e_a[2]) - (-1.876340995079374 * u_a[1]) + (-0.876340995079373 * u_a[1]);


      /***********************************/
      // Speed setting
      v += u_a[0] / 0.7 * (SAMPLE_TIME / 1000.0);  // FORCE/VELOCITY conversion
      if (v > 0.9){ v = 0.9; }
      if (v < -0.9){ v = -0.9;}
      /***********************************/


      /***********************************/
      // Speed/rpm conversion
      ctrlSpeed = v / 0.000109375;
      //ctrlAccel = controlAction; 
      //round(controlAction);
      //ctrlSpeed += ctrlAccel*deltaTime;
      /***********************************/

    

      if (abs(angle_error) > MAX_ANGLE_ERROR || sw_b_on || sw_w_on) {
        //Serial.println("Sono qui");
        position_error = 0;
        angle_error = 0;
        v = 0;
        ctrlSpeed = 0;
        if (sw_b_on) { state = REACH_POSE_BLACK; }
        if (sw_w_on) { state = REACH_POSE_WHITE; }
        if (!sw_b_on && !sw_w_on) { state = REACH_POSE_WHITE; }
      }
      break;
    }
  }
  /****************************************************/

  // Saturate Control Action
  if (ctrlSpeed > MAXSPS){ctrlSpeed = MAXSPS;}
  if (ctrlSpeed < -MAXSPS){ctrlSpeed = -MAXSPS;}

  ltoa(ctrlSpeed, ctrlAction_str, 10);
  Serial3.println(ctrlAction_str);

  delay(SAMPLE_TIME);  // tempo imposto per la discretizzazione del controllo
}






/***************************************************/
// [ 5 ]
// Helper Functions
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution) {
  /*
   This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
   for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
   For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
   is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
   This function takes the pin number of the desired device as an input
   This funciton expects res12 or res14 to properly format position responses.
   Error values are returned as 0xFFFF
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
   This function does the SPI transfer. sendByte is the byte to transmit.
   Use releaseLine to let the spiWriteRead function know if it should release
   the chip select line after transfer.
   This function takes the pin number of the desired device as an input
   The received data is returned.
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
   This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere
   This function takes the pin number of the desired device as an input
  */
  digitalWrite(encoder, csLine);
}
void setZeroSPI(uint8_t encoder) {
  /*
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
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
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
  */
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  spiWriteRead(AMT22_RESET, encoder, true);

  delay(250);  //250 second delay to allow the encoder to start back up
}
/***************************************************/