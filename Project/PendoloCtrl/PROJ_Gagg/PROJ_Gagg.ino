#include <SPI.h>

#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

#define BAUDRATE 57600
#define BAUDRATE_2 19200
#define MAXSPS 8000 //12500

#define RES12 12
#define RES14 14

#define ENC 10
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52

#define SWITCH_BLACK A0 // Switch Black
#define SWITCH_WHITE A1 // Switch White
#define VCC_PIN 31
#define THR 1018

#define CONST 0.0208

// Control Parameters
#define MAX_ERROR 25
#define MIN_ERROR 2

#define INIT_POSE 70
#define REF_ANGLE 180
#define REF_POSE 70

#define KP_ANGLE 3150 //12500 //2750
#define KI_ANGLE 1100 //1000 - 1050

#define KP_POSE 140 //100 - 150
#define KI_POSE 10 // - 10

typedef enum{INIT,
             RESET,
             REACH_POSE_BLACK,
             REACH_POSE_WHITE,
             WAIT,
             CTRL} machineState;

char bufferRX[16];
char display_speedControl[16];
char encPos_str[16];

uint16_t encodedAngle_1 = 0, encodedAngle_2 = 0;
uint32_t msTime_Current = 0, msTime_Past = 0;
double deltaTime = 0;
double int_ang_error = 0, int_pose_error = 0;
double Angle1_Pendulum_minus_Offset = 0.0, Angle2_Pendulum = 0.0, newAngle3_fromEncoder = 0.0;

char read_Serial2;
uint8_t id_BufferRX = 0;
int value_BufferRX = 0;

float ang_error = 0.0, last_ang_error = 0.0;   //TBD
float pose_error = 0.0, last_pose_error = 0.0;
long ctrlAccel = 0, speedControl = 0;
double intAngle = 0.0, previousAngle_fromEncoder = 0.0, cart_Position = 0.0;
bool SwitchWhite_on = false, isActive_Switch_Black = false;

machineState state = INIT;

/////////////////////////////
// definizione vettori
double w[] = {0, 0, 0};
double e[] = {0, 0, 0};
double u[] = {0, 0, 0};

double motorSpeed = 0;
double ang_rad_1 = 0;

//////////////////////////////

void setup(){
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

void loop(){
  // Read Switches
  int value_Switch_White = analogRead(SWITCH_WHITE);
  int value_Switch_Black = analogRead(SWITCH_BLACK);

  if(value_Switch_Black > THR){
    isActive_Switch_Black = true;
  } else{
    isActive_Switch_Black = false;
  }

  if(value_Switch_White > THR){
    SwitchWhite_on = true;
  } else{
    SwitchWhite_on = false;
  }
 
  // Read Econder for Positioning
  if(state != 0){
    uint8_t attempts = 0;
    uint16_t encoderPosition = getPositionSPI(ENC, RES12); 

    if (encoderPosition != 0xFFFF){
      newAngle3_fromEncoder = encoderPosition / 4096.0*360.0;
    }

    if((previousAngle_fromEncoder - newAngle3_fromEncoder) > 300){
      intAngle = intAngle + 360;
    }  else if((previousAngle_fromEncoder - newAngle3_fromEncoder) < -300){
      intAngle = intAngle - 360;
    }
    
    previousAngle_fromEncoder = newAngle3_fromEncoder;
    cart_Position = (intAngle + newAngle3_fromEncoder)*CONST;
  }

  // Read Serial
  while(Serial2.available()){
    read_Serial2 = Serial2.read();
    if(read_Serial2==13 || read_Serial2==10){
      if(id_BufferRX > 0){
        bufferRX[id_BufferRX] = 0;
        value_BufferRX = atoi(bufferRX);
      }
      id_BufferRX = 0;
      
    } else{
      bufferRX[id_BufferRX] = read_Serial2;
      id_BufferRX++;
      id_BufferRX = id_BufferRX&0x07;
    }
  }
  
  if(value_BufferRX >= 0x4000){
    encodedAngle_2 = value_BufferRX - 0x4000;
    Angle2_Pendulum = encodedAngle_2 / 4096.0*360.0;
  } 
  else{
    encodedAngle_1 = value_BufferRX - 15;
    Angle1_Pendulum_minus_Offset = encodedAngle_1 / 4096.0*360.0;
  }

  // Time Handling
  msTime_Current = millis();
  deltaTime = (msTime_Current - msTime_Past)/1000.0;
  msTime_Past = msTime_Current;

  // State Machine
  switch(state){

      // Go Toward BLACK Switch
    case INIT:{
      Serial.println("Homing");
      speedControl = 2500;
      if(isActive_Switch_Black){
        speedControl = 0;
        state = RESET;
      }
      break;  
    }

    // Reset Encoder & Cart Position
    case RESET:{
      setZeroSPI(ENC);
      intAngle = 0.0;
      cart_Position = 0.0;
      state = REACH_POSE_BLACK;
      break;
    }

    case REACH_POSE_BLACK:{
      Serial.println("Reaching Initial Position");
      speedControl = -2500;

      if(cart_Position > INIT_POSE){
        speedControl = 0;
        state = WAIT;
      }

      break;
    }

    case REACH_POSE_WHITE:{
      Serial.println("Reaching Initial Position");
      speedControl = 2500;

      if(cart_Position < INIT_POSE){
        speedControl = 0;
        state = WAIT;
      }

      break;
    }

    case WAIT:{
      speedControl = 0;
      ang_error = REF_ANGLE - Angle1_Pendulum_minus_Offset;

      if(abs(ang_error) < MIN_ERROR){
        state = CTRL;
      }

      motorSpeed=0;

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
    }

    case CTRL:{
      // Put Here Your Regulator //
      ang_error = REF_ANGLE - Angle1_Pendulum_minus_Offset;
      ang_rad_1 = ang_error*3.14/180; // converto l'angolo da gradi a radianti
      e[4]=e[3];
      e[3]=e[2];
      e[2]=e[1];
      e[1]=e[0];
      e[0]=-ang_rad_1;
      
      u[4]=u[3];
      u[3]=u[2];
      u[2]=u[1];
      u[1]=u[0];
      //u[0]=1.984*u[1]-0.9841+u[2]+17.82*e[0]-35.48*e[1]+17.66*e[2];
      u[0]=3.952*u[1]-5.858*u[2]+3.859*u[3]-0.9531*u[4]+28.29*e[0]-112.1*e[1]+116.62*e[2]-110.3*e[3]+27.26*e[4];

      if (u[0]>20.0){
        u[0]=20.0;
      }
      if (u[0]<-20.0){
        u[0]=-20.0;
      }
    
    
      // velocità da imporre
      
      motorSpeed += u[0]/0.7*0.005;
      
      if (motorSpeed>1.0){
        motorSpeed=1.0;
      }
      if (motorSpeed<-1.0){
        motorSpeed=-1.0;
      }
      
      // Conversione con impulsi
      speedControl = -motorSpeed / 0.000109375;

     
      ///////////////////////////

      //ctrlAccel = controlAction; //round(controlAction);
      //speedControl += ctrlAccel*deltaTime;

      if(abs(ang_error) > MAX_ERROR || isActive_Switch_Black || SwitchWhite_on){
        motorSpeed = 0;
        pose_error = 0;
        ang_error = 0;
        
        int_pose_error = 0;
        int_ang_error = 0;
    
        speedControl = 0.0;
        ctrlAccel = 0.0; 

        if(isActive_Switch_Black){
          state = REACH_POSE_BLACK;
        }

        if(SwitchWhite_on){
          state = REACH_POSE_WHITE;
        }

        if(abs(ang_error) > MAX_ERROR && !isActive_Switch_Black && !SwitchWhite_on){
          state = WAIT;
        }
      }
      
      break;
    }
  }

  // Saturate Control Action
  if(speedControl > MAXSPS){
    speedControl = MAXSPS;
  }
  
  if(speedControl < -MAXSPS){
    speedControl = -MAXSPS;
  }
    

  ltoa(speedControl, display_speedControl, 10);
  //Serial.println(display_speedControl);
  Serial3.println(display_speedControl);
  Serial.print(ang_rad_1);
  Serial.print(" - ");
  Serial.print(u[0]);
  Serial.print(" - ");
  Serial.println(motorSpeed);

  // Set Delay?
  delay(2);
}


/************* Helper Functions ************/

/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 */
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
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
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit. 
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine(uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}
