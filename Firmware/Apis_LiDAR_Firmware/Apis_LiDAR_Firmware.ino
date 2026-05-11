#include "SlowSoftI2CMaster.h"
#include "WireS.h"
#include <avr/sleep.h>
#include <EEPROM.h>
// #include <EEPROM.h> //DEBUG!

// Commands
#define CTRL_REG1_ADR 0x20
#define CTRL_REG2_ADR 0x21
#define CTRL_REG3_ADR 0x22
#define CTRL_REG4_ADR 0x23
#define TEMP_CFG_REG_ADR 0x1F
#define OUT_X_ADR 0x28 //Low byte
#define OUT_Y_ADR 0x2A
#define OUT_Z_ADR 0x2C

// Pin definitions
#define POWER_SW 2
#define ACCEL_INT 7 //Interrupt from accelerometer, active low, needs internal pullup
#define MODE_TRIGGER 13 //Trigger for pulse width mode
#define MODE_READ 11 //Read pin for pulse width mode, also INTO
#define STAT_LED 14 //Status LED
#define HALL_SWITCH 8 //Hall effect switch output 
#define ENABLE 4 //Enable pin for Lidar Lite


// #define ACCEL_ADR 0x18
const int ACCEL_ADR = 0x18; //DEBUG!

#define LIDAR_ADR 0x62

#define READ 0x01
#define WRITE 0x00

#define BUF_LENABLEGTH 64 //Length of I2C Buffer, verify with documentation 

// #define ADR_ALT 0x41 //Alternative device address

const unsigned long GlobalTimeout = 200; //Time to wait before timing out

bool LidarFail = false; //Used to indicate failure of Lidar unit
bool AccelFail = false; //Used to indicate failure of on board accelerometer 

volatile uint8_t ADR = 0x50; //Use arbitraty address, change using generall call??
// const uint8_t ADR_Alt = 0x41; //Alternative device address  //WARNING! When a #define is used instead, problems are caused
// NOTE: Switching to 0x41 via a solder jumper requires a board revision to
// add address-selection hardware; no such circuit exists in the current design
// (JP1 is the MIC2544 current-limit jumper, not an address jumper).
// Planned: EEPROM-stored address set via a writable firmware register,
// taking effect on next boot. See github.com/NorthernWidget/Project-Apis.

unsigned int Config = 0; //Global config value
unsigned long Period = 100; //Number of ms between sample events for continuious running

// I2C register map (16 bytes, indices 0x00–0x0F):
//   0x00        Ready flag: 0 = booting/LiDAR not yet initialised, 1 = ready.
//               Set to 0 at startup and whenever power is cut to the LiDAR;
//               set to 1 after InitLiDAR() completes. The library polls this
//               register (up to 150 ms) so it can exit as soon as the LiDAR is
//               ready rather than waiting a fixed time. Old firmware that lacks
//               this flag always reads 0x00 as 0, so the library falls back to
//               the 150 ms timeout, which covers the full startup sequence:
//               delay(10) + POWER_SW + cap charge (~15 ms) + delay(100) +
//               ENABLE + InitAccel + InitLiDAR ≈ 115 ms, with margin.
//               See: github.com/NorthernWidget-Skunkworks/Project-Symbiont-LiDAR/issues/15
//   0x01        Config (written by library: sensitivity bits [1:0])
//   0x02–0x03   Range [cm], little-endian int16
//   0x04–0x09   Accelerometer X, Y, Z raw, little-endian int16 each
//   0x0A–0x0F   Accelerometer offsets X, Y, Z, little-endian int16 each
uint8_t Reg[16] = {0}; //Initialize registers
// bool StartSample = true; //Flag used to start a new converstion, make a conversion on startup
// const unsigned int UpdateRate = 5; //Rate of update

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_A2, PIN_A3, true);  //Initialize software I2C

volatile bool StopFlag = false; //Used to indicate a stop condition 
volatile uint8_t RegID = 0; //Used to denote which register will be read from
volatile bool RepeatedStart = false; //Used to show if the start was repeated or not

int16_t Offsets[3] = {0};  //X,Y,Z acceleration offsets to zero the angle of the device 
int16_t AccelVals[3] = {0}; //Global storage for acceleration data values to be shared between EEPROM functions and getter functions 

bool SwitchLatch = false;  //Latching functionality control for Hall effect switch 
uint8_t LiDAR_Config = 0; //Use default config 

void setup() {
  // Serial.begin(115200); //DEBUG!
  // Serial.println("begin"); //DEBUG!
  // pinMode(ADR_SEL_PIN, INPUT_PULLUP);
  // if(!digitalRead(ADR_SEL_PIN)) ADR = ADR_Alt; //If solder jumper is bridged, use alternate address //DEBUG!
  // NOTE: ADR_SEL_PIN is not defined or wired in the current board revision.
  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, HIGH);
  pinMode(POWER_SW, OUTPUT);
  // digitalWrite(POWER_SW, HIGH); //Turn on power //DEBUG!
  // delay(500); //DEBUG!
  digitalWrite(POWER_SW, LOW); //Turn off output power //FIX??
  Wire.begin(ADR);  //Begin slave I2C
  Serial.begin(9600);
  // Serial.println("START"); //DEBUG!
  // EEPROM.write(0, ADR);

  //Setup I2C slave
  Wire.onAddrReceive(addressEvent); // register event
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);
  Wire.onStop(stopEvent);

  
  pinMode(ACCEL_INT, INPUT); //DEBUG! 
  pinMode(MODE_READ, INPUT);
  pinMode(MODE_TRIGGER, OUTPUT);
  pinMode(HALL_SWITCH, INPUT); 
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  digitalWrite(MODE_TRIGGER, HIGH); //Configure as pullup

  Reg[0] = 0; // Not ready: LiDAR not yet initialised
  delay(10);
  digitalWrite(POWER_SW, HIGH); // Turn on power; 680 µF cap charges at ~227 mA
  delay(100); // Wait for cap charge (~15 ms) and LiDAR Lite power-on (~22 ms)
  digitalWrite(ENABLE, HIGH);
  si.i2c_init(); //Begin I2C master
  InitAccel();
  InitLiDAR();
  Reg[0] = 1; // Ready: LiDAR initialised and accepting I2C commands
  digitalWrite(STAT_LED, LOW);  //Blink on statup
  if(!digitalRead(HALL_SWITCH)) {
    UpdateOffset(Offsets); //Clear values (offsets are 0 on startup until read into)
    SwitchLatch = true; //Set latch to prevent override 
  }
  digitalWrite(STAT_LED, HIGH);

}

void loop() {
  // static unsigned int Count = 0; //Counter to determine update rate
  // if(StartSample == true) {
  //  //Read new values in
  //  AutoRange_Vis();  //Run auto range
  //  delay(800); //Wait for new sample
  //  SplitAndLoad(0x0B, GetALS()); //Load ALS value
  //  SplitAndLoad(0x0D, GetWhite()); //Load white value
  //  SplitAndLoad(0x02, long(GetUV(0))); //Load UVA
  //  SplitAndLoad(0x07, long(GetUV(1))); //Load UVB
  //  SplitAndLoad(0x10, GetLuxGain()); //Load lux multiplier 
  //  SplitAndLoad(0x13, GetADC(0));
  //  SplitAndLoad(0x15, GetADC(1));
  //  SplitAndLoad(0x17, GetADC(2));

  //  StartSample = false; //Clear flag when new values updated  
  // }
  // if(Count++ == UpdateRate) {  //Fix update method??
  //  StartSample = true; //Set flag if number of updates have rolled over 
  //  Count = 0;
  // }

  // for(int i = 0; i < 128; i++) {
  //  si.i2c_start((i << 1) | WRITE);
  //  Serial.print(i, HEX);
  //  Serial.print('\t');
  //  Serial.println(si.i2c_write(0xFF)); //Write MSB
  //  si.i2c_stop();
  // }
  // while(digitalRead(7), LOW); //Wait for updated values //DEBUG!
  // ReadByte(ACCEL_ADR, 0x27);
  // ReadWord(ACCEL_ADR, OUT_X_ADR);
  LiDAR_Config = Reg[1] & 0x03; //Pull low two bits from Config reg 1 to get Lidar configuration state
  InitLiDAR(); //reinitialize LiDAR after power cycle
  Reg[0] = 1; // Ready: LiDAR initialised and accepting I2C commands
  unsigned long StartTime = millis();  //Measure time from start of measurment 
  uint8_t Stat1 = ReadByte(ACCEL_ADR, 0x27); 
  uint8_t Stat2 = ReadByte(ACCEL_ADR, 0x07);
  // while(((Stat1 & 0x08) >> 3) != 1 || ((Stat2 & 0x08) >> 3) != 1 || ((Stat2 & 0x80) >> 7) != 1) {
  unsigned long LocalTime = millis();
  while(((Stat1 & 0x08) >> 3) != 1 || Stat2 != 0xFF && (millis() - LocalTime) < GlobalTimeout) {  //Try to get status from 
    Stat1 = ReadByte(ACCEL_ADR, 0x27);
    Stat2 = ReadByte(ACCEL_ADR, 0x07);
    delay(1); //DEBUG!
  }
  if((millis() - LocalTime) < GlobalTimeout) AccelFail = true; //Set flag if timeout occoured 
  else AccelFail = false;

  // si.i2c_read(false);
  // si.i2c_read(false);
  // si.i2c_read(false); //DEBUG!
  // si.i2c_read(false);
  // si.i2c_read(false);
  delay(5); //DEBUG!
  // while(((ReadByte(ACCEL_ADR, 0x27) & 0x08) >> 3) != 1 || (digitalRead(7) == LOW)); //Wait for updated values

  // Serial.println("START"); //DEBUG!
  // Serial.println(Stat1, BIN); //DEBUG! 
  // Serial.println(Stat2, BIN); //DEBUG!
  // Serial.print("\n\n"); //Newline return

  int16_t Range = GetRange();  //DEBUG! Replace!
  Serial.print('R'); //Preceed range value
  Serial.println(Range); 

  Reg[0] = 0; // Not ready: cutting power to LiDAR
  digitalWrite(ENABLE, LOW);
  digitalWrite(POWER_SW, LOW); //Turn off 5v switched power
  GetOffsets(); //Read in offsets
  GetG(true);
  // Serial.println(ReadByte(LIDAR_ADR, 0x0E)); //DEBUG! //READ RSSI

  // for(int i = 0; i < 3; i++) {
  //  Serial.println(GetG(i));
  // }
  // Serial.println(ReadByte(ACCEL_ADR, 0x27), BIN); //DEBUG! 
  
  // delay(1000);
  while((millis() - StartTime) < Period) {  //Wait for period rollover 
    set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here
    sleep_enable();
    sei();
    sleep_cpu();
    

    if(!digitalRead(HALL_SWITCH) && !SwitchLatch) {  //Only run update if switch is not lauched previously (new application of trigger)
      SwitchLatch = true; //latch switch until toggle of state
      digitalWrite(STAT_LED, HIGH); //Turn on status LED while latched 
      GetG(false); //Get new acclerometer values
      UpdateOffset(AccelVals);
    }
    if(digitalRead(HALL_SWITCH)) {
      SwitchLatch = false; //If switch is high, back to default state, reset latch 
      digitalWrite(STAT_LED, LOW); //Turn off stat LED once latch is cleared 
    }
    // if(Serial.available() > 0) {  //FIX add serial control??
    //  uint8_t Data1 = Serial.read();
    //  uint8_t Data2 = Serial.read();
    //  if(Data2 == 'F') Ctrl = Data1; //If 
    // }
  }
  sleep_disable();
  digitalWrite(POWER_SW, HIGH); //Turn on 5v switched power
  delay(100); //Wait for voltage to stabilize after cap charge 
  digitalWrite(ENABLE, HIGH); //NOTE: MUST toggle enable after voltage ramp to ensure effective measurment 
  // while(Serial.available() < 1 && digitalRead())
}

// float GetAngle(uint8_t Axis)
// {
//  float ValX = GetG(0); //Used to get g values
//  float ValY = GetG(1);
//  float ValZ = GetG(2);
//   float Val = 0;
//   switch(Axis) {
//     case(0):
//       Val = asin(ValX); 
//       break;
//     case(1):
//       Val = asin(ValY);
//       break;
//     case(2):
//       Val = acos(ValZ);
//       break;
//     case(3):
//       Val = atan(ValX/(sqrt(pow(ValY, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return pitch angle
//       break;
//     case(4):
//       Val = atan(ValY/(sqrt(pow(ValX, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return roll angle
//       break;
//   }
//   if(ValX == ValY && ValX == ValZ) Val = -9999; //Return error value is all vals are the same (1 in 6.87x10^10 likelyhood of occouring without error)
//   return Val; 
// }

uint8_t InitAccel() 
{
  // WriteByte(ACCEL_ADR, CTRL_REG1_ADR, 0x07);
  WriteByte(ACCEL_ADR, CTRL_REG1_ADR, 0x77); //Set for 100Hz output data rate //FIX! Set to low power initally??
  WriteByte(ACCEL_ADR, CTRL_REG4_ADR, 0x88); //Turn on high resolution mode //FIX! Setup to use self text
  WriteByte(ACCEL_ADR, CTRL_REG3_ADR, 0x10);
  WriteByte(ACCEL_ADR, TEMP_CFG_REG_ADR, 0x80);
}

float GetG(bool Set)  //FIX! Add offset support //By default set/send data to registers 
{ 
  // uint8_t AxisADR = OUT_X_ADR + 2*Axis; //Add appropriate offset
  // int16_t Data = ReadWord(ACCEL_ADR, AxisADR);
  // return Data*(4.0/4096.0); //FIX! Make fixed integer! 
  // Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool OutOfRange = false; //Used to test if values are within expected range
  int16_t Axis[3] = {0}; //Initalize variables for x,y,z values

  bool Error = SendCommand(ACCEL_ADR, OUT_X_ADR | 0x80);
  si.i2c_stop(); 

  uint8_t Data[6] = {0}; //Init data
  si.i2c_start((ACCEL_ADR << 1) | READ);
  for(int i = 0; i < 6; i++) {
    Data[i] = si.i2c_read(false);
  }
  si.i2c_stop();
  for(int i = 0; i < 3; i++) {
    Axis[i] = (((int16_t)(Data[2*i + 1] << 8) | (int16_t)Data[2*i]) >> 4);
  }

  if(Error == false || OutOfRange) {  //If read error occours 
    AccelFail = true; //Set flag
    Axis[0] = -9999;
    Axis[1] = -9999;
    Axis[2] = -9999;
  }
  else {  //Otherwise load/send data normally 
    AccelFail = false; //Clear flag
    for(int i = 0; i < 3; i++) AccelVals[i] = Axis[i]; //Copy local raw axis data to accel vals

    if(Set) {  //If sending data is commanded, print data out
      Serial.print('X'); Serial.println(Axis[0] - Offsets[0]);  //FIX! Optimize to prevent multiple addition 
      Serial.print('Y'); Serial.println(Axis[1] - Offsets[1]);
      Serial.print('Z'); Serial.println(Axis[2] - Offsets[2]);

      SplitAndLoad(0x04, Axis[0]);  //Load accel values
      SplitAndLoad(0x06, Axis[1]);
      SplitAndLoad(0x08, Axis[2]);

      SplitAndLoad(0x0A, Offsets[0]);  //Load offsets
      SplitAndLoad(0x0C, Offsets[1]);
      SplitAndLoad(0x0E, Offsets[2]);
    }
  }


  // return Data;
}

uint8_t InitLiDAR() 
{
  // WriteByte(LIDAR_ADR, 0x02, 0x80);
  // WriteByte(LIDAR_ADR, 0x04, 0x08);
  // WriteByte(LIDAR_ADR, 0x12, 0x05);
  // WriteByte(LIDAR_ADR, 0x1C, 0x00);
  uint8_t SigCountMax = 0;
  uint8_t AcqConfigReg = 0;
  uint8_t RefCountMax = 0;
  uint8_t ThresholdBypass = 0;
  switch(LiDAR_Config) {
    case 0: //Default, ballanced
      SigCountMax = 0x80;
      AcqConfigReg = 0x08;
      RefCountMax = 0x05;
      ThresholdBypass = 0x00;
      break;
    case 1: //High sensitivity
      SigCountMax = 0x80;
      AcqConfigReg = 0x08;
      RefCountMax = 0x05;
      ThresholdBypass = 0x80;
      break;
    case 2: //Low sensitivity
      SigCountMax = 0x80;
      AcqConfigReg = 0x08;
      RefCountMax = 0x05;
      ThresholdBypass = 0xB0;
      break;
    case 3: //Max range
      SigCountMax = 0xFF;
      AcqConfigReg = 0x08;
      RefCountMax = 0x05;
      ThresholdBypass = 0x00;
      break;
  }
  WriteByte(LIDAR_ADR, 0x02, SigCountMax);
  WriteByte(LIDAR_ADR, 0x04, AcqConfigReg | 0x01);  //Setup MODE pin to indicate satus 
  WriteByte(LIDAR_ADR, 0x12, RefCountMax);
  WriteByte(LIDAR_ADR, 0x1C, ThresholdBypass);
} 

int16_t GetRange()  //FIX! add range constraint??
{
  int16_t Data = 0; //Used to store results
  WriteByte(LIDAR_ADR, 0x00, 0x01);
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x00); 
  // si.i2c_stop();
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x01); //Command to take measurment WITH correction bias 
  // si.i2c_stop();
  unsigned long LocalTime = millis();
  // while((ReadByte(LIDAR_ADR, 0x01) & 0x01) == 1 && (millis() - LocalTime) < GlobalTimeout && digitalRead(MODE_READ) == LOW); //Wait for updated value or timeout
  while((millis() - LocalTime) < GlobalTimeout && digitalRead(MODE_READ) == LOW); //Wait for updated value or timeout
  if((millis() - LocalTime) < GlobalTimeout) {  //If timeout has NOT occoured, read as normal
    Data = ReadWord_LE(LIDAR_ADR, 0x0F);
    SplitAndLoad(0x02, Data);
    LidarFail = false;  //Clear failure flag
  }
  else {  //Otherwise set failure flag and set out of range data value
    LidarFail = true;
    Data = -9999; 
    SplitAndLoad(0x02, Data);
  }

  return Data;

}

uint8_t SendCommand(uint8_t Adr, uint8_t Command)  //FIX! Fix error return!
{
    si.i2c_start((Adr << 1) | WRITE);
    bool Error = si.i2c_write(Command);
    return Error; //DEBUG!
}

uint8_t WriteWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  si.i2c_write(Data & 0xFF); //Write LSB
  uint8_t Error = si.i2c_write((Data >> 8) & 0xFF); //Write MSB
  si.i2c_stop();
  return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteByte(uint8_t Adr, uint8_t Command, uint8_t Data)  //Writes value to 16 bit register
{
  Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  uint8_t Error = si.i2c_write((Data) & 0xFF); //Write MSB
  si.i2c_stop();
  return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteWord_LE(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  si.i2c_write((Data >> 8) & 0xFF); //Write MSB
  si.i2c_write(Data & 0xFF); //Write LSB
  si.i2c_stop();
  // return Error;  //Invert error so that it will return 0 is works
}

// uint8_t WriteConfig(uint8_t Adr, uint8_t NewConfig)
// {
//  si.i2c_start((Adr << 1) | WRITE);
//  si.i2c_write(CONF_CMD);  //Write command code to Config register
//  uint8_t Error = si.i2c_write(NewConfig);
//  si.i2c_stop();
//  if(Error == true) {
//    Config = NewConfig; //Set global config if write was sucessful 
//    return 0;
//  }
//  else return -1; //If write failed, return failure condition
// }

int ReadByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
  bool Error = SendCommand(Adr, Command);
  si.i2c_rep_start((Adr << 1) | READ);
  uint8_t ValLow = si.i2c_read(false);
  uint8_t ValHigh = si.i2c_read(false);
  si.i2c_stop();
  Error = true; //DEBUG!
  if(Error == true) {
    if(Pos == 0) return ValLow;
    if(Pos == 1) return ValHigh;
  }
  else return -1; //Return error if read failed

}

int ReadByte(uint8_t Adr, uint8_t Command) //Send command value, and high/low byte to read, returns desired byte
{
  Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool Error = SendCommand(Adr, Command);
  si.i2c_stop(); //DEBUG!
  si.i2c_start((Adr << 1) | READ);
  uint8_t Val = si.i2c_read(true);  //DEBUG! origionally false 
  // uint8_t ValHigh = si.i2c_read(false);
  si.i2c_stop();
  Error = true; //DEBUG!
  if(Error == true) {
    return Val; //DEBUG!
  //  if(Pos == 0) return ValLow;
  //  if(Pos == 1) return ValHigh;
  }
  else return -1; //Return error if read failed

}

int16_t ReadWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
  // Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool Error = SendCommand(Adr, Command);
  si.i2c_stop(); 
  // Serial.print("Error = "); Serial.println(Error); //DEBUG!
  // uint8_t Data[6] = {0}; //Init data
  si.i2c_start((Adr << 1) | READ);

  uint8_t ByteLow = si.i2c_read(false);  //Read in high and low bytes (big endian)
  uint8_t ByteHigh = si.i2c_read(false);
  si.i2c_stop();
  // if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
  // else return -1; //Return error if read failed
  return ((int16_t)(ByteHigh << 8) | (int16_t)ByteLow); //DEBUG!  //FIX! Right shift?? 
}

int ReadWord_LE(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
  bool Error = SendCommand(Adr, Command);
  si.i2c_stop();
  si.i2c_start((Adr << 1) | READ);
  uint8_t ByteHigh = (int8_t) si.i2c_read(false);  //Read in high and low bytes (big endian)
  uint8_t ByteLow = (int8_t) si.i2c_read(false);
  si.i2c_stop();
  // if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
  // else return -1; //Return error if read failed
  return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

void SplitAndLoad(uint8_t Pos, int16_t Val) //Write 16 bits
{
  uint8_t Len = sizeof(Val);
  for(int i = Pos; i < Pos + Len; i++) {
    Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
  }
}

void SplitAndLoad(uint8_t Pos, long Val)  //Write 32 bits
{
  uint8_t Len = sizeof(Val);
  for(int i = Pos; i < Pos + Len; i++) {
    Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
  }
}

boolean addressEvent(uint16_t address, uint8_t count)
{
  RepeatedStart = (count > 0 ? true : false);
  return true; // send ACK to master
}

void requestEvent()
{ 
  //Allow for repeated start condition 
  if(RepeatedStart) {
    for(int i = 0; i < 2; i++) {
      Wire.write(Reg[RegID + i]);
    }
  }
  else {
    Wire.write(Reg[RegID]);
  }
}

void receiveEvent(int DataLen) 
{
    //Write data to appropriate location
    if(DataLen == 2){
      //Remove while loop?? 
      while(Wire.available() < 2); //Only option for writing would be register address, and single 8 bit value
      uint8_t Pos = Wire.read();
      uint8_t Val = Wire.read();
      //Check for validity of write??
      Reg[Pos] = Val; //Set register value
  }

  if(DataLen == 1){
    RegID = Wire.read(); //Read in the register ID to be used for subsequent read
  }
}

void stopEvent() 
{
  StopFlag = true;
  //End comunication
}

void UpdateOffset(int16_t *AxisData)  //Pass in array of X,Y,Z offset values
{
  // uint8_t Val[4] = {0}; //Blank array to use as temporary storage for desconsturcted float
  // for(int i = 0; i < 3; i++) {
  //  memcpy(Val, &AxisData[i], sizeof(float)); //Deconstruct the ith axis value into the temprary Val register 
  //  for(int p = 0; p < 4; p++) {
  //    EEPROM.write(Val[p], p + i); //Write from desired entry in EEPROM (the pth entry of the ith 4 byte float)
  //  }
  // }
  for(int i = 0; i < 3; i++) {
    // ((EEPROM.read(p + i) << 8) | EEPROM.read(2*i + 1)); //Read from desired entry in EEPROM and concatonate
    EEPROM.write(2*i, AxisData[i] >> 8);  //Write MSB
    EEPROM.write(2*i + 1, AxisData[i] & 0xFF);  //Write LSB
  }
}

void GetOffsets()
{
  //Float implementation
  // uint8_t Val[4] = {0}; //Blank array to read bytes into which can be converted to single float
  // for(int i = 0; i < 3; i++) {
  //  for(int p = 0; p < 4; p++) {
  //    Val[p] = EEPROM.read(p + i); //Read from desired entry in EEPROM (the pth entry of the ith 4 byte float)
  //  }
  //  memcpy(&Offsets[i], &Val, sizeof(float)); //Load the 4 discrete bytes back into the ith offset float
  // }

  // uint8_t Val[4] = {0}; //Blank array to read bytes into which can be converted to single float
  for(int i = 0; i < 3; i++) {
      Offsets[i] = (int)((EEPROM.read(2*i) << 8) | EEPROM.read(2*i + 1)); //Read from desired entry in EEPROM and concatonate
    // memcpy(&Offsets[i], &Val, sizeof(float)); //Load the 4 discrete bytes back into the ith offset float
  }
}

// void ResetOffset()  //Set offset back to zero values
// {
//  for(int i = 0; i < 12; i++) {
//    EEPROM.write(i) = 0; //Clear all utilized EEPROM values
//  }
// }
