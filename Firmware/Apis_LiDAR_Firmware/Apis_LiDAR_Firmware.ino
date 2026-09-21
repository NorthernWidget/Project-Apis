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

// Firmware and hardware version constants.
// HW major.minor = PCB revision (v0.1: first fab run 2019-08-14, re-ordered 2023-05-01).
// FW patch = firmware revision; bump on any behavioral change visible to the library.
#define FW_HW_MAJOR 0
#define FW_HW_MINOR 1
#define FW_FW_PATCH 0

// #define ADR_ALT 0x41 //Alternative device address

const unsigned long timeoutGlobal = 200; //Time to wait before timing out

bool lidarFail = false; //Used to indicate failure of Lidar unit
bool accelFail = false; //Used to indicate failure of on board accelerometer 

volatile uint8_t adr = 0x50; //Use arbitraty address, change using generall call??
// const uint8_t ADR_Alt = 0x41; //Alternative device address  //WARNING! When a #define is used instead, problems are caused
// NOTE: Switching to 0x41 via a solder jumper requires a board revision to
// add address-selection hardware; no such circuit exists in the current design
// (JP1 is the MIC2544 current-limit jumper, not an address jumper).
// EEPROM byte 6 holds a persistent I2C address (written via register 0x0C);
// read in setup() before Wire.begin(). Falls back to 0x50 if 0xFF (erased).

unsigned int config = 0; //Global config value
unsigned long period = 100; //Number of ms between sample events for continuious running

// I2C register map (32 bytes, indices 0x00–0x1F).
// Registers not listed are reserved and zero-initialised.
//   0x00        Status flags. Bit 0 = ready: 0 = booting/LiDAR not yet
//               initialised, 1 = ready. Set to 0 at startup and whenever power
//               is cut to the LiDAR; set to 1 after initLiDAR() completes.
//               The library polls bit 0 (up to 150 ms) so it can exit as soon
//               as the LiDAR is ready rather than waiting a fixed time.
//               See: github.com/NorthernWidget/Project-Apis/issues/15
//   0x01–0x04   ASCII device name: 'A','p','i','s' (statically initialised;
//               read by library begin() to detect old firmware)
//   0x05        Hardware version major (FW_HW_MAJOR)
//   0x06        Hardware version minor (FW_HW_MINOR)
//   0x07        Firmware patch version (FW_FW_PATCH)
//   0x08–0x09   Range [cm], little-endian int16
//   0x0A        LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
//   0x0B        config: sensitivity mode bits [1:0], writable by master
//   0x0C        I2C address, writable; saved to EEPROM byte 6 on write,
//               takes effect on next boot; falls back to 0x50 if 0xFF
//   0x10–0x15   Accelerometer X, Y, Z raw, little-endian int16 each
//   0x18–0x1D   Accelerometer offsets X, Y, Z, little-endian int16 each
uint8_t reg[32] = {
  0,                         // 0x00: status (not ready)
  'A', 'p', 'i', 's',        // 0x01–0x04: device name
  FW_HW_MAJOR, FW_HW_MINOR,  // 0x05–0x06: hardware version
  FW_FW_PATCH                 // 0x07: firmware patch version
  // 0x08–0x1F: zero-initialised; measurements updated at runtime
};
// bool startReading = true; //Flag used to start a new converstion, make a conversion on startup
// const unsigned int updateRate = 5; //Rate of update

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_A2, PIN_A3, true);  //Initialize software I2C

volatile bool stopFlag = false; //Used to indicate a stop condition 
volatile uint8_t regID = 0; //Used to denote which register will be read from
volatile bool repeatedStart = false; //Used to show if the start was repeated or not

int16_t offsets[3] = {0};  //X,Y,Z acceleration offsets to zero the angle of the device 
int16_t accelVals[3] = {0}; //Global storage for acceleration data values to be shared between EEPROM functions and getter functions 

bool switchLatch = false;  //Latching functionality control for Hall effect switch 
uint8_t lidarConfig = 0; //Use default config 

void setup() {
  // Serial.begin(115200); //DEBUG!
  // Serial.println("begin"); //DEBUG!
  // pinMode(ADR_SEL_PIN, INPUT_PULLUP);
  // if(!digitalRead(ADR_SEL_PIN)) adr = ADR_Alt; //If solder jumper is bridged, use alternate address //DEBUG!
  // NOTE: ADR_SEL_PIN is not defined or wired in the current board revision.
  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, HIGH);
  pinMode(POWER_SW, OUTPUT);
  // digitalWrite(POWER_SW, HIGH); //Turn on power //DEBUG!
  // delay(500); //DEBUG!
  digitalWrite(POWER_SW, LOW); //Turn off output power //FIX??
  uint8_t storedAdr = EEPROM.read(6); // Persistent I2C address; 0xFF = not set
  if (storedAdr != 0xFF) adr = storedAdr;
  Wire.begin(adr);  //Begin slave I2C
  Serial.begin(9600);
  // Serial.println("START"); //DEBUG!
  // EEPROM.write(0, adr);

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

  reg[0] = 0; // Not ready: LiDAR not yet initialised
  delay(10);
  digitalWrite(POWER_SW, HIGH); // Turn on power; 680 µF cap charges at ~227 mA
  delay(100); // Wait for cap charge (~15 ms) and LiDAR Lite power-on (~22 ms)
  digitalWrite(ENABLE, HIGH);
  si.i2c_init(); //Begin I2C master
  initAccel();
  initLiDAR();
  reg[0] = 1; // Ready: LiDAR initialised and accepting I2C commands
  digitalWrite(STAT_LED, LOW);  //Blink on statup
  if(!digitalRead(HALL_SWITCH)) {
    updateOffset(offsets); //Clear values (offsets are 0 on startup until read into)
    switchLatch = true; //Set latch to prevent override 
  }
  digitalWrite(STAT_LED, HIGH);

}

void loop() {
  // static unsigned int Count = 0; //Counter to determine update rate
  // if(startReading == true) {
  //  //Read new values in
  //  autoRangeVis();  //Run auto range
  //  delay(800); //Wait for new sample
  //  splitAndLoad(0x0B, GetALS()); //Load ALS value
  //  splitAndLoad(0x0D, GetWhite()); //Load white value
  //  splitAndLoad(0x02, long(GetUV(0))); //Load UVA
  //  splitAndLoad(0x07, long(GetUV(1))); //Load UVB
  //  splitAndLoad(0x10, GetLuxGain()); //Load lux multiplier 
  //  splitAndLoad(0x13, GetADC(0));
  //  splitAndLoad(0x15, GetADC(1));
  //  splitAndLoad(0x17, GetADC(2));

  //  startReading = false; //Clear flag when new values updated  
  // }
  // if(Count++ == updateRate) {  //Fix update method??
  //  startReading = true; //Set flag if number of updates have rolled over 
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
  // readByte(ACCEL_ADR, 0x27);
  // readWord(ACCEL_ADR, OUT_X_ADR);
  lidarConfig = reg[0x0B] & 0x03; //Pull low two bits from config reg (0x0B) to get Lidar configuration state
  initLiDAR(); //reinitialize LiDAR after power cycle
  reg[0] = 1; // Ready: LiDAR initialised and accepting I2C commands
  unsigned long StartTime = millis();  //Measure time from start of measurment 
  uint8_t Stat1 = readByte(ACCEL_ADR, 0x27); 
  uint8_t Stat2 = readByte(ACCEL_ADR, 0x07);
  // while(((Stat1 & 0x08) >> 3) != 1 || ((Stat2 & 0x08) >> 3) != 1 || ((Stat2 & 0x80) >> 7) != 1) {
  unsigned long LocalTime = millis();
  while(((Stat1 & 0x08) >> 3) != 1 || Stat2 != 0xFF && (millis() - LocalTime) < timeoutGlobal) {  //Try to get status from 
    Stat1 = readByte(ACCEL_ADR, 0x27);
    Stat2 = readByte(ACCEL_ADR, 0x07);
    delay(1); //DEBUG!
  }
  if((millis() - LocalTime) < timeoutGlobal) accelFail = true; //Set flag if timeout occoured 
  else accelFail = false;

  // si.i2c_read(false);
  // si.i2c_read(false);
  // si.i2c_read(false); //DEBUG!
  // si.i2c_read(false);
  // si.i2c_read(false);
  delay(5); //DEBUG!
  // while(((readByte(ACCEL_ADR, 0x27) & 0x08) >> 3) != 1 || (digitalRead(7) == LOW)); //Wait for updated values

  // Serial.println("START"); //DEBUG!
  // Serial.println(Stat1, BIN); //DEBUG! 
  // Serial.println(Stat2, BIN); //DEBUG!
  // Serial.print("\n\n"); //Newline return

  int16_t Range = getRange();  //DEBUG! Replace!
  Serial.print('R'); //Preceed range value
  Serial.println(Range); 

  reg[0] = 0; // Not ready: cutting power to LiDAR
  digitalWrite(ENABLE, LOW);
  digitalWrite(POWER_SW, LOW); //Turn off 5v switched power
  getOffsets(); //Read in offsets
  getG(true);
  // Serial.println(readByte(LIDAR_ADR, 0x0E)); //DEBUG! //READ RSSI

  // for(int i = 0; i < 3; i++) {
  //  Serial.println(getG(i));
  // }
  // Serial.println(readByte(ACCEL_ADR, 0x27), BIN); //DEBUG! 
  
  // delay(1000);
  while((millis() - StartTime) < period) {  //Wait for period rollover 
    set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here
    sleep_enable();
    sei();
    sleep_cpu();
    

    if(!digitalRead(HALL_SWITCH) && !switchLatch) {  //Only run update if switch is not lauched previously (new application of trigger)
      switchLatch = true; //latch switch until toggle of state
      digitalWrite(STAT_LED, HIGH); //Turn on status LED while latched 
      getG(false); //Get new acclerometer values
      updateOffset(accelVals);
    }
    if(digitalRead(HALL_SWITCH)) {
      switchLatch = false; //If switch is high, back to default state, reset latch 
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

// float getAngle(uint8_t Axis)
// {
//  float ValX = getG(0); //Used to get g values
//  float ValY = getG(1);
//  float ValZ = getG(2);
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

uint8_t initAccel() 
{
  // writeByte(ACCEL_ADR, CTRL_REG1_ADR, 0x07);
  writeByte(ACCEL_ADR, CTRL_REG1_ADR, 0x77); //Set for 100Hz output data rate //FIX! Set to low power initally??
  writeByte(ACCEL_ADR, CTRL_REG4_ADR, 0x88); //Turn on high resolution mode //FIX! Setup to use self text
  writeByte(ACCEL_ADR, CTRL_REG3_ADR, 0x10);
  writeByte(ACCEL_ADR, TEMP_CFG_REG_ADR, 0x80);
}

float getG(bool Set)  //FIX! Add offset support //By default set/send data to registers 
{ 
  // uint8_t AxisADR = OUT_X_ADR + 2*Axis; //Add appropriate offset
  // int16_t Data = readWord(ACCEL_ADR, AxisADR);
  // return Data*(4.0/4096.0); //FIX! Make fixed integer! 
  // Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool OutOfRange = false; //Used to test if values are within expected range
  int16_t Axis[3] = {0}; //Initalize variables for x,y,z values

  bool Error = sendCommand(ACCEL_ADR, OUT_X_ADR | 0x80);
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
    accelFail = true; //Set flag
    Axis[0] = -9999;
    Axis[1] = -9999;
    Axis[2] = -9999;
  }
  else {  //Otherwise load/send data normally 
    accelFail = false; //Clear flag
    for(int i = 0; i < 3; i++) accelVals[i] = Axis[i]; //Copy local raw axis data to accel vals

    if(Set) {  //If sending data is commanded, print data out
      Serial.print('X'); Serial.println(Axis[0] - offsets[0]);  //FIX! Optimize to prevent multiple addition 
      Serial.print('Y'); Serial.println(Axis[1] - offsets[1]);
      Serial.print('Z'); Serial.println(Axis[2] - offsets[2]);

      splitAndLoad(0x10, Axis[0]);  //Load accel values
      splitAndLoad(0x12, Axis[1]);
      splitAndLoad(0x14, Axis[2]);

      splitAndLoad(0x18, offsets[0]);  //Load offsets
      splitAndLoad(0x1A, offsets[1]);
      splitAndLoad(0x1C, offsets[2]);
    }
  }


  // return Data;
}

uint8_t initLiDAR() 
{
  // writeByte(LIDAR_ADR, 0x02, 0x80);
  // writeByte(LIDAR_ADR, 0x04, 0x08);
  // writeByte(LIDAR_ADR, 0x12, 0x05);
  // writeByte(LIDAR_ADR, 0x1C, 0x00);
  uint8_t SigCountMax = 0;
  uint8_t AcqConfigReg = 0;
  uint8_t RefCountMax = 0;
  uint8_t ThresholdBypass = 0;
  switch(lidarConfig) {
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
  writeByte(LIDAR_ADR, 0x02, SigCountMax);
  writeByte(LIDAR_ADR, 0x04, AcqConfigReg | 0x01);  //Setup MODE pin to indicate satus 
  writeByte(LIDAR_ADR, 0x12, RefCountMax);
  writeByte(LIDAR_ADR, 0x1C, ThresholdBypass);
} 

int16_t getRange()  //FIX! add range constraint??
{
  int16_t Data = 0; //Used to store results
  writeByte(LIDAR_ADR, 0x00, 0x01);
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x00); 
  // si.i2c_stop();
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x01); //Command to take measurment WITH correction bias 
  // si.i2c_stop();
  unsigned long LocalTime = millis();
  // while((readByte(LIDAR_ADR, 0x01) & 0x01) == 1 && (millis() - LocalTime) < timeoutGlobal && digitalRead(MODE_READ) == LOW); //Wait for updated value or timeout
  while((millis() - LocalTime) < timeoutGlobal && digitalRead(MODE_READ) == LOW); //Wait for updated value or timeout
  if((millis() - LocalTime) < timeoutGlobal) {  //If timeout has NOT occoured, read as normal
    Data = readWordLE(LIDAR_ADR, 0x0F);
    splitAndLoad(0x08, Data);
    // Read signal strength from LiDAR Lite reg 0x0E directly (no auto-increment bit)
    sendCommand(LIDAR_ADR, 0x0E);
    si.i2c_stop();
    si.i2c_start((LIDAR_ADR << 1) | READ);
    reg[0x0A] = si.i2c_read(false);
    si.i2c_stop();
    lidarFail = false;  //Clear failure flag
  }
  else {  //Otherwise set failure flag and set out of range data value
    lidarFail = true;
    Data = -9999;
    splitAndLoad(0x08, Data);
    reg[0x0A] = 0;
  }

  return Data;

}

uint8_t sendCommand(uint8_t Adr, uint8_t Command)  //FIX! Fix error return!
{
    si.i2c_start((Adr << 1) | WRITE);
    bool Error = si.i2c_write(Command);
    return Error; //DEBUG!
}

uint8_t writeWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  si.i2c_write(Data & 0xFF); //Write LSB
  uint8_t Error = si.i2c_write((Data >> 8) & 0xFF); //Write MSB
  si.i2c_stop();
  return Error;  //Invert error so that it will return 0 is works
}

uint8_t writeByte(uint8_t Adr, uint8_t Command, uint8_t Data)  //Writes value to 16 bit register
{
  Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  uint8_t Error = si.i2c_write((Data) & 0xFF); //Write MSB
  si.i2c_stop();
  return Error;  //Invert error so that it will return 0 is works
}

uint8_t writeWordLE(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
  si.i2c_start((Adr << 1) | WRITE);
  si.i2c_write(Command); //Write Command value
  si.i2c_write((Data >> 8) & 0xFF); //Write MSB
  si.i2c_write(Data & 0xFF); //Write LSB
  si.i2c_stop();
  // return Error;  //Invert error so that it will return 0 is works
}

// uint8_t writeConfig(uint8_t Adr, uint8_t NewConfig)
// {
//  si.i2c_start((Adr << 1) | WRITE);
//  si.i2c_write(CONF_CMD);  //Write command code to config register
//  uint8_t Error = si.i2c_write(NewConfig);
//  si.i2c_stop();
//  if(Error == true) {
//    config = NewConfig; //Set global config if write was sucessful 
//    return 0;
//  }
//  else return -1; //If write failed, return failure condition
// }

int readByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
  bool Error = sendCommand(Adr, Command);
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

int readByte(uint8_t Adr, uint8_t Command) //Send command value, and high/low byte to read, returns desired byte
{
  Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool Error = sendCommand(Adr, Command);
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

int16_t readWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
  // Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
  bool Error = sendCommand(Adr, Command);
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

int readWordLE(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
  bool Error = sendCommand(Adr, Command);
  si.i2c_stop();
  si.i2c_start((Adr << 1) | READ);
  uint8_t ByteHigh = (int8_t) si.i2c_read(false);  //Read in high and low bytes (big endian)
  uint8_t ByteLow = (int8_t) si.i2c_read(false);
  si.i2c_stop();
  // if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
  // else return -1; //Return error if read failed
  return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

void splitAndLoad(uint8_t Pos, int16_t Val) //Write 16 bits
{
  uint8_t Len = sizeof(Val);
  for(int i = Pos; i < Pos + Len; i++) {
    reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
  }
}

void splitAndLoad(uint8_t Pos, long Val)  //Write 32 bits
{
  uint8_t Len = sizeof(Val);
  for(int i = Pos; i < Pos + Len; i++) {
    reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
  }
}

boolean addressEvent(uint16_t address, uint8_t count)
{
  repeatedStart = (count > 0 ? true : false);
  return true; // send ACK to master
}

void requestEvent()
{ 
  //Allow for repeated start condition 
  if(repeatedStart) {
    for(int i = 0; i < 2; i++) {
      Wire.write(reg[regID + i]);
    }
  }
  else {
    Wire.write(reg[regID]);
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
      reg[Pos] = Val; //Set register value
      if (Pos == 0x0C) EEPROM.write(6, Val); //Persist I2C address; takes effect on next boot
  }

  if(DataLen == 1){
    regID = Wire.read(); //Read in the register ID to be used for subsequent read
  }
}

void stopEvent() 
{
  stopFlag = true;
  //End comunication
}

void updateOffset(int16_t *AxisData)  //Pass in array of X,Y,Z offset values
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

void getOffsets()
{
  //Float implementation
  // uint8_t Val[4] = {0}; //Blank array to read bytes into which can be converted to single float
  // for(int i = 0; i < 3; i++) {
  //  for(int p = 0; p < 4; p++) {
  //    Val[p] = EEPROM.read(p + i); //Read from desired entry in EEPROM (the pth entry of the ith 4 byte float)
  //  }
  //  memcpy(&offsets[i], &Val, sizeof(float)); //Load the 4 discrete bytes back into the ith offset float
  // }

  // uint8_t Val[4] = {0}; //Blank array to read bytes into which can be converted to single float
  for(int i = 0; i < 3; i++) {
      offsets[i] = (int)((EEPROM.read(2*i) << 8) | EEPROM.read(2*i + 1)); //Read from desired entry in EEPROM and concatonate
    // memcpy(&offsets[i], &Val, sizeof(float)); //Load the 4 discrete bytes back into the ith offset float
  }
}

// void resetOffset()  //Set offset back to zero values
// {
//  for(int i = 0; i < 12; i++) {
//    EEPROM.write(i) = 0; //Clear all utilized EEPROM values
//  }
// }
