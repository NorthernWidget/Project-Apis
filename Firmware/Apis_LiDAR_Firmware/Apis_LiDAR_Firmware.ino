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
#define OUT_ADC3_ADR 0x0C //Auxiliary ADC channel 3, low byte: the temperature sensor when TEMP_EN is set
#define STATUS_REG_ADR 0x27
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

// Serial output (range and accelerometer axes per reading) only in debug builds:
// TX/RX are not used in normal operation (ICSP-only), and the prints cost ~25 ms
// per reading at 9600 baud.
// #define APIS_DEBUG

#define READ 0x01
#define WRITE 0x00

#define BUF_LENABLEGTH 64 //Length of I2C Buffer, verify with documentation

// Firmware patch version: bump on any behavioural change visible to the
// library. The hardware version lives in Page 0 (EEPROM), written at
// provisioning; the firmware writes this constant into the served copy of
// Page 0 at 0x0A and recomputes the CRC there (NW-Device-Specification).
#define FW_FW_PATCH 5

// The stored pages are the top 64 bytes of EEPROM in bus order: Page 0
// (identity, 32 bytes) at 0xC0-0xDF on the ATtiny1634's 256-byte EEPROM,
// written once by NW-Provision and read at boot; Page 1 (calibration)
// directly above it at 0xE0-0xFF, in bus order (little-endian words), served
// byte for byte: Block 0 the current zero (offsets X, Y, Z and the
// temperature word), Blocks 1 and 2 the two zeros before it, 0x38-0x39 the
// zero generation (patch 5). A blank word (0xFFFF) is served as 0.
#define PAGE0_BASE   (E2END + 1 - 64)
#define REG_I2C_ADDR 0x1F
#define PAGE1_BASE   (E2END + 1 - 32)
#define ADR_DEFAULT  0x41   // Schema 1 'A'; used when Page 0 byte 0x1F is 0xFF

// #define ADR_ALT 0x41 //Alternative device address

const unsigned long timeoutGlobal = 200; //Time to wait before timing out

bool lidarFail = false; //Used to indicate failure of Lidar unit
bool accelFail = false; //Used to indicate failure of on board accelerometer 

volatile uint8_t adr = ADR_DEFAULT; //I2C address: Page 0 byte 0x1F (EEPROM), or ADR_DEFAULT if unprogrammed
// const uint8_t ADR_Alt = 0x41; //Alternative device address  //WARNING! When a #define is used instead, problems are caused
// NOTE: Switching to 0x41 via a solder jumper requires a board revision to
// add address-selection hardware; no such circuit exists in the current design
// (JP1 is the MIC2544 current-limit jumper, not an address jumper).
// The I2C address is Page 0 byte 0x1F (EEPROM 0xDF), written via register
// 0x1F; read in setup() before Wire.begin(). Falls back to ADR_DEFAULT if 0xFF.

unsigned int config = 0; //Global config value
// On-demand run model (NW-Device-Specification, Apis appendix): the unit idles until a
// trigger. The LiDAR is powered only while readings are being taken, per the
// readings-requested word (0x44-0x45): up at the first trigger, down when the
// requested count is done. No idle timer; a batch that stalls is abandoned.
bool lidarOn = false; //LiDAR powered and configured
uint16_t requested = 0; //readings-requested word, latched at the first trigger after a write
uint16_t requestBase = 0; //reading counter when the request was latched (internal, not a register)
volatile bool requestWritten = false; //set by receiveEvent() on a write to 0x44/0x45
unsigned long lidarLastReading = 0; //millis() of the last reading while powered
const unsigned long lidarBatchTimeout = 2000; // ms without a trigger before a batch is abandoned (fault fallback)
uint8_t lidarConfigApplied = 0xFF; //Config bits last written to the LiDAR
uint8_t lidarInitFail = 0; //fault code if the last power-up failed, else 0
const unsigned long lidarRailMs = 20; //rail ramp before enable: 680 uF via the MIC2544 at ~227 mA is ~15 ms
const unsigned long lidarBootTimeout = 100; // ms to wait for ACK + health after enable (manual: ~22 ms)

// Page 0 (0x00–0x1F) identity is copied from EEPROM at boot (loadPage0);
// Page 1 (0x20–0x3F) calibration and Page 2 (0x40–0x5F) sensor data per
// NW-Device-Specification Schema 1 (Apis appendix; pages renumbered
// 2026-09-23, spec 4c3b18d: calibration is Page 1, data Page 2).
//   0x20–0x25   Accelerometer offsets X, Y, Z, little-endian int16 each (Page 1)
//   0x26–0x27   Accelerometer temperature word when the offsets were taken (Page 1)
//   0x28–0x2F   The previous zero, same form; 0x30–0x37 the one before that (Page 1, patch 5)
//   0x38–0x39   Zero generation, uint16 little-endian: zeros stored since manufacture, 0 never (Page 1, patch 5)
//   0x40        Status: bit 0 ready (registers hold a complete reading);
//               bit 1 LiDAR fault; bit 2 accelerometer fault; bit 7 pan-fault
//   0x41        Control (writable): bit 0 trigger a reading now (self-clearing);
//               bit 1 measure LiDAR, bit 2 measure accelerometer, on the next
//               reading (power-up: both set); bit 7 sleep (reserved here; the
//               firmware clears it; implementation deferred)
//   0x42–0x43   Reading counter, uint16 little-endian, +1 when ready is set
//   0x44–0x45   Readings requested, uint16 little-endian (writable)
//   0x46        Config (writable): sensitivity mode bits [1:0]
//   0x47        Report, latched until the controller writes Control: the
//               device's most recent report, a fault (its chip's status bit
//               is set too) or a notice (no status bit):
//               bits 7–5 chip (0 LiDAR, 1 accelerometer, 7 the unit), bits 4–0 kind
//               (1 no-acknowledge, 2 timeout, 5 not initialised, 6 reset,
//               9 calibration stored, 10 batch abandoned)
//   0x48–0x49   Range [cm], little-endian int16
//   0x4A        LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
//   0x50–0x55   Accelerometer X, Y, Z raw, little-endian int16 each
//   0x56–0x57   Accelerometer temperature, the LIS3DH OUT_ADC3 word as read (L, H), relative, 1 digit/°C in the high byte
//   0x58–0x59   Zero generation again, the Page 1 word mirrored so that a reading carries it (patch 5)
#define REG_OFFSET   0x20
#define REG_ZERO_GEN 0x38
#define REG_STATUS   0x40
#define REG_CTRL     0x41
#define REG_COUNTER  0x42
#define REG_REQUEST  0x44  // Readings requested, uint16 LE, writable: chips held powered for this many readings
#define REG_CONFIG   0x46
#define REG_REPORT   0x47
#define REG_RANGE    0x48
#define REG_SIGNAL   0x4A
#define REG_ACCEL    0x50
#define REG_ACCEL_TEMP 0x56
#define REG_ZERO_GEN_MIRROR 0x58
#define BIT_READY    0x01
#define BIT_PANFAULT 0x80
#define BIT_TRIGGER  0x01
#define CHIP_LIDAR   0x02   // control bit 1 = chip 0
#define CHIP_ACCEL   0x04   // control bit 2 = chip 1
#define BIT_SLEEP    0x80
#define FAULT_LIDAR_TIMEOUT 0x02   // chip 0, kind 2
#define FAULT_LIDAR_NOACK   0x01   // chip 0, kind 1: never acknowledged after power-up
#define FAULT_LIDAR_NOTINIT 0x05   // chip 0, kind 5: acknowledged but never reported healthy
#define FAULT_ACCEL_NOACK   0x21   // chip 1, kind 1
#define NOTICE_UNIT_RESET   0xE6   // unit (7), kind 6: reset since the controller last wrote Control (a notice: no status bit)
#define NOTICE_UNIT_PAGE0   0xE3   // unit (7), kind 3: Page 0 CRC did not match (unprovisioned or corrupt)
#define NOTICE_ACCEL_CALIBRATED 0x29 // chip 1, kind 9: a zero was stored (Page 1 holds it)
#define NOTICE_BATCH_ABANDONED  0x0A // chip 0, kind 10: the controller stopped triggering and the LiDAR was powered down

// Register array: three 32-byte pages (NW-Device-Specification). Page 0
// (0x00–0x1F) identity, Page 1 (0x20–0x3F) calibration, Page 2 (0x40–0x5F)
// status and sensor data. A controller writes a start address, then reads
// up to 32 bytes with auto-increment (see requestEvent).
#define REG_SIZE 96
uint8_t reg[REG_SIZE] = {0};  // Page 0 filled by loadPage0(); Page 1 by loadPage1() at boot and after each zero; Page 2 at runtime
bool page0Valid = false;      // Page 0 CRC matched what NW-Provision wrote

// CRC-8/SMBUS (poly 0x07, init 0x00), the NW-Device-Specification reference.
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// Copy Page 0 from EEPROM into the served register array, check its CRC,
// then substitute this firmware's patch version at 0x0A and recompute the
// CRC of the served copy (EEPROM is left as provisioned).
void loadPage0() {
  for (uint8_t i = 0; i < 32; i++) reg[i] = EEPROM.read(PAGE0_BASE + i);
  page0Valid = (crc8(reg, 0x1E) == reg[0x1E]) && reg[0x00] == 0x01;
  reg[0x0A] = FW_FW_PATCH;
  reg[0x1E] = crc8(reg, 0x1E);
}

// Copy Page 1 from EEPROM into the served register array byte for byte, with
// every blank word (0xFFFF, never written) served as 0, and mirror the zero
// generation into Page 2 (0x58-0x59). Called at boot and after a zero is
// stored; the copy into the array is atomic so a page read never straddles it.
void loadPage1() {
  uint8_t page[32];
  for(uint8_t i = 0; i < 32; i += 2) {
    page[i] = EEPROM.read(PAGE1_BASE + i);
    page[i + 1] = EEPROM.read(PAGE1_BASE + i + 1);
    if(page[i] == 0xFF && page[i + 1] == 0xFF) { page[i] = 0; page[i + 1] = 0; } //blank word reads as 0
  }
  cli();
  for(uint8_t i = 0; i < 32; i++) reg[REG_OFFSET + i] = page[i];
  reg[REG_ZERO_GEN_MIRROR] = page[REG_ZERO_GEN - REG_OFFSET];
  reg[REG_ZERO_GEN_MIRROR + 1] = page[REG_ZERO_GEN - REG_OFFSET + 1];
  sei();
}

// Registers a controller may write. Everything else is read-only and writes
// to it are ignored (NW-Device-Specification, Page 2 rules).
bool isWritable(uint8_t pos) {
  return pos == REG_CTRL || pos == REG_CONFIG || pos == REG_I2C_ADDR
      || pos == REG_REQUEST || pos == REG_REQUEST + 1;
}
// bool startReading = true; //Flag used to start a new converstion, make a conversion on startup
// const unsigned int updateRate = 5; //Rate of update

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_A2, PIN_A3, true);  //Initialize software I2C

volatile bool stopFlag = false; //Used to indicate a stop condition 
volatile uint8_t regID = 0; //Used to denote which register will be read from

int16_t offsets[3] = {0};  //X,Y,Z acceleration offsets to zero the angle of the device 
int16_t offsetTemp = 0; //Accelerometer temperature word when the offsets were taken (Page 1, 0x26); the reference for a drift correction
int16_t accelTemp = 0; //Accelerometer temperature word of the last reading (OUT_ADC3, relative)
//The zero: samples at the 10 Hz output rate until the standard error of every
//axis mean is below ZERO_SE_MAX counts, after at least ZERO_MIN_SAMPLES and at
//most ZERO_MAX_SAMPLES (100 s at 10 Hz). The magnet only starts it.
#define ZERO_MIN_SAMPLES 32
#define ZERO_MAX_SAMPLES 1000
#define ZERO_SE_MAX 0.25 //counts (1 mg per count in high-resolution mode): 0.014 degrees
int16_t accelVals[3] = {0}; //Global storage for acceleration data values to be shared between EEPROM functions and getter functions 

bool switchLatch = false;  //Latching functionality control for Hall effect switch 
bool zeroRequested = false; //A zero is due (the magnet was seen at boot)
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
  loadPage0();
  loadPage1();
  if (reg[REG_I2C_ADDR] != 0xFF) adr = reg[REG_I2C_ADDR]; // Provisioned address; 0xFF = use default
  Wire.begin(adr);  //Begin slave I2C
#ifdef APIS_DEBUG
  Serial.begin(9600);
#endif
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

  reg[REG_STATUS] = 0; // Not ready: no reading yet
  reg[REG_CTRL] = CHIP_LIDAR | CHIP_ACCEL; // Power-up: every chip selected
  reg[REG_REPORT] = page0Valid ? NOTICE_UNIT_RESET : NOTICE_UNIT_PAGE0; // Latched until the controller writes Control
  delay(10);
  si.i2c_init(); //Begin I2C master
  initAccel();
  // The LiDAR stays off until the first trigger (lidarPowerUp()).
  digitalWrite(STAT_LED, LOW);  //Blink on statup
  if(!digitalRead(HALL_SWITCH)) {
    zeroRequested = true; //Magnet present at boot: take a new zero once running; it need not stay (it used to clear the offsets instead)
    switchLatch = true; //Set latch to prevent override 
  }
  digitalWrite(STAT_LED, HIGH);

}

void loop() {
  // static unsigned int Count = 0; //Counter to determine update rate
  // if(startReading == true) {
  //  //Read new values in
  //  AutoRange_Vis();  //Run auto range
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
  // Idle until the controller triggers (on-demand only; no free-running cycle).
  while(!(reg[REG_CTRL] & BIT_TRIGGER)) {
    set_sleep_mode(SLEEP_MODE_IDLE);   // I2C address match wakes the core
    sleep_enable();
    sei();
    sleep_cpu();
    if(zeroRequested || (!digitalRead(HALL_SWITCH) && !switchLatch)) {  //Magnet at boot, or newly applied: one application is one zero
      zeroRequested = false;
      if(!digitalRead(HALL_SWITCH)) switchLatch = true; //latch switch until toggle of state
      digitalWrite(STAT_LED, HIGH); //Turn on status LED while the zero is being taken; off once it is stored. The magnet may leave at once
      if(zeroAccel()) { updateOffset(accelVals, accelTemp); latchNotice(NOTICE_ACCEL_CALIBRATED); } //As many samples as the zero needs; nothing stored if the accelerometer failed
      digitalWrite(STAT_LED, LOW);
    }
    if(digitalRead(HALL_SWITCH)) {
      switchLatch = false; //If switch is high, back to default state, reset latch 
    }
    // if(Serial.available() > 0) {  //FIX add serial control??
    //  uint8_t Data1 = Serial.read();
    //  uint8_t Data2 = Serial.read();
    //  if(Data2 == 'F') Ctrl = Data1; //If 
    // }
    if(lidarOn && (millis() - lidarLastReading) > lidarBatchTimeout) {
      lidarPowerDown(); // batch abandoned: the controller stopped triggering
      latchNotice(NOTICE_BATCH_ABANDONED);
    }
  }
  sleep_disable();
  lidarConfig = reg[REG_CONFIG] & 0x03; //Pull low two bits from Config (0x46) to get Lidar configuration state
  // A reading begins: clear ready, take the chip selection, consume the trigger.
  reg[REG_STATUS] &= ~BIT_READY;
  bool doLidar = reg[REG_CTRL] & CHIP_LIDAR;
  bool doAccel = reg[REG_CTRL] & CHIP_ACCEL;
  reg[REG_CTRL] &= ~(BIT_TRIGGER | BIT_SLEEP); // trigger consumed; sleep not implemented
  if(requestWritten) { // a new readings-requested word: count from this reading
    requestWritten = false;
    requested = reg[REG_REQUEST] | (reg[REG_REQUEST + 1] << 8);
    requestBase = reg[REG_COUNTER] | (reg[REG_COUNTER + 1] << 8);
    lidarInitFail = 0; // a new batch gets a fresh power-up attempt
  }
  // A power-up that failed earlier in this batch is not retried on every trigger:
  // the remaining readings report the fault at once (a batch on a dead LiDAR
  // then costs ~10 ms per reading instead of ~440 ms). Single readings retry.
  if(doLidar && !lidarOn && !lidarInitFail) lidarPowerUp();
  if(lidarOn && lidarConfig != lidarConfigApplied) { initLiDAR(); lidarConfigApplied = lidarConfig; } // Config changed mid-batch
  uint8_t Stat1 = readByte(ACCEL_ADR, 0x27); 
  uint8_t Stat2 = readByte(ACCEL_ADR, 0x07);
  // while(((Stat1 & 0x08) >> 3) != 1 || ((Stat2 & 0x08) >> 3) != 1 || ((Stat2 & 0x80) >> 7) != 1) {
  unsigned long LocalTime = millis();
  // Wait until both status registers report ready, or the timeout elapses.
  // The timeout guards the whole condition (it used to guard only the second
  // half, so a stuck Stat1 could wait forever). See Project-Apis #22.
  while((((Stat1 & 0x08) >> 3) != 1 || Stat2 != 0xFF) && (millis() - LocalTime) < timeoutGlobal) {  //Try to get status from 
    Stat1 = readByte(ACCEL_ADR, 0x27);
    Stat2 = readByte(ACCEL_ADR, 0x07);
    delay(1); //DEBUG!
  }
  accelFail = (millis() - LocalTime) >= timeoutGlobal; //Set flag if timeout occoured (was inverted; #22)

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

  int16_t Range = -9999;
  if(doLidar && lidarOn) Range = getRange();  //DEBUG! Replace!
  if(doLidar && !lidarOn) { splitAndLoad(REG_RANGE, -9999); reg[REG_SIGNAL] = 0; } // power-up failed
#ifdef APIS_DEBUG
  Serial.print('R'); //Preceed range value
  Serial.println(Range); 
#endif
  getOffsets(); //Read in offsets
  if (doAccel) getG(true);

  // Reading complete: load status and fault, bump the counter, set ready.
  // Atomic so a controller's page read never straddles the update.
  uint8_t status = BIT_READY;
  if(doLidar && lidarInitFail) { status |= 0x02; reg[REG_REPORT] = lidarInitFail; }
  else if(doLidar && lidarFail) { status |= 0x02; reg[REG_REPORT] = FAULT_LIDAR_TIMEOUT; }
  if (doAccel && accelFail) { status |= 0x04; reg[REG_REPORT] = FAULT_ACCEL_NOACK; }
  if (status & 0x7E) status |= BIT_PANFAULT;
  uint16_t count = reg[REG_COUNTER] | (reg[REG_COUNTER + 1] << 8);
  count++;
  cli();
  reg[REG_COUNTER] = count & 0xFF; reg[REG_COUNTER + 1] = count >> 8;
  reg[REG_STATUS] = status;
  sei();
  // Serial.println(readByte(LIDAR_ADR, 0x0E)); //DEBUG! //READ RSSI
  // Serial.println(readByte(ACCEL_ADR, 0x27), BIN); //DEBUG! 
  // Power decision: down after a single reading (requested 0 or 1) or once the
  // requested count is done; otherwise stay powered for the next trigger.
  uint16_t done = count - requestBase;
  bool batchOver = (requested <= 1) || (done >= requested);
  if(lidarOn) {
    if(batchOver) lidarPowerDown();
    else lidarLastReading = millis();
  }
  if(batchOver) lidarInitFail = 0; //The next batch (or single reading) tries the power-up again
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
  writeByte(ACCEL_ADR, CTRL_REG1_ADR, 0x27); //10 Hz output data rate (5 Hz bandwidth: 0.5 mg rms per sample instead of 3 mg at 400 Hz); was 0x77 //FIX! Set to low power initally??
  writeByte(ACCEL_ADR, CTRL_REG4_ADR, 0x88); //Turn on high resolution mode //FIX! Setup to use self text
  writeByte(ACCEL_ADR, CTRL_REG3_ADR, 0x10);
  writeByte(ACCEL_ADR, TEMP_CFG_REG_ADR, 0xC0); //ADC and temperature sensor on (was 0x80, ADC only): OUT_ADC3 carries the chip temperature for a drift correction
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
    accelTemp = readAccelTemp(); //The chip temperature beside the axes, for the drift correction

    if(Set) {  //If sending data is commanded, print data out
#ifdef APIS_DEBUG
      Serial.print('X'); Serial.println(Axis[0] - offsets[0]);  //FIX! Optimize to prevent multiple addition 
      Serial.print('Y'); Serial.println(Axis[1] - offsets[1]);
      Serial.print('Z'); Serial.println(Axis[2] - offsets[2]);
#endif

      splitAndLoad(REG_ACCEL, Axis[0]);  //Load accel values
      splitAndLoad(REG_ACCEL + 2, Axis[1]);
      splitAndLoad(REG_ACCEL + 4, Axis[2]);
      splitAndLoad(REG_ACCEL_TEMP, accelTemp);
    }
  }


  // return Data;
}

void lidarPowerUp()
{
  // Readiness by register, not by clock: after the rail ramp and enable, poll for an
  // I2C acknowledge and then the health flag (STATUS 0x01 bit 5: reference and
  // receiver bias operational). One retry toggles enable after a further rail
  // wait (enable must follow the ramp); a second failure latches a fault.
  digitalWrite(POWER_SW, HIGH); // Turn on 5v switched power; 680 uF cap charges at ~227 mA
  lidarInitFail = 0;
  for(uint8_t attempt = 0; attempt < 2; attempt++) {
    delay(lidarRailMs);
    digitalWrite(ENABLE, HIGH); //NOTE: MUST toggle enable after voltage ramp to ensure effective measurment 
    unsigned long t0 = millis();
    bool acked = false;
    while((millis() - t0) < lidarBootTimeout) {
      if(si.i2c_start((LIDAR_ADR << 1) | WRITE)) {
        si.i2c_stop();
        acked = true;
        int st = readByte(LIDAR_ADR, 0x01);
        if(st >= 0 && (st & 0x20)) { // healthy
          initLiDAR();
          lidarConfigApplied = lidarConfig;
          lidarOn = true;
          lidarLastReading = millis();
          return;
        }
      }
      else si.i2c_stop();
      delay(1);
    }
    lidarInitFail = acked ? FAULT_LIDAR_NOTINIT : FAULT_LIDAR_NOACK;
    digitalWrite(ENABLE, LOW); // retry once
  }
  lidarPowerDown();
}

void lidarPowerDown()
{
  digitalWrite(ENABLE, LOW);
  digitalWrite(POWER_SW, LOW); //Turn off 5v switched power
  lidarOn = false;
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
  writeByte(LIDAR_ADR, 0x00, 0x01); // ACQ_COMMAND: any non-zero value starts a measurement (v3HP)
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x00); 
  // si.i2c_stop();
  // si.i2c_start((LIDAR_ADR << 1) | WRITE);
  // si.i2c_write(0x01); //Command to take measurment WITH correction bias 
  // si.i2c_stop();
  unsigned long LocalTime = millis();
  // while((readByte(LIDAR_ADR, 0x01) & 0x01) == 1 && (millis() - LocalTime) < timeoutGlobal && digitalRead(MODE_READ) == LOW); //Wait for updated value or timeout
  // Wait for the acquisition by polling STATUS (0x01) bit 0 (busy) until it clears:
  // a level, so an acquisition that finished before the first poll reads done, and
  // done is correct (the data stays valid until the next measurement concludes).
  // The mode pin is not used: on this board it is held high through R11 and
  // cannot indicate busy (Project-Apis #24).
  while((millis() - LocalTime) < timeoutGlobal && (readByte(LIDAR_ADR, 0x01) & 0x01));
  if((millis() - LocalTime) < timeoutGlobal) {  //If timeout has NOT occoured, read as normal
    Data = readWordLE(LIDAR_ADR, 0x0F);
    splitAndLoad(REG_RANGE, Data);
    // Read signal strength from LiDAR Lite reg 0x0E directly (no auto-increment bit)
    sendCommand(LIDAR_ADR, 0x0E);
    si.i2c_stop();
    si.i2c_start((LIDAR_ADR << 1) | READ);
    reg[REG_SIGNAL] = si.i2c_read(false);
    si.i2c_stop();
    lidarFail = false;  //Clear failure flag
  }
  else {  //Otherwise set failure flag and set out of range data value
    lidarFail = true;
    Data = -9999;
    splitAndLoad(REG_RANGE, Data);
    reg[REG_SIGNAL] = 0;
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
  return true; // send ACK to master
}

void requestEvent()
{ 
  // Serve up to one full page from the requested register with auto-increment.
  // WireS clocks out only as many bytes as the controller asks for; the rest
  // of the buffer is discarded at the stop condition. Reads past the end of
  // the array return zeros, never a wrap onto Page 0.
  for(uint8_t i = 0; i < 32; i++) {
    uint16_t k = (uint16_t)regID + i;
    Wire.write(k < REG_SIZE ? reg[k] : 0x00);
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
      if (!isWritable(Pos)) return; //Read-only register: ignore the write
      reg[Pos] = Val; //Set register value
      if (Pos == REG_CTRL) reg[REG_REPORT] = 0; //A control write acknowledges the report
      if(Pos == REG_REQUEST || Pos == REG_REQUEST + 1) requestWritten = true; //Latched at the next trigger
      if (Pos == REG_I2C_ADDR) EEPROM.update(PAGE0_BASE + REG_I2C_ADDR, Val); //Persist I2C address (compare-before-write); takes effect on next boot
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

int16_t readAccelTemp() //The LIS3DH OUT_ADC3 word (L, H) as read: the temperature sensor, relative, 1 digit/°C in the high byte
{
  sendCommand(ACCEL_ADR, OUT_ADC3_ADR | 0x80); //auto-increment
  si.i2c_stop();
  si.i2c_start((ACCEL_ADR << 1) | READ);
  uint8_t Low = si.i2c_read(false);
  uint8_t High = si.i2c_read(false);
  si.i2c_stop();
  return (int16_t)((High << 8) | Low);
}

bool zeroAccel() //Average fresh samples into accelVals until the mean of every axis is settled; false if the accelerometer failed
{
  int32_t Sum[3] = {0};
  float SumSq[3] = {0};
  uint16_t n = 0;
  while(n < ZERO_MAX_SAMPLES) {
    unsigned long LocalTime = millis();
    while(!(readByte(ACCEL_ADR, STATUS_REG_ADR) & 0x08) && (millis() - LocalTime) < timeoutGlobal) delay(1); //A new sample (ZYXDA), one per 100 ms at 10 Hz
    getG(false);
    if(accelFail) return false;
    n++;
    for(int i = 0; i < 3; i++) { Sum[i] += accelVals[i]; SumSq[i] += (float)accelVals[i]*accelVals[i]; }
    if(n >= ZERO_MIN_SAMPLES) {
      bool settled = true;
      for(int i = 0; i < 3; i++) {
        float Mean = (float)Sum[i]/n;
        float Var = (SumSq[i] - n*Mean*Mean)/(n - 1);
        if(Var < 0) Var = 0;
        if(sqrt(Var/n) > ZERO_SE_MAX) settled = false; //Standard error of the mean, in counts
      }
      if(settled) break;
    }
  }
  for(int i = 0; i < 3; i++) accelVals[i] = (int16_t)((Sum[i] + (Sum[i] >= 0 ? (int32_t)n/2 : -(int32_t)n/2))/(int32_t)n); //Rounded mean
  return true;
}

bool isFaultCode(uint8_t code) //A report code whose kind is a fault (1-5, 7, 8), as opposed to a notice (6, 9, 10)
{
  uint8_t kind = code & 0x1F;
  return kind >= 1 && kind <= 8 && kind != 6;
}

void latchNotice(uint8_t code) //A notice never overwrites a fault the controller has not yet acknowledged
{
  if(!isFaultCode(reg[REG_REPORT])) reg[REG_REPORT] = code;
}

void updateOffset(int16_t *AxisData, int16_t Temp)  //Pass in array of X,Y,Z offset values and the temperature word they were taken at; the two zeros before it are kept
{
  //Shift the record first, Block 1 to Block 2 then Block 0 to Block 1, so the new zero lands in Block 0 (patch 5)
  for(int i = 15; i >= 0; i--) EEPROM.update(PAGE1_BASE + 8 + i, EEPROM.read(PAGE1_BASE + i));
  // uint8_t Val[4] = {0}; //Blank array to use as temporary storage for desconsturcted float
  // for(int i = 0; i < 3; i++) {
  //  memcpy(Val, &AxisData[i], sizeof(float)); //Deconstruct the ith axis value into the temprary Val register 
  //  for(int p = 0; p < 4; p++) {
  //    EEPROM.write(Val[p], p + i); //Write from desired entry in EEPROM (the pth entry of the ith 4 byte float)
  //  }
  // }
  for(int i = 0; i < 3; i++) {
    // ((EEPROM.read(p + i) << 8) | EEPROM.read(2*i + 1)); //Read from desired entry in EEPROM and concatonate
    EEPROM.update(PAGE1_BASE + 2*i, AxisData[i] & 0xFF);  //Write LSB: bus order, so the page is served byte for byte
    EEPROM.update(PAGE1_BASE + 2*i + 1, AxisData[i] >> 8);  //Write MSB
  }
  EEPROM.update(PAGE1_BASE + 6, Temp & 0xFF); //The temperature the zero was taken at, beside it
  EEPROM.update(PAGE1_BASE + 7, Temp >> 8);
  uint16_t generation = EEPROM.read(PAGE1_BASE + 24) | (EEPROM.read(PAGE1_BASE + 25) << 8); //Zeros stored since manufacture
  if(generation == 0xFFFF) generation = 0; //never written
  generation++;
  EEPROM.update(PAGE1_BASE + 24, generation & 0xFF);
  EEPROM.update(PAGE1_BASE + 25, generation >> 8);
  loadPage1(); //The served page and the mirror follow at once
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
      offsets[i] = (int)((EEPROM.read(PAGE1_BASE + 2*i + 1) << 8) | EEPROM.read(PAGE1_BASE + 2*i)); //Read from desired entry in EEPROM and concatonate (little-endian)
      if (offsets[i] == -1) offsets[i] = 0; //0xFFFF = never written (fresh Page 1): no offset
    // memcpy(&offsets[i], &Val, sizeof(float)); //Load the 4 discrete bytes back into the ith offset float
  }
  offsetTemp = (int16_t)((EEPROM.read(PAGE1_BASE + 7) << 8) | EEPROM.read(PAGE1_BASE + 6));
  if(offsetTemp == -1) offsetTemp = 0; //never written
}

// void resetOffset()  //Set offset back to zero values
// {
//  for(int i = 0; i < 12; i++) {
//    EEPROM.write(i) = 0; //Clear all utilized EEPROM values
//  }
// }
