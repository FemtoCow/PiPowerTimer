/*

Raspberry Pi supervisor HAT based on ATMega328P
(3.3V device, when programming select Arduino Mini, 8MHz)

Accessories:

  RTC (MCP7940, http://ww1.microchip.com/downloads/en/DeviceDoc/20005010F.pdf )
  Temperature Sensor (LMT88, 3.3V, http://www.ti.com/lit/ds/symlink/lmt88.pdf )
  Battery Voltage Meter (1M - 100k voltage divider)
  Current Meter (for the whole assembly) (ZXCT1110, R_SENSE=0.1R, R_GAIN=1k, http://www.diodes.com/_files/datasheets/ZXCT1107_10.pdf )
  5V Step Down Converter (LM22676, http://www.ti.com/lit/ds/symlink/lm22676-q1.pdf )


Connections:

  Arduino 
    0        (ARX)
    1        (ATX)
    2*       (D2)
   ~3*       MFP, LED, BTN; pulled high externally, WAKE UP interrupt, set as input or as output low, do NOT set as output high
    4        n/c 
   ~5        SW_SCL; pulled high externally, software I2C to RTC
   ~6        SW_SDA; pulled high externally, software I2C to RTC 
    7        EN_5V; pulled low externally, set high to enable 5V power converter
    8        n/c
   ~9        n/c
  ~10        n/c
  ~11        (MOSI)
   12        (MISO)
   13        (SCK)
   A0        current sensing
   A1        n/c
   A2        (A2)
   A3        (A3)
   A4        SDA; I2C as slave, connected to rPi
   A5        SCL; I2C as slave, connected to rPi
   A6        battery voltage sensing
   A7        board temperature sensing


I2C register description:
  
  The device communicates with rPi over I2C as a slave device. The device address is 0x69. 
  
  Following is the list of registers (all read only)
  0x10 - Current Temperature - return one signed byte, temperature in C, -128C .. 127C
  0x20 - Current Battery Voltage - return one byte, battery voltage in 1/10V, 0.0V .. 25.5V
  0x30 - Current Current Drawn - return one byte, current drawn from battery in 10mA, 0A .. 2.55A
  0x40 - RTC
  0xA1 - byte, A1/4
  0xA2 - byte, A2/4
  0xA3 - byte, A3/4
  0xD0 - byte, pin 0
  0xD1 - byte, pin 1
  0xD2 - byte, pin 2
  0xF0 - virtual serial port, r/w, master must pool for new data periodically
  0xF3 - TEST - return one byte 0x42
  0xF5 - API version - return one byte


*/

//http://dsscircuits.com/articles/arduino-i2c-slave-guide

#include <Wire.h>
#include <DigitalButtons.h>      //https://github.com/cano64/ArduinoButtons
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>



#define THE_DEVICE_ADDRESS 0x69
#define I2C_API_VERSION 1


//class to handle i2c slave API
class MrSlave {
  public:
    byte reg; //register ID set by master last time
    
    MrSlave();
    void init();
    void onIdle();
    void receiveData(int byteCount);
    void sendData();
    
} mrSlave;


// API to access the device hardware
class TheDevice {
  public:
    TheDevice();
    void init();
    void onIdle();
    signed char getTemperatureC();
    unsigned char getBatteryVoltage10V();
    unsigned char getBatteryCurrent10mA();
    void enable5V();
    void disable5V();
    void LEDon();
    void LEDoff();
    void SleepWDT(unsigned char wdt_period);
    void Sleep(unsigned char sec);
    void SleepWakeOnInterrupt(unsigned char i);
    unsigned int getFreeRAM();
    
} theDevice;


/********************* MrSlave ************************/

MrSlave::MrSlave(): reg(0x00) {
   
}

void MrSlave::init() {
  // initialize i2c as slave
  Wire.begin(THE_DEVICE_ADDRESS);

  // define callbacks for i2c communication
  // in a parallel universe which is much better than ours, 
  // there are pointers to member functions used instead
  Wire.onReceive(onSlaveReceive);
  Wire.onRequest(onSlaveRequest);
}  
  
void MrSlave::onIdle() {

}

void onSlaveReceive(int byteCount) {
  mrSlave.receiveData(byteCount);
}

// callback for i2c received data
void MrSlave::receiveData(int byteCount) {
  if (Wire.available()) {
    //first byte is a register ID
    this->reg = Wire.read();
    //rest is shit
    while(Wire.available()) {
      Wire.read();
    }
  }
}

// callback for sending data
void onSlaveRequest(){
  mrSlave.sendData();
}

void MrSlave::sendData() {
  switch (this->reg) {
    case 0x10:
      Wire.write(theDevice.getTemperatureC());
      break;        
    case 0x20:
      Wire.write(theDevice.getBatteryVoltage10V());
      break;        
    case 0x30:
      Wire.write(theDevice.getBatteryCurrent10mA());
      break;        
    case 0x40:
      break;        
    case 0xA1:
      break;        
    case 0xA2:
      break;        
    case 0xA3:
      break;        
    case 0xD0:
      break;        
    case 0xD1:
      break;        
    case 0xD2:
      break;        
    case 0xF0:
      break;        
    case 0xF3:
        Wire.write(0x42);    
      break;        
    case 0xF5:
        Wire.write(I2C_API_VERSION);
      break;   
    default:     
      break;
  }
}

/********************* TheDevice ***********************/

TheDevice::TheDevice() {
  
}

void TheDevice::init() {
  this->disable5V();
}

void TheDevice::onIdle() {
  
}

signed char TheDevice::getTemperatureC() {
  int a = analogRead(A7);
  //T = 161.6583 - 0.279798 * a; //based on 3.3V reference
  //T = (20,692 - 35.75 * a) / 128
  int t = 20692 - 35*a - a>>1 - a>>2;
  return t>>7;
}

unsigned char TheDevice::getBatteryVoltage10V() {
  int a = analogRead(A6);
  //V = A * Vcc/1023 * (Rl+Rh)/Rl
  //V = A * (10 * 3.3/1023 * 1100/100)
  //V = A * 22.709677419354838709677419354839 / 64
  int v = 22*a + a>>1 + a>>3 + a>>4;
  return v>>6;
}

unsigned char TheDevice::getBatteryCurrent10mA() {
  int a = analogRead(A0);
  //Iout = 0.004 * Vs = Vout / Rg, I = Vs/Rs, Vout = A * Vcc/1023
  //I = A * 250 * Vcc / 1023 / Rg / Rs
  //I = A * (250 * 3300 / 1023 / 1000 / 0.1) [mA]
  //I = A * 25.806451612903225806451612903226 / 32 [10mA]
  int i = 25*a + a>>1 + a>>2 + a>>4;
  return i>>5;
}

void TheDevice::enable5V() {
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void TheDevice::disable5V() {
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}


void TheDevice::LEDon() {

}


void TheDevice::LEDoff() {

}

void TheDevice::SleepWDT(unsigned char wdt_period) {
  wdt_enable(wdt_period);
  wdt_reset();
  WDTCSR |= _BV(WDIE); //wake up interrupt when time is up
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  wdt_disable();
  WDTCSR &= ~_BV(WDIE);
}


void TheDevice::Sleep(unsigned char sec) {
  while (sec >= 8) { this->SleepWDT(WDTO_8S);   sec -= 8; }
  if (sec >= 4)    { this->SleepWDT(WDTO_4S);   sec -= 4; }
  if (sec >= 2)    { this->SleepWDT(WDTO_2S);   sec -= 2; }
  if (sec >= 1)    { this->SleepWDT(WDTO_1S);   sec -= 1; }
}


void TheDevice::SleepWakeOnInterrupt(unsigned char i) {
  cli();
  attachInterrupt(i, jebat_cecky, LOW);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();  
  sleep_enable();
  sleep_bod_disable();
  sei(); //first instruction after SEI is guaranteed to execute before any interrupt
  sleep_cpu();
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable();
  power_all_enable();
}

void jebat_cecky() {
  detachInterrupt(0);
}

unsigned int TheDevice::getFreeRAM() {
  extern int  __bss_end;
  extern int  *__brkval;
  int free_memory;
  if((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}


/********************* Arduino *************************/

void setup() {
  theDevice.init();
  mrSlave.init();
}

void loop() {
  theDevice.onIdle();
  mrSlave.onIdle();

}

//moo
// I=U/R, U/(Rl+Rh) = Ul/Rl, U = Ul * (Rl+Rh)/Rl
// A * k = V, k = Vin/1023, V = Vin * A/1023
// Ul = V, U = A* Vin/1023 * (Rl+Rh)/Rl
