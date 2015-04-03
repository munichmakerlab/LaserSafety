#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <avr/wdt.h>


// Pins
float temp1_min = 8.0 ;
float temp1_max = 27;

float temp2_min = 8.0;
float temp2_max = 26;

#define p_lid 2
#define p_pressure 3
#define p_waterflow 4
#define p_waterleak1 5
#define p_waterleak2 6

#define p_onewire 7

#define  p_safety 12 // The output pin to the laser controller

// Sensors
bool s_waterflow_ok = false;
bool s_pressure_ok = false;
bool s_lid_ok = false;
bool s_waterleak1_ok = false;
bool s_waterleak2_ok = false;
bool s_temp1_ok = false;
bool s_temp2_ok = false;

float s_temp1;
float s_temp2;


bool safety_flag = false;
bool disable_laser = true;

DeviceAddress hinlauf = {0x28, 0x04, 0xDD, 0xAC, 0x04, 0x00, 0x00, 0x55};
DeviceAddress ruecklauf = {0x28, 0xC3, 0x0D, 0xAE, 0x04, 0x00, 0x00, 0x16};

OneWire oneWire(p_onewire);

DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(p_safety, OUTPUT);
  digitalWrite(p_safety, HIGH);

  Serial.begin(9600);
  Serial.println("Starting...");

  setupi2c();

  sensors.begin();
  sensors.setResolution(hinlauf, 10);
  sensors.setResolution(ruecklauf, 10);

  pinMode(p_lid, INPUT_PULLUP);
  pinMode(p_waterflow, INPUT_PULLUP);
  pinMode(p_pressure, INPUT_PULLUP);
  pinMode(p_waterleak1, INPUT_PULLUP);
  pinMode(p_waterleak2, INPUT_PULLUP);
  
  Timer1.initialize(50000);
  Timer1.attachInterrupt(doImportantStuff);
  
  // watchdogSetup();
}

void watchdogSetup(void)
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  /*
   WDTCSR configuration:
   WDIE = 1: Interrupt Enable
   WDE = 1 :Reset Enable
   WDP3 = 0 :For 2000ms Time-out
   WDP2 = 1 :For 2000ms Time-out
   WDP1 = 1 :For 2000ms Time-out
   WDP0 = 1 :For 2000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (0 << WDE);
  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (0 << WDE) | ( 1 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  sei();
}

void setupi2c() {
  Wire.begin();     //I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x80);  // autoincrement ab Register 0h

  Wire.write(0x00);  // Register 00 /  Mode1
  Wire.write(0x00);  // Register 01 /  Mode2

  Wire.write(0x00);  // Register 02 /  PWM LED 1    // Default alle PWM auf 0
  Wire.write(0x00);  // Register 03 /  PWM LED 2
  Wire.write(0x00);  // Register 04 /  PWM LED 3
  Wire.write(0x00);  // Register 05 /  PWM LED 4
  Wire.write(0x00);  // Register 06 /  PWM LED 5
  Wire.write(0x00);  // Register 07 /  PWM LED 6
  Wire.write(0x00);  // Register 08 /  PWM LED 7
  Wire.write(0x00);  // Register 09 /  PWM LED 8
  Wire.write(0x00);  // Register 0A /  PWM LED 9
  Wire.write(0x00);  // Register 0B /  PWM LED 10
  Wire.write(0x00);  // Register 0C /  PWM LED 11
  Wire.write(0x00);  // Register 0D /  PWM LED 12
  Wire.write(0x00);  // Register 0E /  PWM LED 13
  Wire.write(0x00);  // Register 0F /  PWM LED 14
  Wire.write(0x00);  // Register 10 /  PWM LED 15
  Wire.write(0x00);  // Register 11 /  PWM LED 16  // Default alle PWM auf 0

  Wire.write(0xFF);  // Register 12 /  Group duty cycle control
  Wire.write(0x00);  // Register 13 /  Group frequency
  Wire.write(0xAA);  // Register 14 /  LED output state 0  // Default alle LEDs auf PWM
  Wire.write(0xAA);  // Register 15 /  LED output state 1  // Default alle LEDs auf PWM
  Wire.write(0xAA);  // Register 16 /  LED output state 2  // Default alle LEDs auf PWM
  Wire.write(0xAA);  // Register 17 /  LED output state 3  // Default alle LEDs auf PWM
  Wire.write(0x00);  // Register 18 /  I2C bus subaddress 1
  Wire.write(0x00);  // Register 19 /  I2C bus subaddress 2
  Wire.write(0x00);  // Register 1A /  I2C bus subaddress 3
  Wire.write(0x00);  // Register 1B /  All Call I2C bus address
  Wire.write(0xFF);  // Register 1C /  IREF configuration
  Wire.endTransmission();  // I2C-Stop
}

void Set_LED_PWM(int LED, int PWM)
{
  Wire.begin();             //I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x01 + LED);    // Register LED-Nr
  Wire.write(PWM);
  Wire.endTransmission();   // I2C-Stop
}


void get_sensor_states() {

  // s_lid_ok
  if (digitalRead(p_lid) == LOW) {
    s_lid_ok = true;
  } else {
    s_lid_ok = false;
  }

  // s_pressure_ok
  if (digitalRead(p_pressure) == LOW) {
    s_pressure_ok = true;
  } else {
    s_pressure_ok = false;
  }

  // s_waterflow_ok
  if (digitalRead(p_waterflow) == LOW) {
    s_waterflow_ok = true;
  } else {
    s_waterflow_ok = false;
  }

  // s_waterleak

  if (digitalRead(p_waterleak1) == LOW) {
    s_waterleak1_ok = true;
  } else {
    s_waterleak1_ok = false;
  }

  if (digitalRead(p_waterleak2) == LOW) {
    s_waterleak2_ok = true;
  } else {
    s_waterleak2_ok = false;
  }

  // s_temp1
  s_temp1 = sensors.getTempC(hinlauf);
  Serial.println(s_temp1);

  if (s_temp1 > temp1_min && s_temp1 < temp1_max ) {
    s_temp1_ok = true;
  } else {
    s_temp1_ok = false;
  }

  // s_temp2
  s_temp2 = sensors.getTempC(ruecklauf);
  Serial.println(s_temp2);

  if (s_temp2 > temp2_min && s_temp2 < temp2_max) {
    s_temp2_ok = true;
  } else {
    s_temp2_ok = false;
  }
}

void set_safety_flag() { // Checks, if all inputs indicate safe performance, then sets safety_flag
  if (
    s_lid_ok &&
    s_pressure_ok &&
    s_waterflow_ok &&
    s_waterleak1_ok &&
    s_waterleak2_ok &&
    s_temp1_ok &&
    s_temp2_ok
  )
  {
    safety_flag = true;
    // Serial.println("True");
  } else {
    safety_flag = false;
  }

}

void updateDisplay() {
  // Write out sensor states to i2c
  if (s_lid_ok) {
    Set_LED_PWM(1, 255);
  } else {
    Set_LED_PWM(1, 0);
  }
  if (s_pressure_ok) {
    Set_LED_PWM(2, 255);
  } else {
    Set_LED_PWM(2, 0);
  }
  if (s_waterflow_ok) {
    Set_LED_PWM(3, 255);
  } else {
    Set_LED_PWM(3, 0);
  }
}

void loop() {

  sensors.requestTemperatures();
  Serial.println("Done reading temp");
  
  // DEBUG CODE
  if (digitalRead(13) == LOW) {
    // generate artificial hang:
    Serial.println("Generating hang...");
    delay(10000);
  }

  wdt_reset();
}

void doImportantStuff() {
  get_sensor_states(); // set all the s_ variables from input
  set_safety_flag(); // check all variables, then set safety_flag
  disable_laser = ! safety_flag; // If save operation not garateed, disable laser (HIGH Output will disable the laser!)
  digitalWrite(p_safety, disable_laser); // Write out pin state
  
  updateDisplay();
  
  Serial.println(safety_flag);
  wdt_reset();
}

ISR(WDT_vect) // Watchdog timer interrupt.
{
  digitalWrite(p_safety, HIGH);
  Serial.println("Watchodg");
  //

  while (true) {
    
  }

  // To be filled with "useful" debug info later
  //EEPROM.write(crash_report_addr, 1);

}
