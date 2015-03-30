#include <OneWire.h>
#include <DallasTemperature.h>

#include <EEPROM.h>
#include <avr/wdt.h>


// Pins
float temp1_min = 8.0  ;
float temp1_max = 27;

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

float s_temp1;


bool safety_flag = false;
bool disable_laser = true;

OneWire oneWire(p_onewire);

DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(p_safety, OUTPUT);
  digitalWrite(p_safety, HIGH);
  
  Serial.begin(9600);
  Serial.println("Starting...");
  
  // watchdogSetup();
  
  sensors.begin();
  
  pinMode(p_lid, INPUT_PULLUP);
  pinMode(p_waterflow, INPUT_PULLUP);
  pinMode(p_pressure, INPUT_PULLUP);
  pinMode(p_waterleak1, INPUT_PULLUP);
  pinMode(p_waterleak2, INPUT_PULLUP);
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
  WDTCSR |= (1<<WDCE) | (0<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (0<<WDE) | (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
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
  s_temp1 = sensors.getTempCByIndex(0);
  Serial.println(s_temp1);
  
  if (s_temp1 > temp1_min && s_temp1 < temp1_max ) {
    s_temp1_ok = true;
  } else {
    s_temp1_ok = false;
  }
  
}

void set_safety_flag() { // Checks, if all inputs indicate safe performance, then sets safety_flag
  if (
  s_lid_ok &&
  s_pressure_ok &&
  s_waterflow_ok &&
  s_waterleak1_ok &&
  s_waterleak2_ok &&
  s_temp1_ok
  )
  {
    safety_flag = true;
    // Serial.println("True");
  } else {
    safety_flag = false;
  }
  
}

void loop() {
  
  sensors.requestTemperatures();
  
  get_sensor_states(); // set all the s_ variables from input
  set_safety_flag(); // check all variables, then set safety_flag
  
  disable_laser = ! safety_flag; // If save operation not garateed, disable laser (HIGH Output will disable the laser!)
  digitalWrite(p_safety, disable_laser); // Write out pin state
  
  
  // DEBUG CODE
  if (digitalRead(2) == LOW) {
    // generate artificial hang:
    Serial.println("Generating hang...");
    delay(10000);
  }
  
  wdt_reset();
}

ISR(WDT_vect) // Watchdog timer interrupt.
{
  digitalWrite(p_safety, HIGH);
  
  // 
  
  while (true) {
    Serial.println("Watchodg");
  }
  
  // To be filled with "useful" debug info later
  //EEPROM.write(crash_report_addr, 1);
  
}
