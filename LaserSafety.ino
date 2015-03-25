#include <EEPROM.h>
#include <avr/wdt.h>

int crash_report_addr = 0; // Adress for the first byte of crash reports, used for hang detection

// Pins
int p_waterflow = 3;
int p_pressure = 4;
int p_safety = 13; // The output pin to the laser controller

// Sensors
bool s_waterflow_ok = false;
bool s_pressure_ok = false;

bool safety_flag = false;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(p_safety, OUTPUT);
  digitalWrite(p_safety, LOW);
  
  Serial.begin(9600);
  Serial.println("Starting...");
  
  checkForCrash();
  
  watchdogSetup();
  
  pinMode(2, INPUT_PULLUP);
  pinMode(p_waterflow, INPUT_PULLUP);
  pinMode(p_pressure, INPUT_PULLUP);
}

void checkForCrash() { // Check EEPROM for crash byte

  byte crashByte = EEPROM.read(crash_report_addr);
  
  // If not 0, it was probably set by the WDT interrupt  
  if ( crashByte != 0) {
    // Clear byte (so we can reboot normally)
    EEPROM.write(crash_report_addr, 0);
    
    // Go into endless loop, write debug info to serial
    while (true) {
      Serial.println("Rebooted from Watchdog timer interrupt.");
      Serial.println("Write this down, restart, and hope for the best.");
      Serial.println(crashByte);
      delay(10000);
    }
  }
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
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);
  sei();
}


void get_sensor_states() {
  // s_waterflow_ok
  if (digitalRead(p_waterflow) == LOW) {
    s_waterflow_ok = true;
  } else {
    s_waterflow_ok = false;
  }
  
  // s_pressure_ok
  if (digitalRead(p_pressure) == LOW) {
    s_pressure_ok = true;
  } else {
    s_pressure_ok = false;
  }
  
}

void set_safety_flag() { // Checks, if all inputs indicate safe performance, then sets safety_flag
  if (
  s_waterflow_ok &&
  s_pressure_ok
  ) {
    safety_flag = true;
  }else {
    safety_flag = false;
  }
  
}

void loop() {
  
  get_sensor_states();
  set_safety_flag();
  digitalWrite(p_safety, safety_flag);
  
  
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
  // To be filled with "useful" debug info later
  EEPROM.write(crash_report_addr, 1);
}
