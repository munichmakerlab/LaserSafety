#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <avr/wdt.h>

// Pins
#define p_flow_sensor 2 
#define p_pressure 3
#define p_waterleak1 4
#define p_waterleak2 5
#define p_waterleak3 6

#define p_onewire 7

#define p_safety 13 // The output pin to the laser controller


// Sensor states
bool s_waterflow_ok = false;
bool s_pressure_ok = false;
bool s_waterleak1_ok = false;
bool s_waterleak2_ok = false;
bool s_waterleak3_ok = false;
bool s_temp1_ok = false;
bool s_temp2_ok = false;

bool safety_flag = false;
bool disable_laser = true;

// Temp stuff
float s_temp1;
float s_temp2;

float temp1_min = 8.0;
float temp1_max = 27;

float temp2_min = 8.0;
float temp2_max = 26;


// Dallas temp sensor adresses 
DeviceAddress water_outlet = {0x28, 0x04, 0xDD, 0xAC, 0x04, 0x00, 0x00, 0x55};
DeviceAddress water_inlet = {0x28, 0xC3, 0x0D, 0xAE, 0x04, 0x00, 0x00, 0x16};
OneWire oneWire(p_onewire);
DallasTemperature temp_sensors(&oneWire);

unsigned int temp_requests_time = 1000;
unsigned long temp_last_update;


// Flow sensor
float volume;
float volume_min = 4;
float volume_max = 7;
volatile int NbTopsFan; //measuring the rising edges of the signal                               

// ################## setup functions #################


void i2c_setup() {
  Wire.begin();     //I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x80);  // autoincrement ab Register 0h

  Wire.write(0x00);  // Register 00 /  Mode1
  Wire.write(0x00);  // Register 01 /  Mode2

  // default all on (not fully) 
  Wire.write(0xA0);  // Register 02 /  PWM LED 1    
  Wire.write(0xA0);  // Register 03 /  PWM LED 2
  Wire.write(0xA0);  // Register 04 /  PWM LED 3
  Wire.write(0xA0);  // Register 05 /  PWM LED 4
  Wire.write(0xA0);  // Register 06 /  PWM LED 5
  Wire.write(0xA0);  // Register 07 /  PWM LED 6
  Wire.write(0xA0);  // Register 08 /  PWM LED 7
  Wire.write(0xA0);  // Register 09 /  PWM LED 8
  Wire.write(0xA0);  // Register 0A /  PWM LED 9
  Wire.write(0xA0);  // Register 0B /  PWM LED 10
  Wire.write(0xA0);  // Register 0C /  PWM LED 11
  Wire.write(0xA0);  // Register 0D /  PWM LED 12
  Wire.write(0xA0);  // Register 0E /  PWM LED 13
  Wire.write(0xA0);  // Register 0F /  PWM LED 14
  Wire.write(0xA0);  // Register 10 /  PWM LED 15
  Wire.write(0xA0);  // Register 11 /  PWM LED 16  

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


void temp_setup() {

  temp_sensors.begin();
  temp_sensors.setResolution(water_inlet, 10);
  temp_sensors.setResolution(water_outlet, 10);

  temp_sensors.requestTemperatures();
  temp_last_update = millis(); 
}

void setup() {
  pinMode(p_safety, OUTPUT);
  digitalWrite(p_safety, HIGH);

  Serial.begin(9600);
  Serial.println("START;");

  i2c_setup();  // Display
  temp_setup();

  //pinMode(p_lid, INPUT_PULLUP);
  //pinMode(p_waterflow, INPUT_PULLUP);
  pinMode(p_pressure, INPUT);
  pinMode(p_waterleak1, INPUT);
  pinMode(p_waterleak2, INPUT); 
  pinMode(p_waterleak3, INPUT);
  pinMode(p_flow_sensor, INPUT);
  
  attachInterrupt(0, count_rpms_flow_sensor, RISING); //flow sensor

  // Watchdog disabled.
  // **** YOU NEED TO FIX THE TIMING AND ENABLE IT BEFORE USING THIS!
  // watchdogSetup();
}


// ################## loop functions #################

void Set_LED_PWM(int LED, int PWM)
{
  Wire.begin();             //I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x01 + LED);    // Register LED-Nr
  Wire.write(PWM);
  Wire.endTransmission();   // I2C-Stop
}

void count_rpms_flow_sensor()     //This is the function that the interupt calls 
{ 
 NbTopsFan++;  //This function measures the rising and falling edge of the flow sensor 
}


bool check_generic_LOW(int pin) {
  if (digitalRead(pin) == LOW) {
    return true;
  } else {
    return false;
  }
}

bool check_generic_HIGH(int pin) {
  if (digitalRead(pin) == HIGH) {
    return true;
  } else {
    return false;
  }
}

bool check_flow() {
  volume = messure_flow_over_time();
  Serial.print (volume); //Prints the number calculated above (x ,DEC)
  Serial.print (" L/hour\r\n"); //Prints "L/hour" and returns a  new line
  
  if (volume > volume_min && volume < volume_max ) {
    return true;
  } else {
    return false;
  }
}

float messure_flow_over_time() {
 NbTopsFan = 0;      //Set NbTops to 0 ready for calculations
 delay(1000);      //Wait 1 second
 return ((NbTopsFan * 60 / 7.5) / 60); //((Pulse frequency x 60) / 7.5Q,) / 60 = flow rate in L per minute (normal ~5)
}
    


void get_sensor_states() {

  // s_lid_ok
  //if (digitalRead(p_lid) == LOW) {
  //  s_lid_ok = true;
  //} else {
  //  s_lid_ok = false;
  //}

  // s_pressure_ok
  s_pressure_ok = check_generic_LOW(p_pressure);

  // s_waterleak
  s_waterleak1_ok = check_generic_HIGH(p_waterleak1);
  s_waterleak2_ok = check_generic_HIGH(p_waterleak2);
  s_waterleak3_ok = check_generic_HIGH(p_waterleak3);

  // s_temp1
  s_temp1 = temp_sensors.getTempC(water_inlet);
  Serial.println(s_temp1);

  if (s_temp1 > temp1_min && s_temp1 < temp1_max ) {
    s_temp1_ok = true;
  } else {
    s_temp1_ok = false;
  }

  // s_temp2
  s_temp2 = temp_sensors.getTempC(water_outlet);
  Serial.println(s_temp2);

  if (s_temp2 > temp2_min && s_temp2 < temp2_max) {
    s_temp2_ok = true;
  } else {
    s_temp2_ok = false;
  }
  
  s_waterflow_ok = check_flow();
  
}

void set_safety_flag() { // Checks, if all inputs indicate safe performance, then sets safety_flag
  if (
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

void generic_display_state(int pin, bool state){
  Serial.print(state);
  if (state) {
    Set_LED_PWM(pin, 255);
    Set_LED_PWM(pin+1, 0);
  } else {
    Set_LED_PWM(pin, 0);
    Set_LED_PWM(pin+1, 255);
  }
}

void update_display() {
  // Write out sensor states to i2c
  // (Counting from 1!)
  
  Serial.print("States: ");
  generic_display_state(1, safety_flag);
  
  generic_display_state(3, s_pressure_ok);
  generic_display_state(5, s_waterflow_ok);
  generic_display_state(7, s_temp1_ok);
  generic_display_state(9, s_temp2_ok);
  generic_display_state(11, s_waterleak1_ok);
  generic_display_state(13, s_waterleak2_ok);
  generic_display_state(15, s_waterleak3_ok);
  Serial.println("");
}

void request_update_temp_sensors() {
  //global temperature request to all devices on the bus
  //cli();
  temp_sensors.requestTemperatures(); 
  temp_last_update = millis(); 
  //sei();
  
}

void loop() {
 if ( temp_last_update + temp_requests_time < millis() ) {
   Serial.println("Updating temp sensors");
   request_update_temp_sensors();
 }
 
 
 get_sensor_states();
 
 set_safety_flag(); 
 disable_laser = safety_flag; // If save operation not ok, disable laser (HIGH Output will disable the laser!)
 digitalWrite(p_safety, disable_laser); // Write out pin state

 update_display();
 
 
  // DEBUG CODE
  //if (digitalRead(13) == LOW) {
    // generate artificial hang:
  //  Serial.println("Generating hang...");
  //  delay(10000);
  //}

  //wdt_reset();
}
































// ######################## disused junk ##################




void doImportantStuff_outdated() {
  get_sensor_states(); // set all the s_ variables from input
  set_safety_flag(); // check all variables, then set safety_flag
  disable_laser = ! safety_flag; // If save operation not garateed, disable laser (HIGH Output will disable the laser!)
  digitalWrite(p_safety, disable_laser); // Write out pin state
  
  update_display(); // Send the states to the display panel via I2C
  
  Serial.println(safety_flag);
  wdt_reset(); // reset the watchdog timer
  
  // Maybe add something to verify the temperatures (in loop()) were updated some reasonable time ago
}

//ISR(WDT_vect) // Watchdog timer interrupt.
//{
//  digitalWrite(p_safety, HIGH);
//  Serial.println("Watchodg");
  //

//  while (true) {
    // Loop forever
//  }

//}


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

