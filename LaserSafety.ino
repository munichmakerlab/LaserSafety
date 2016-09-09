//          PIN LAYOUT 
//                                       +-----+
//          +----[PWR]-------------------| USB |--+
//          |                            +-----+  |
//          |         GND/RST2  [ ][ ]            |
//          |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5 
//          |          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4 
//          |                             AREF[ ] |
//          |                              GND[ ] |
//          | [ ]N/C                    SCK/13[ ] |   B5   OUT    Laser Controller:     p_safety 
//          | [ ]IOREF                 MISO/12[ ] |   .
//          | [ ]RST                   MOSI/11[ ]~|   .
//          | [ ]3V3    +---+               10[ ]~|   .
//          | [ ]5v    -| A |-               9[ ]~|   .
//          | [ ]GND   -| R |-               8[ ] |   B0
//          | [ ]GND   -| D |-                    |
//          | [ ]Vin   -| U |-               7[ ] |   D7    I/O   One Wire bus:         p_onewire
//          |          -| I |-               6[ ]~|   D6    IN    Water leak sensor:    p_waterleak3
//          | [ ]A0    -| N |-               5[ ]~|   D5    IN    Water leak sensor:    p_waterleak2
//          | [ ]A1    -| O |-               4[ ] |   D4    IN    Water leak sensor:    p_waterleak1
//          | [ ]A2     +---+           INT1/3[ ]~|   D3    IN    Pressure Sensor:      p_pressure
//          | [ ]A3                     INT0/2[ ] |   D2    IN    Flow sensor:          p_flow_sensor
//          | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   D1
//          | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0
//          |            [ ] [ ] [ ]              |
//          |  UNO_R3    GND MOSI 5V  ____________/
//           \_______________________/
  
// Source:	  http://busyducks.com/ascii-art-arduinos

#ifdef DEBUG
 #define DEBUG_PRINT(x)   Serial.print (x)
 #define DEBUG_PRINTLN(x) Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x) 
#endif

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

bool temp1_fail_last_cycle = false;
bool temp2_fail_last_cycle = false;


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

void dim_leds() {
  for (int p=255 ; p >0; p--)   // Dimmrichtung dunkler
  {
    Set_LED_ALL(p);      
    delay(1);  // Warten
  } 
}

void Set_LED_ALL(int PWM)
{
  Wire.begin();            // I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x82);         // Startregister 02h 
  for (int i=1 ; i < 17; i++){  // 16Bytes (Register 02h bis 11h) schreiben
    Wire.write(PWM);
  }
  Wire.endTransmission();  // I2C-Stop
}

void i2c_setup() {
  Wire.begin();     //I2C-Start
  Wire.beginTransmission(B1100000); // TLC59116 Slave Adresse ->C0 hex
  Wire.write(0x80);  // autoincrement ab Register 0h

  Wire.write(0x00);  // Register 00 /  Mode1
  Wire.write(0x00);  // Register 01 /  Mode2

  // default all on (not fully) 
  Wire.write(0xF0);  // Register 02 /  PWM LED 1    
  Wire.write(0xF0);  // Register 03 /  PWM LED 2
  Wire.write(0xF0);  // Register 04 /  PWM LED 3
  Wire.write(0xF0);  // Register 05 /  PWM LED 4
  Wire.write(0xF0);  // Register 06 /  PWM LED 5
  Wire.write(0xF0);  // Register 07 /  PWM LED 6
  Wire.write(0xF0);  // Register 08 /  PWM LED 7
  Wire.write(0xF0);  // Register 09 /  PWM LED 8
  Wire.write(0xF0);  // Register 0A /  PWM LED 9
  Wire.write(0xF0);  // Register 0B /  PWM LED 10
  Wire.write(0xF0);  // Register 0C /  PWM LED 11
  Wire.write(0xF0);  // Register 0D /  PWM LED 12
  Wire.write(0xF0);  // Register 0E /  PWM LED 13
  Wire.write(0xF0);  // Register 0F /  PWM LED 14
  Wire.write(0xF0);  // Register 10 /  PWM LED 15
  Wire.write(0xF0);  // Register 11 /  PWM LED 16  

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

  // Deactivate Watchdog
  wdt_disable();
  
  pinMode(p_safety, OUTPUT);
  digitalWrite(p_safety, HIGH);

  Serial.begin(9600);
  DEBUG_PRINTLN("START;");

  // init Display
  i2c_setup();
  delay(700); 
  dim_leds();
  
  temp_setup();

  //pinMode(p_lid, INPUT_PULLUP);
  //pinMode(p_waterflow, INPUT_PULLUP);
  pinMode(p_pressure, INPUT);
  pinMode(p_waterleak1, INPUT);
  pinMode(p_waterleak2, INPUT); 
  pinMode(p_waterleak3, INPUT);
  pinMode(p_flow_sensor, INPUT);
  
  attachInterrupt(0, count_rpms_flow_sensor, RISING); //flow sensor

  // activate Watchdog
  wdt_enable(WDTO_2S);
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
  DEBUG_PRINTLN("check_flow()");
  volume = messure_flow_over_time();
  Serial.print (volume); //Prints the number calculated above (x ,DEC)
  Serial.print (" L/hour\r\n"); //Prints "L/hour" and returns a  new line
  
  if (volume > volume_min && volume < volume_max ) {
    DEBUG_PRINTLN("flow ok");
    return true;
  } else {
    DEBUG_PRINTLN("flow not ok");
    return false;
  }
}

float messure_flow_over_time() {
 DEBUG_PRINTLN("messure_flow_over_time()");
 NbTopsFan = 0;      //Set NbTops to 0 ready for calculations
 delay(1000);      //Wait 1 second
 return ((NbTopsFan * 60 / 7.5) / 60); //((Pulse frequency x 60) / 7.5Q,) / 60 = flow rate in L per minute (normal ~5)
}
    


void get_sensor_states() {

  // s_pressure_ok
  s_pressure_ok = check_generic_LOW(p_pressure);

  // s_waterleak
  s_waterleak1_ok = check_generic_HIGH(p_waterleak1);
  s_waterleak2_ok = check_generic_HIGH(p_waterleak2);
  s_waterleak3_ok = check_generic_HIGH(p_waterleak3);

  // s_temp1
  s_temp1 = temp_sensors.getTempC(water_inlet);
  // DEBUG_PRINTLN(s_temp1);

  if (s_temp1 > temp1_min && s_temp1 < temp1_max ) {
    s_temp1_ok = true;
    temp1_fail_last_cycle = false;
  } else {
    if (s_temp1 == -127.00 && ! temp1_fail_last_cycle){  //If the temp doesn't make sense but did last cycle
      s_temp1_ok = true; // Go on
      temp1_fail_last_cycle = true; // But remember that something was wrong
    } else { // It's just too low or high, but plausible
      s_temp1_ok = false;
    }
  }

  // s_temp2
  s_temp2 = temp_sensors.getTempC(water_outlet);
  // DEBUG_PRINTLN(s_temp2);

  if (s_temp2 > temp2_min && s_temp2 < temp2_max) {
    s_temp2_ok = true;
    temp2_fail_last_cycle = false;
  } else {
    if (s_temp2 == -127.00 && ! temp2_fail_last_cycle){  //If the temp doesn't make sense but did last cycle
      s_temp2_ok = true; // Go on
      temp2_fail_last_cycle = true; // But remember that something was wrong
    } else { // It's just too low or high, but plausible
      s_temp2_ok = false;
    }
  }
  
  s_waterflow_ok = check_flow();

  DEBUG_PRINTLN("get_sensor_states() done");
  
}

void set_safety_flag() { // Checks, if all inputs indicate safe performance, then sets safety_flag
  if (
    s_pressure_ok &&
    s_waterflow_ok &&
    s_waterleak1_ok &&
    s_waterleak2_ok &&
    s_waterleak3_ok &&
    s_temp1_ok &&
    s_temp2_ok 
  )
  {
    safety_flag = true;
    // DEBUG_PRINTLN("True");
  } else {
    safety_flag = false;
  }

}

void generic_display_state(int pin, bool state){
  Serial.print(state);
  if (state) {
    DEBUG_PRINTLN("true");
    Set_LED_PWM(pin, 255);
    Set_LED_PWM(pin+1, 0);
  } else {
    DEBUG_PRINTLN("false");
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
  Serial.print("\nTemps:");
  Serial.print("Out: ");
  Serial.print(s_temp1);
  Serial.print(" / In: ");
  DEBUG_PRINTLN(s_temp2);
  
}

void request_update_temp_sensors() {
  //global temperature request to all devices on the bus
  //cli();
  temp_sensors.requestTemperatures(); 
  temp_last_update = millis(); 
  //sei();
  
}

void loop() {

  // Reset Watchdog
 wdt_reset();
  
 if ( temp_last_update + temp_requests_time < millis() ) {
   // DEBUG_PRINTLN("Updating temp sensors");
   DEBUG_PRINTLN("-----");
   request_update_temp_sensors();
 }
 
 DEBUG_PRINTLN("get_sensor_states()");
 get_sensor_states();

 DEBUG_PRINTLN("set_safety_flag()");
 set_safety_flag(); 
 disable_laser = safety_flag; // If save operation not ok, disable laser (HIGH Output will disable the laser!)
 digitalWrite(p_safety, disable_laser); // Write out pin state 

 DEBUG_PRINTLN("update_display");
 update_display();
}
