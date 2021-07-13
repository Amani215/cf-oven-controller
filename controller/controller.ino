#include "pid.h"
#include "thermocouple.h"
#include "LiquidCrystal_I2C.h"


// Pin numbers are arbitrary - change it before flashing
#define LCD_PRESENT true
#define LCD_ADDRESS 0x27
#define TEMP_CALIBRATION true
#define PHOTO_TRIAC_PIN 13
#define MAX_TEMPERATURE 142     // In Farenheit
#define CURE_TIME 8*3600        // Corresponds to ~142 Franeheit (different when MAX_TEMPERATURE is different)
#define HEATING_TIME 2          // ASSUMPTION (This is the time for the heater to go up by 1 Celsius/Farenheit)
#define INCREMENT_PERIOD 60000  // Increment the temperature every minute

// Setup the LCD Matrix
LiquidCrystal_I2C lcd(LCD_ADDRESS,20,4);

// Setup the thermocouples, use Fahrenheit
// For now, use one thermocouple for ease of setup and testing
// Set the second argument to true if Celsius is desired.
Thermocouple t1(A0, false, 0.5);
Thermocouple t2(A0, false, 0.5);
Thermocouple t3(A0, false, 0.5);
Thermocouple t4(A0, false, 0.5);
Thermocouple t5(A0, false, 0.5);

// PID object params
double dt = 0.1;          // loop interval time
double max_out = 1;     // maximum allowable output from pid
double min_out = -1;    // minimum allowable output from pid
double Kp = 0.01;          // proportional gain
double Kd = 0.01;         // derivative gain
double Ki = 0.5;          // integral gain

// Create pid object with params
PID pid = PID(dt, max_out, min_out, Kp, Kd, Ki);

// Variables for test/debug
double test_setpoint = 60;

//Variables for the heating sequence
unsigned long heating_started_time = 0;
unsigned long prev_minute_time;
int can_heat = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Serial started.\n");

  if (LCD_PRESENT) {
    // Initialize LCD, wait for 5 sec
    Serial.print("Initializing LCD.\n");
    lcd.init();
    lcd.backlight();
    lcd.clear();
  }

  pinMode(PHOTO_TRIAC_PIN, OUTPUT);
  
  //starting point of the program (for the heating sequence)
  prev_minute_time = millis();
}

void loop() {

  if (TEMP_CALIBRATION) {
    // Log the temperature to serial output
    // We'll save this to an output file on a companion computer
    Serial.print("T1, ");
    Serial.print(t1.read());
    Serial.println("");

    Serial.print("T2, ");
    Serial.print(t2.read());
    Serial.println("");

    Serial.print("T3, ");
    Serial.print(t3.read());
    Serial.println("");

    Serial.print("T4, ");
    Serial.print(t4.read());
    Serial.println("");

    Serial.print("T5, ");
    Serial.print(t5.read());
    Serial.println("");
  }

  // For now, we'll average all the temp readings in F
  float avg_temp = (t1.read() + t2.read() + t3.read() + t4.read() + t5.read()) / 5;

  // Calculate this interval's control output
  double control_output = pid.calculate(test_setpoint, avg_temp);

  // Shut the oven down after the cure time passes
  if((millis() - prev_minute_time >= CURE_TIME) && (test_setpoint == MAX_TEMPERATURE)){
    digitalWrite(PHOTO_TRIAC_PIN, 0);
  }
  else
  {
    // Actuate the heater 
    if(control_output > 0 && can_heat){
      heating_started_time = millis();
      digitalWrite(PHOTO_TRIAC_PIN, 1);
      can_heat = 0;
    }else if(millis() - heating_started_time >= HEATING_TIME * control_output){
        digitalWrite(PHOTO_TRIAC_PIN, 0);
        can_heat = 1;
    }

    // Increase the temperature by 2 Farenheit every minute until max temperature is reached
    if((millis() - prev_minute_time >= INCREMENT_PERIOD) && (test_setpoint < MAX_TEMPERATURE)){
      test_setpoint = test_setpoint + 2 > MAX_TEMPERATURE ? MAX_TEMPERATURE : test_setpoint + 2;
      prev_minute_time = millis();
    }
  }

  if (LCD_PRESENT) {
    // Refresh the LCD screen
    lcd.clear(); // Clear the screen

    lcd.setCursor(0,0);
    lcd.print("Temp: ");
    lcd.print(avg_temp);
    lcd.print(" F.");

    lcd.setCursor(0,1);
    lcd.print("CtrlVal: ");
    lcd.print(control_output);
  }

  // Delay the loop for human readable debugging
  delay(300);
}
