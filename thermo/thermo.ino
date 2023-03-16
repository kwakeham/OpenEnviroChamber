/*
Frigidbear
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ArduPID.h"
#inc

#define compressor_rest_time 60000 //5 minutes?


#define compressor_pin 12
#define heater_pin 13

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MAXCS   10
Adafruit_MAX31855 thermocouple(MAXCS);

ArduPID myController;

enum chamber_state_t {
  unknown_state,
  idle_state,
  cooling_state,
  heating_state
};

int menu_xpos = 0;
int menu_ypos = 0;

int menu_xmove = 0;
int menu_ymove = 0;

int cold_temp = 15;
int hot_temp = 40;
int cold_time = 15;
int hot_time = 15;

int count;

bool system_running = false;

chamber_state_t chamber_state = idle_state;
double chamber_temp;

unsigned long OldTime;
unsigned long counter;
bool currentstatus = false;
bool compressor_running = false;
unsigned long compressor_stop_time;
bool compressor_state = false;

bool temp_stable = false;
unsigned long temp_stable_time = 0;

//ardupid
double input;
double output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 0;
double p = 30;
double i = 0.1;
double d = 0;


void setup() {

  Serial.begin(9600);

//  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  // initialize the display
  // note you may have to change the address
  // the most common are 0X3C and 0X3D

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.display();

  //ensure the MAX31855 responds
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");

  //set the SSR pin output
  pinMode(compressor_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(heater_pin, OUTPUT);


  myController.begin(&input, &output, &setpoint, p, i, d);
  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(0, 180);
//  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(0, 100); // Growth bounds for the integral term to prevent integral wind-up
  
//  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
   myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
  
}

void loop(void) {

  joytstickupdate();
  statusupdate("Waiting", currentstatus);
  valueupdate();
  menuselect();
  readtemp();
  temperature_control();
  
  display.display();
  delay(100);
}

void temperature_control(void)
{
  if(system_running)
  {
    switch (chamber_state) {
      case unknown_state:
        break;
      case idle_state:
        break;
      case cooling_state:
        break;
      case heating_state:
        break;
      default:
        break;
    }
  }
  else //if not running make sure to shut off compressor
  {
    compressor_state = false;
    
    compressor_control(compressor_state);
  }
}

void compressor_control(bool run_compressor)
{

}

void compressor_control(uint heatvalue)
{
  analogWrite(heater_pin, 0); //ensure heater is off
}

void readtemp(void)
{
  double temporary_temp = thermocouple.readCelsius();
  if (isnan(temporary_temp)) {
     Serial.println("Something wrong with thermocouple!");
  }
  else
  {
    chamber_temp = temporary_temp;
    display.setTextColor(WHITE,BLACK);        // Draw white text
    display.setCursor(76,28);             // Start at top-left corner
    display.print(chamber_temp);
    display.print(F("c"));
  }
}

void statusupdate(String currentstatus, bool runningstatus)
{
  display.setTextSize(1);             // Normal 1:1 pixel scale
  if(system_running)
  {
    display.clearDisplay();
    display.fillRect(0, 0, SCREEN_WIDTH, 16, WHITE);
    display.setTextColor(BLACK);        // Draw white text
    display.setCursor(30,4);             // Start at top-left corner
    display.print("Running");
    
  } else
  {
    
    display.clearDisplay();
    display.drawRect(0, 0, SCREEN_WIDTH, 16, WHITE);
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(30,4);             // Start at top-left corner
    display.print(currentstatus);
  }
}



/////////////////////////////////////////////////////////
