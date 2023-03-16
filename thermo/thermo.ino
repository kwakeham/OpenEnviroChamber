/*
Frigidbear
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ArduPID.h"
#include "Vrekrer_scpi_parser.h"

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

enum chamber_state_t {
  unknown_state,
  idle_state,
  cooling_state,
  heating_state
};

bool system_running = false;
bool compressor_state = false;
chamber_state_t chamber_state = idle_state;
double chamber_temp;

//ardupid
double output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 0;
double p = 30;
double i = 0.1;
double d = 0;

ArduPID heatController;
ArduPID coolController;



void setup() {

  Serial.begin(115200);

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

  //set the SSR and Mosfet pin output
  pinMode(compressor_pin, OUTPUT);
  pinMode(heater_pin, OUTPUT);


  heatController.begin(&chamber_temp, &output, &setpoint, p, i, d);
  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  heatController.setOutputLimits(0, 180);
  // myController.setBias(255.0 / 2.0);
  heatController.setWindUpLimits(0, 100); // Growth bounds for the integral term to prevent integral wind-up
  // myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  heatController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
  
  coolController.begin(&chamber_temp, &output, &setpoint, p, i, d);
  coolController.setSampleTime(compressor_rest_time);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls

  
}

void loop(void) {
  statusupdate("Waiting", system_running);
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
    heater_control(0);
  }
}

void compressor_control(bool run_compressor)
{
  // digitalWrite(compressor_pin, HIGH);
}

void heater_control(unsigned int heatvalue)
{
  analogWrite(heater_pin, heatvalue); //ensure heater is off
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
