/*
Frigidbear
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "ArduPID.h"
#include "Vrekrer_scpi_parser.h"

#define compressor_rest_time 600//00 //60 seconds

#define compressor_pin 12
#define heater_pin 11

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;

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

//two controllers because we might end up tuning differently
ArduPID heatController;
ArduPID coolController;

SCPI_Parser my_instrument;
int brightness = 0;
const int ledPin = 13;
const int intensity[11] = {0, 3, 5, 9, 15, 24, 38, 62, 99, 159, 255};


void setup() {

  Serial.begin(115200);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  // // initialize the display: note you may have to change the address the most common are 0X3C and 0X3D
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display.display();
  // display.clearDisplay();
  // display.display();

  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  uint32_t m = micros();
  oled.clear();
  oled.println("Hello world!");
  oled.println("A long line may be truncated");
  oled.println();
  oled.set2X();
  oled.println("2X demo");
  oled.set1X();
  oled.print("\nmicros: ");
  oled.print(micros() - m);
  
  //ensure the MAX31855 responds
  Serial.println("MAX31855 test");
  delay(200);  // wait for MAX chip to stabilize

  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("MAX31855 startup DONE.");

  //set the SSR and Mosfet pin output
  pinMode(compressor_pin, OUTPUT);
  pinMode(heater_pin, OUTPUT);

  //setup the two controllers.
  heatController.begin(&chamber_temp, &output, &setpoint, p, i, d);
  heatController.setOutputLimits(0, 180);
  heatController.setWindUpLimits(0, 100); // Growth bounds for the integral term to prevent integral wind-up
  heatController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
  coolController.begin(&chamber_temp, &output, &setpoint, p, i, d);
  coolController.setSampleTime(compressor_rest_time);      // This should prevent compressor starting more than once per compressor_reset_time

  //scpi setup
  statusupdate("Waiting", system_running);

  my_instrument.RegisterCommand(F("*IDN?"), &Identify);
  my_instrument.SetCommandTreeBase(F("SYSTem:ENVI"));
    my_instrument.RegisterCommand(F(":TEMPerature"), &SetTemperature);
    my_instrument.RegisterCommand(F(":TEMPerature?"), &GetTemperature);
    my_instrument.RegisterCommand(F(":COOL"), &GetTemperature);
    my_instrument.RegisterCommand(F(":HEAT"), &GetTemperature);
    
  my_instrument.PrintDebugInfo(Serial);

  pinMode(ledPin, OUTPUT);
  pinMode(LED_BUILTIN, INPUT);
  analogWrite(ledPin, 0);
}


void loop(void) {
  my_instrument.ProcessInput(Serial, "\n");
  readtemp();
  temperature_control();
  // display.display();
  delay(100);

}


//General system control
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


//MAX31855 Thermocouple
void readtemp(void)
{
  double temporary_temp = thermocouple.readCelsius();
  if (isnan(temporary_temp)) {
     Serial.println("Something wrong with thermocouple!");
  }
  else
  {
    chamber_temp = temporary_temp;
    oled.setCursor(90,2);
    oled.print(chamber_temp);
    oled.print("c");
  }
}


//OLED
void statusupdate(String currentstatus, bool runningstatus)
{
  // display.setTextSize(1);             // Normal 1:1 pixel scale
  // if(system_running)
  // {
  //   display.clearDisplay();
  //   display.fillRect(0, 0, SCREEN_WIDTH, 16, WHITE);
  //   display.setTextColor(BLACK);        // Draw white text
  //   display.setCursor(30,4);             // Start at top-left corner
  //   display.print("Running");
    
  // } else
  // {
    
  //   display.clearDisplay();
  //   display.drawRect(0, 0, SCREEN_WIDTH, 16, WHITE);
  //   display.setTextColor(WHITE);        // Draw white text
  //   display.setCursor(30,4);             // Start at top-left corner
  //   display.print(currentstatus);
  // }
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.print(currentstatus);
}

//SCPI

void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("Vrekrer,SCPI Dimmer,#00," VREKRER_SCPI_VERSION));
  //*IDN? Suggested return string should be in the following format:
  // "<vendor>,<model>,<serial number>,<firmware>"
}

void SetTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  // For simplicity no bad parameter check is done.
  if (parameters.Size() > 0) {
    brightness = constrain(String(parameters[0]).toInt(), 0, 10);
    analogWrite(ledPin, intensity[brightness]);
  }
}

void GetTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(String(brightness, DEC));
}