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

#define compressor_rest_time 6000//0 //60 seconds

#define compressor_pin 13
#define heater_pin 12

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
double heat_output;
double cool_output;
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
  oled.clear();

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
  heatController.begin(&chamber_temp, &heat_output, &setpoint, p, i, d);
  heatController.setOutputLimits(0, 180);
  heatController.setWindUpLimits(0, 100); // Growth bounds for the integral term to prevent integral wind-up
  heatController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
    
  coolController.begin(&chamber_temp, &cool_output, &setpoint, p, i, d);
  coolController.setOutputLimits(-255, 0);
  coolController.setWindUpLimits(-100, 0); // Growth bounds for the integral term to prevent integral wind-up
  coolController.setSampleTime(compressor_rest_time);      // This should prevent compressor starting more than once per compressor_reset_time

  //scpi setup
  statusupdate("Waiting", system_running);

  my_instrument.RegisterCommand(F("*IDN?"), &Identify);
  my_instrument.SetCommandTreeBase(F("ENVI"));
    my_instrument.RegisterCommand(F(":SETTemperature"), &SetTemperature);
    my_instrument.RegisterCommand(F(":GETSettemperature?"), &GetSetTemperature);
    my_instrument.RegisterCommand(F(":TEMPerature?"), &GetTemperature);
    my_instrument.RegisterCommand(F(":COOL"), &ChamberMode);
    my_instrument.RegisterCommand(F(":HEAT"), &ChamberMode);
    my_instrument.RegisterCommand(F(":STAT?"), &ChamberState);
    my_instrument.RegisterCommand(F(":RUN"), &ChamberRun);
    my_instrument.RegisterCommand(F(":RUN?"), &isChamberRun);
    my_instrument.RegisterCommand(F(":STOP"), &ChamberStop);
    
  my_instrument.PrintDebugInfo(Serial);

  // pinMode(ledPin, OUTPUT);
  // analogWrite(ledPin, 0);
}


void loop(void) {
  readtemp();
  my_instrument.ProcessInput(Serial, "\n");
  temperature_control();
  // display.display();
  // coolController.debug(&Serial, "cc", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
  // delay(500);
  
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
        coolController.compute();
        compressor_control(cool_output);
        heater_control(0); //ensure off
        break;
      case heating_state:
        heatController.compute();
        heater_control(heat_output);
        compressor_control(false); //ensure off
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

void compressor_control(double coolvalue)
{
  if(coolvalue < -10)
  {
    compressor_run(true);
  }
  else
  {
    compressor_run(false);
  }

}

void compressor_run(bool compressor_run)
{
  digitalWrite(compressor_pin, compressor_run);
}

void heater_control(double heatvalue)
{
  analogWrite(heater_pin, heatvalue); //ensure heater is off
}


//MAX31855 Thermocouple
void readtemp(void)
{
  double temporary_temperature = thermocouple.readCelsius();
  if (isnan(temporary_temperature)) {
     Serial.println("Something wrong with thermocouple!");
  }
  else
  {
    chamber_temp = temporary_temperature;
    oled.setCursor(90,2);
    oled.print(chamber_temp);
    oled.print("c");
  }
}


//OLED
void statusupdate(String currentstatus, bool runningstatus)
{
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
    setpoint = constrain(String(parameters[0]).toInt(), 0, 255);
  }
}

void GetTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.print(":GETTemperature: ");
  interface.println(String(chamber_temp, DEC));
}

void GetSetTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.print(":GETSettemperature: ");
  interface.println(String(setpoint, DEC));
}

void ChamberMode(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  String last_header = String(commands.Last());
  last_header.toUpperCase();
  if (last_header.startsWith("HEAT")) {
    chamber_state = heating_state;
    heatController.start();
    coolController.stop();
  } else {
    chamber_state = cooling_state;
    heatController.stop();
    coolController.start();
  }
}
void ChamberState(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  if (chamber_state == heating_state)
  {
    interface.println(":heating_state");
  } else if (chamber_state == cooling_state)
  { 
    interface.println(":cooling_state");
  }
}

void ChamberRun(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  system_running = true;
}

void ChamberStop(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  system_running = false;
}

void isChamberRun(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  if (system_running)
  {
    interface.println("Chamber Running");
  } else
  { 
    interface.println("Chamber NOT Running");
  }
}