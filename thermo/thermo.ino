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

#define compressor_rest_time 300000 //5 minutes
#define cool_loop_time 30000 //30 seconds, we want the cooling loop time to be slower because compressors respond slowly

#define compressor_pin 12
#define heater_pin 13
#define fan_pin 9

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
unsigned long compressor_stop_time = 0;

//ardupid
double heat_output;
double cool_output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 0;
double p_h = 30;
double i_h = 0.0002; //much lower I term
double d_h = 0;

double p_c = 11; //1 degree (gain of 10) should be good, but rounding and such so 11 it is
double i_c = 0.0002; //some i term for cooling
double d_c = 0;

//two controllers because we might end up tuning differently
ArduPID heatController;
ArduPID coolController;

SCPI_Parser my_instrument;

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

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
  pinMode(fan_pin, OUTPUT);
  digitalWrite(fan_pin, true); //fan always on for now
  compressor_run(0); //This should be okay as it shouldn't update the compressor_state (initialize to false) and therefore compressor_stop_time should still be 0 for edge case on statup
  heater_control(0);

  //setup the two controllers.
  heatController.begin(&chamber_temp, &heat_output, &setpoint, p_h, i_h, d_h);
  heatController.setOutputLimits(0, 255);
  heatController.setWindUpLimits(0, 100); // Growth bounds for the integral term to prevent integral wind-up
  heatController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
    
  coolController.begin(&chamber_temp, &cool_output, &setpoint, p_c, i_c, d_c);
  coolController.setOutputLimits(-255, 100); //allow some heat, but not much because we won't update quickly and don't want to fight compressor with heater
  coolController.setWindUpLimits(-20, 0); // limit cooling antiwindup but no heating integral
  coolController.setSampleTime(cool_loop_time);      // Slow time, but not too slow, compressor protection code should ensure no damage to compressor

  //scpi setup
  statusupdate("Waiting", system_running);
  oled.setCursor(0,1);
  oled.print("Stopped");

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
}


void loop(void) {
  readtemp();
  my_instrument.ProcessInput(Serial, "\n");
  temperature_control();
  // display.display();
  delay(500);
  
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
        coolController.debug(&Serial, "cc", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
        break;
      case heating_state:
        heatController.compute();
        heater_control(heat_output);
        compressor_run(false); //ensure off
        heatController.debug(&Serial, "hc", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
        break;
      default:
        break;
    }
  }
  else //if not running make sure to shut off compressor
  {
    compressor_control(false);
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
  if (coolvalue > 0)
  {
    heater_control(coolvalue);
  } else
  {
    heater_control(0); //ensure off
  }

}
//guards the compressor
void compressor_run(bool compressor_run)
{
  //true if we want compressor to run
  if (compressor_run)
  {  
    //edge case of startup
    if(compressor_stop_time == 0)
    {
      compressor_state = true;
      digitalWrite(compressor_pin, true);
    }
    else if (millis() > (compressor_stop_time + compressor_rest_time))
    {
      compressor_state = true;
      digitalWrite(compressor_pin, true);
    }
    // If it wasn't a startup condition and it hasn't been enought time  to reset the compressor(compressor_rest_time) stay off
    else
    {
      digitalWrite(compressor_pin, false); // I don't think we need this but just in case
    }
  }
  //commanded not to run
  else
  {
    // If the compressor was running and we're now commanded to shut it off
    // Now we shouldn't be able to store the compressor state until it was on
    if (compressor_state)
    {
      compressor_state = false; //remember the state
      compressor_stop_time = millis();  //remember the time we shut it off
      Serial.print("Stop time: "); //a little debug
      Serial.println(compressor_stop_time);
    }
    //always allow shutdown of the compressor
    digitalWrite(compressor_pin, false);
  }  
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
    oled.setCursor(0,0);
    oled.print("Heating");
  } else {
    chamber_state = cooling_state;
    heatController.stop();
    coolController.start();
    oled.setCursor(0,0);
    oled.print("Cooling");
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
  heatController.reset();//reset the I and D terms for heat controller
  coolController.reset(); //reset the I and D terms for the cooling controller, though it's unlikely we have I and D
  oled.setCursor(0,1);
  oled.print("Running");

}

void ChamberStop(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  system_running = false;
  oled.setCursor(0,1);
  oled.print("Stopped");
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