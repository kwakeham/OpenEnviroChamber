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

#define joyx A0
#define joyy A1
#define threshold 200

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

  // establish whatever pin reads you need for joystick  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

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
  // statusupdate("Waiting", currentstatus);
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
        analogWrite(heater_pin, 0); //ensure heater is off
        if (chamber_temp >= cold_temp)
        {
          compressor_state = true;
        } else if (chamber_temp+1 < cold_temp)
        {
          compressor_state = false;
        }

        //if the temperature mets requirements, start timer and check
        if (chamber_temp+1 < cold_temp)
        {
          if(temp_stable)
          {
            if(millis() > (temp_stable_time + (long)cold_time*1000*60))
            {
              temp_stable = false;
              chamber_state = heating_state;
              myController.start(); //start the heating controller
            }
          }
          else
          {
            temp_stable = true;
            temp_stable_time = millis();       
          }
        }
        Serial.println("coolingstate");
        break;
      case heating_state:
        compressor_state = false;
        //if the temperature mets requirements, start timer and check
        input = chamber_temp;
        setpoint = hot_temp;
        myController.compute();
        analogWrite(heater_pin, output);
        if (chamber_temp >= hot_temp-1)
        {
          if(temp_stable)
          {
            if(millis() > (temp_stable_time + (long)hot_time*1000*60))
            {
              temp_stable = false;
              chamber_state = cooling_state;
              myController.stop(); 
            }
          }
          else
          {
            temp_stable = true;
            temp_stable_time = millis();
          }
        }
        Serial.print(output);
        Serial.println(" ,heatingstate");
        break;
      default:
        // statements
        break;
    }
  }
  else //if not running make sure to shut off compressor
  {
    compressor_state = false;
    analogWrite(heater_pin, 0); //ensure heater is off
  }

  compressor_control(compressor_state);
}

void compressor_control(bool run_compressor)
{
  bool actualrun = false;
  if(run_compressor) //turn on compressor
  {
    if(compressor_running)
    {
      //should have enabled from before
      actualrun = true;
    } else
    { //check to see if we can turn on the compressor
      if (millis() > (compressor_stop_time + compressor_rest_time))
      {
        digitalWrite(compressor_pin, HIGH);
        actualrun = true;
        compressor_running = true;
      }

    }
  }
  else //turn off compressors
  {
    if(compressor_running)
    { //get time
      compressor_stop_time = millis();
      digitalWrite(compressor_pin, LOW);
      actualrun = false;
    } else
    {
      //ensure it's still off?
      digitalWrite(compressor_pin, LOW);
      actualrun = false;
    }
    compressor_running = false;
    
  }

  Serial.print("temp: ");
  Serial.print(chamber_temp);
  Serial.print(" compressor: ");
  Serial.print(run_compressor);
  Serial.print(" compressor_actual: ");
  Serial.println(actualrun);
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

void joytstickupdate(void)
{
  int x = analogRead(joyx);
  int y = analogRead(joyy);
  if(x > (512+threshold))
  {
      menu_xmove = 1;
  }
  
  if(x < (512-threshold))
  {
      menu_xmove = -1;
  }

  if(y > (512+threshold))
  {
      menu_ymove = 1;
  }
  
  if(y < (512-threshold))
  {
      menu_ymove = -1;
  }

  //prevent cursor movment unless exit
  if(system_running)
  {menu_ymove = 0;}

  menu_ypos = menu_ypos+menu_ymove;
  menu_ymove = 0;
  //Max out menus
  if(menu_ypos > 4)
  {menu_ypos = 4;}
  if(menu_ypos < 0)
  {menu_ypos = 0;}
}

void valueupdate()
{
  if(!system_running)
  {
    switch (menu_ypos) {
      case 0:
        cold_temp = cold_temp+menu_xmove;
        break;
      case 1:
        cold_time = cold_time+menu_xmove;
        break;
      case 2:
        hot_temp = hot_temp+menu_xmove;
        break;
      case 3:
        hot_time = hot_time+menu_xmove;
        break;
      case 4:
        if(menu_xmove>0)
        {
          system_running = true;
          chamber_state = cooling_state;
          myController.start();
        }
        break;
      default:
        // statements
        break;
    }    
  }
  else
  {
    if (menu_xmove<0)
    {
      system_running = false;
      chamber_state = idle_state;
    }
  }
  menu_xmove = 0;
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

void menuselect()
{
  //Cold
  display.setTextColor(WHITE,BLACK); 
  display.setCursor(4,16);             // Start at top-left corner
  display.print(F("COLD"));
  if(menu_ypos == 0)
  {
    display.setTextColor(BLACK,WHITE);        // Draw white text
  } else
  {
    display.setTextColor(WHITE,BLACK);        // Draw white text
  }
  display.setCursor(4,28);             // Start at top-left corner
  display.print(cold_temp);
  display.print(F("c"));
  display.setTextColor(WHITE,BLACK); 
  display.setCursor(4,40);             // Start at top-left corner
  display.print(F("TIME"));
  if(menu_ypos == 1)
  {
    display.setTextColor(BLACK,WHITE);        // Draw white text
  } else
  {
    display.setTextColor(WHITE,BLACK);        // Draw white text
  }
  display.setCursor(4,52);             // Start at top-left corner
  display.print(cold_time);
  display.print(F("m"));

  //Hot
  display.setTextColor(WHITE,BLACK);        // Draw white text
  display.setCursor(40,16);             // Start at top-left corner
  display.print(F("HOT"));
  if(menu_ypos == 2)
  {
    display.setTextColor(BLACK,WHITE);        // Draw white text
  } else
  {
    display.setTextColor(WHITE,BLACK);        // Draw white text
  }
  display.setCursor(40,28);             // Start at top-left corner
  display.print(hot_temp);
  display.print(F("c"));
  display.setTextColor(WHITE,BLACK);        // Draw white text
  display.setCursor(40,40);             // Start at top-left corner
  display.print(F("TIME"));
  if(menu_ypos == 3)
  {
    display.setTextColor(BLACK,WHITE);        // Draw white text
  } else
  {
    display.setTextColor(WHITE,BLACK);        // Draw white text
  }
  display.setCursor(40,52);             // Start at top-left corner
  display.print(hot_time);
  display.print(F("m"));

  //START
  if(menu_ypos == 4)
  {
    display.setTextColor(BLACK,WHITE);        // Draw white text
  } else
  {
    display.setTextColor(WHITE,BLACK);        // Draw white text
  }
  display.setCursor(76,16);             // Start at top-left corner
  display.print(F("START"));
}

/////////////////////////////////////////////////////////
