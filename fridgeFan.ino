

/*
25/4/23 Mike Garner
Fridge Fan Control with 2.2" TFT Display 220*176 pixels
16/6/23
Adjusted display brightness mapping lines 282 & 283
25/07/23 
Added water sensor to unit to sound buxxer if water detected
Use D7 for sensor D12 as output for buzzer

Arduino Pins:
 A1  	Light sensor
 P2 	DHT22 temp & humidity sensor. (Blue wire)
 P3 	PWM pin for fan (White wire) 
 P4   N/C
 D5   N/C
 D6 	PWM pin for tft led
 D7		Water sensor
 D8 	TFT Reset
 D9  	TFT RS
 D10 	TFT CS	
 D11  TFT SDI (MISO)
 D12 	Buzzer 
 D13 	TFT CLK

 3.3v supply (1): Display
 5v supply (5)  : DHT22; Light Sensor; Water Sensor; Buzzer; Fan Transistor
 Ground (6)     : Uno has 3 slots

*/

//Libraries
#include <DHT.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "TFT_22_ILI9225.h"
#include <../fonts/FreeSans24pt7b.h>
#include <../fonts/FreeSerifBold9pt7b.h>
#include <../fonts/Org_01.h>

#define TFT_RST 8
#define TFT_RS  9
#define TFT_CS  10  // SS
#define TFT_SDI 11  // MOSI
#define TFT_CLK 13  // SCK
#define TFT_LED 6   // 0 if wired to +5V directly
#define TFT_BRIGHTNESS 200

char myString[6];       //Array for float to string conversions

#define DHTTYPE DHT22
#define DHTPIN 2          // digital pin DHT sensor connected to
#define h2oPIN 7          // digital pin water sensor connected to

DHT dht(DHTPIN, DHTTYPE);

// Use software SPI (slower)
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK, TFT_LED, TFT_BRIGHTNESS);

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const byte LIGHT = A1;            //Analog pin for light sensor data
const byte PWM_595 = 3;           //PWM pin for led to tft

//Global Variables ****************************************************************
long lastSecond;        //The millis counter to see when a second rolls by

//Fridge temp & control variables
int fan = false;        //Set fan to "0" at start
float fanSpeed;         
float fanPercent;
float temp_C = 0;
float temp_C_Old = 0;
int tempMin = 10;       //temperature at which fan starts
int tempMax = 40;       //temperature at which fan at 100%
int fanPin = 3;         //pin for fan control
String fridgeTempStr = ("");

int ledLvl;             //Variable to map light level for PWM
int lightSensor;        //Variable for light sensor
int h2o;                //Variable for water sensor
int buzzer = 12;        //Buzzer pin
int i = 1;              //Used to create frequency for buzzer

//TFT Co-ordinate
int16_t x=0, y=0, w, h;	//Positions on TFT Signed integers
int barLeft;				    //Left side of temp bar graph
int barRight;				    //Right side of temp bar graph

//***********************************************************************

void setup()
{
  	Serial.begin(115200);	    // start serial port
    
    pinMode(7,INPUT);        // initialize water sensor pin
    pinMode(buzzer, OUTPUT); // initialize pin 5
    
  	dht.begin();			//Start the DHT sensor 
  	delay(3000);			//Give sensor time to respond

  	tft.begin();
  	tft.clear(); 
  	tft.setOrientation(3);  //Put screen in landscape mode
// Draw header string
	
	tft.fillRectangle(0, 25,220,0, COLOR_DARKBLUE); //Background for heading
	tft.setGFXFont(&FreeSerifBold9pt7b);  			// Set font
 	String heading = "FRIDGE MONITOR";
 	tft.getGFXTextExtent(heading, x, y, &w, &h); 	// Get string extents
  	tft.drawGFXText(16, 21, heading, COLOR_WHITE); // Print string

// Draw line under heading
	tft.fillRectangle(x, 26,220,27, COLOR_GOLD);

//Draw box around temperature
	tft.drawRectangle(20, 45,190,100, COLOR_WHITE);

// Place "c" char on screen 
	String myChar = "c";
	tft.setGFXFont(&FreeSans24pt7b);  					// Set font
	tft.drawGFXText(155, 85, myChar, COLOR_WHITE); 		// Print string

// Place % ticks above graph
	String myPercent = " 0%    25%    50%    75%    100%";
	String myFanSpeed = "FAN SPEED";
	tft.setGFXFont(&Org_01);
	tft.drawGFXText(0, 132, myPercent, COLOR_WHITE); // Print string
	tft.drawGFXText(70, 168, myFanSpeed, COLOR_WHITE); // Print string

//Check we are getting a reading from DHT22 sensor
	temp_C = dht.readTemperature();
	if (isnan (temp_C))
    { 
//Pass reading to routine which will loop while (t_dht) is not a number (isnan)
      Serial.println("DHT sensor offline!");
      //dht_OffLine(temp_C);
    }
    Serial.println("DHT sensor online!");

//Set start of program to enable readings at specific intervals
  //int seconds = 0;
  lastSecond = millis();    //Returns the number of miliseconds program has been running
}

//End of Void Setup ********************************************

//Routine "smartDelay" for a given amount of time (ms) --------------------------------
static void smartDelay(unsigned long ms)  
{
  unsigned long start = millis(); //time program been running
  do 
  {
    delay(100);
  } while (millis() - start < ms);  //Exit once time ticks over ms value
}
//End of "smartDelay" routine -------------------------------------------

//Routine dht_OffLine if sensor fails (isnan). Put fans to 100% ---------------------------------
float dht_OffLine (float temp_C)
{
   if (isnan(temp_C))
   {
      do
      {
        analogWrite(fanPin, 255);   // Fan to full 
        Serial.print("DHT sensor offline ");
        delay(2000);

//Re-read DHT sensor
        temp_C = dht.readTemperature();
      }
//Check again to see if we have a valid reading 
      while (isnan(temp_C));
    }
//If valid reading return temp_C and back to main loop  
    return temp_C;  //return fridgeTemp;
}
//End of "dht_OffLine" routine ----------------------------------

//Routine "fanControl" to set fan speed. ------------------------------
//On initial startup fan will stall if rpm too low.
//So start fan at full then once running map to appropriate level

int fanControl(float temp_C)
{
  if (fan != true)   //Not true ie has not been running
  {
    Serial.println("1st start ");
    fanSpeed = 255; 
    return fanSpeed;    
  }
  else            //Has been running so map
  {
    fanSpeed = map(temp_C, tempMin, tempMax, 25, 255);  //25 = ~10% of max
    fanSpeed = constrain(fanSpeed, 25, 255);
    return fanSpeed; 
  }
}
//End of "fanControl" routine -----------------------------------------

//Routine "displayTemp_C" on tft screen
void displayTemp_C ()
{
//Only run if temperature has changed
	if (temp_C != temp_C_Old)
	{
		fridgeTempStr = dtostrf(temp_C,5,2,myString);  		//Create string to display tt.tt
		tft.getGFXTextExtent(fridgeTempStr, x, y, &w, &h); 	// Get string extents
		x = 30, y = 85;										//Set display co-ordinates x & y
		tft.fillRectangle(x, y+1, w+x+2, y-h, COLOR_BLACK);	//Place rectangle over previous data
		tft.setGFXFont(&FreeSans24pt7b);  					// Set font
		tft.drawGFXText(x, y, fridgeTempStr, COLOR_GREEN);	// Print string
		temp_C_Old = temp_C;
	}

//Display Bar Graph for fan speed
		if(fanSpeed > 254)	//Display at maximum for fans 100%
		{
		x = 10, y = 155, w = 210, h = 140;				//Set co-ords for 200 pixels wide 
		tft.fillRectangle(x, y, w, h, COLOR_GREEN); 
		}
		else if(fanSpeed < 255)							//Less than maximum
		{
		barLeft = fanPercent*2;							//1% = 2 pixels
		x = 10, y = 155, (w = barLeft+10), h = 140;		//Set display co-ordinates left  
		tft.fillRectangle(x, y, w, h, COLOR_GREEN); 	//Set left bar
		x = (barLeft + 10);								//Start of right bar (from pixel)
		w = (tft.maxX()-10);							//Width of right bar (to pixel)
		tft.fillRectangle(x, y, w, h, COLOR_SNOW);		//Draw right bar
	}
}	
//End of routine to display temperature----------------------------------------------------

//Sound buzzer if water sensed
void buzz()
// Output frequency to buzzer
  {
    for(i=0;i<90;i++)
    {
      digitalWrite(buzzer,HIGH);
      delay(1);//wait for 1ms
      digitalWrite(buzzer,LOW);
      delay(1);//wait for 1ms
    }
    //output another frequency
    delay(50);
    for(i=0;i<100;i++)
    {
      digitalWrite(buzzer,HIGH);
      delay(2);//wait for 2ms
      digitalWrite(buzzer,LOW);
      delay(2);//wait for 2ms
    }
 }
   
//End of buzzer routine----------------------------------------------------------


//Routine "displayData" Collation of data for display ---------------
void displayData ()
{

  Serial.println();
  Serial.println("Current sensor readings: ");
  
  //Fridge temperature & fan Speed
  Serial.print("Fridge ");
  Serial.print(temp_C, 2); 
  Serial.print("\xC2\xB0"); //Prints the degree symbol
  Serial.print("C ; ");
  Serial.print(" Fan Speed PWM = ");
  Serial.print(fanSpeed);
  Serial.print(" Fan ");
  Serial.print(fanPercent, 0);
  Serial.println(" %");
   
  //Light level data
  Serial.print("lightSensor = ");
  Serial.print(lightSensor);
  Serial.print(" ledLvl,"); 
  Serial.println(ledLvl); 

  //Water sensor data
  Serial.print("Water Sensor = ");
  Serial.println(h2o);

}
//End of "displayData" routine ---------------------------------------------

//*********************************************************************

void loop()
{
//Read DHT sensor
      temp_C = dht.readTemperature();
      if (isnan (temp_C))
     { 
//Pass reading to routine which will loop while (temp_C) is not a number (isnan)
      dht_OffLine(temp_C);
      }
//Fan Control
  if (temp_C<tempMin)       //Skip mapping routine no fan required
    {
      fan = false;          //Set fan to False not running
      fanSpeed = 0;         //No fan needed
    }
  if (temp_C>tempMax)       //Override fancontrol fans at max
    {
      fan = true;
      fanSpeed = 255;
    }
  else 
    {
      fanSpeed = fanControl(temp_C);  	//Access mapping routine
      fan = true;                     	//Set fan to 1 as now running
    }  
	fanPercent = (fanSpeed/255)*100;    //Gives percent of full speed
  	analogWrite(fanPin, fanSpeed);      // sends PWM signal to pin_D3 for fan control 

//Read water sensor at pin 7
//If high (=1) activate buzzer
  h2o = digitalRead(7);
  if (h2o == 1)
  buzz();
  
//Read light sensor at A1
	lightSensor = analogRead(LIGHT);
//Serial.println (lightSensor);
//Will give value from 0 (dark) to 1024 (bright). 
//Sensor located inside so map 0 to 900 rather than max 1024
  	ledLvl = map(lightSensor, 0, 900, 50, 255);  //Full on when bright dim when dark
  	ledLvl = constrain(ledLvl, 50, 255); 

//Adjust led bar graph brightness full daytime dim nightime
 	analogWrite(TFT_LED, ledLvl);      // sends PWM signal to pin_6 for led lux control 

  	displayTemp_C ();					//Display temperature on tft

  	displayData();

	smartDelay(5000);                   //Delay  5 secs between readings
}
