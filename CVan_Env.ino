/******************************************************************************
  SparkFun Si7021 Breakout  
  Hardware Connections:
      HTU21D ------------- Photon
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- D1/SCL
       DA ------------------- D0/SDA

  Hardware Platform: SparkFun RedBoard     Arduino IDE 1.6.5
    2nd temp sensor is 1 wire Dallas DS18B20 connected to pin 2
    Pressure sensor is MPL3115A2 can also supply temp and altitude 
  
  GPS is Tiny GPS; Uno RX = 5 TX = 4 ; Used baud of 9600 not 4800 for software serial
  
    Liquid Crystal 20*4 display 
    Pin configuration
      lcd 1   Grd Display
      lcd 2   +5v Display
      lcd 3   Display contrast via pot     
      lcd 4   RS      to Uno 7
      lcd 5   RW      Grd as we are only writing to display   
      lcd 6   Enable  to Uno 8
      lcd 7   n/c
      lcd 8   n/c 
      lcd 9   n/c 
      lcd 10  n/c
      lcd 11  DB4     to Uno 9
      lcd 12  DB5     to Uno 10
      lcd 13  DB6     to Uno 11
      lcd 14  DB7     to Uno 12
      lcd 15  +5v Backlight for PWM control to Uno 3
      lcd 16  Grd Backlight

  UNO Digital Pins used
  D0    RX  Used when gps set to UART NB set switch to SW to upload
  D1    TX  Used when gps set to UART NB set switch to SW to upload   
  D2    to DB18B20 data pin
  D3    to lcd 15 PWM for backlight  
  D4    TX pin for GPS
  D5    RX pin for GPS 
  D6    used somewhere on shield. Had issues with gps when tried to use as digital pin
  D7    to lcd 4 RS 
  D8    to lcd 6 Enable 
  D9    to lcd 11 DB4
  D10   to lcd 12 DB5
  D11   to lcd 13 DB6
  D12   to lcd 14 DB7
  D13   used by uno/shield led

  UNO Analog Pins used
  A0  to timezone button. Pulled high so when button pressed pulled low
  A1  to timezone led to indicate timezone being changed 
  A2  Ground likely used to ground shield
  A3  Power likely used to power shield
  A4 
  A5    

*******************************************************************************/
#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>
#include <OneWire.h>                //Required by DS18B20
#include <DallasTemperature.h>      //Required by DS18B20
#include "SparkFunMPL3115A2.h"      //Pressure sensor
#include "Adafruit_LiquidCrystal.h" //Liquid Crystal
#include <TinyGPS++.h>              //TinyGPS
#include <SoftwareSerial.h>         //SoftwareSerial
#include <TimeLib.h>                //Arduino time library

//No need to display decimals use int rather than float
int humidity = 0;
float tempf = 0;
int tempc = 0;
int externaltempc = 0;
int hpa = 0;  
int power = A3;
int GND = A2;
int backlightPin = 3;
int brightness = 128;
long TZ = 28800;              //WST = UTC+8hrs in seconds
int TZButtonState = 0;        //Timezone button monitor A0
int TZDirection = 1;          //Assumes starting in WST

unsigned long startMillis;
unsigned long last = 0UL; //Use for timer with millis()
unsigned long period;

//Create custom char House & Tree for In/Out temp display
byte house[8]={B00000,B00100,B01010,B10001,B01110,B01110,B01110,B00000};        //lcd char 1
byte tree[8]={B00100,B01110,B10101,B01110,B10101,B00100,B00100,B10101};         //lcd char 2
//Create custom char for latitude, longtitude & satelites
byte latitude[8] = {B11111,B00000,B11111,B00000,B11111,B00000,B11111,B00000};   //lcd char 3
byte longtitude[8]={B10101,B10101,B10101,B10101,B10101,B10101,B10101,B10101};   //lcd char 4
byte satelites[8]={B10001,B01010,B00100,B11111,B00100,B01010,B10001,B00000};    //lcd char 5
byte upArrow[8]={B00000,B00100,B01110,B10101,B00100,B00100,B00100,B00000};      //lcd char 6
byte rightArrow[8]={B00000,B00100,B000010,B11111,B00010,B00100,B00000,B00000};  //lcd char 7

//Create Instance of SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather sensor;
MPL3115A2 myPressure;
// DS18B20 data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20(&oneWire);

// Associate lcd pins with arduino 
const int RS = 7, EN = 8, DB4 = 9, DB5 = 10, DB6 = 11, DB7 = 12;

//Tiny GPS
static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;   //Baud rate for GPS 4800 not working

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Define variables for GPS data
unsigned long age;            //Used to get age of gps data
int gpsYear,gpsMonth,gpsDay;  //Calculated from gps data
int hr,min,sec;               //hours, minutes, seconds
const int offset = 8;         //Perth offset hours from UTC
String WeekDay = "";        
String TheMonth = "";
//Arrays for Days of Week & Months of Year
const char* DOW[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* MOY[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

float Lat, Long;
int Sats, altitude;
static char dToString[8];   //Create array to take float values for dtostrf function
String LatStr,LongStr;      //Strings for Lat & Long

//Co-Ordinates for distance & bearing calcs
//5 Urbahns -31.813900:115.745300  Sydney -33.865143:151.209900
float Urbahns_Lat = -31.813900;   
float Urbahns_Lon = 115.745300;
int distanceTo5Urbahns, courseTo5Urbahns ;
const char* bearingTo5Urbahns;

//On time & Off time for display 24 hr clock
int OnTime = 6;
int OffTime = 21;

Adafruit_LiquidCrystal lcd(RS, EN, DB4, DB5, DB6, DB7);
const int numRows = 4;
const int numCols = 20;

//Create an array for each row of lcd 0 - 3 chars 0-20
char lcdRow0[20];
char lcdRow1[20];
char lcdRow2[20];
char lcdRow3[20];

//---------------------------------------------------------------

void setup()
{
    Wire.begin();
    Serial.begin(115200);   // open serial over USB at 115200 baud
    ss.begin(GPSBaud);      // software serial gps baud rate 9600
    
    pinMode(power, OUTPUT); //Sets analog A3 as digital output
    pinMode(GND, OUTPUT);   //Sets analog A2 as digital output
        
    digitalWrite(power, HIGH);  //Puts A3 at 5V
    digitalWrite(GND, LOW);     //Puts A2 to Grd
        
    startMillis = millis();
    
    //Initialize the analog pins used for time zone adjustments
    pinMode(A0, INPUT_PULLUP);  //Monitored pin connected to button
    pinMode(A1, OUTPUT);        //Connected to led

    
    //Initialize the Dallas I2C sensors and ping them
    sensor.begin();
    myPressure.begin();
    //Configure pressure sensor
    myPressure.setModeBarometer();    // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(7);  // Set Oversample to the recommended 128
    myPressure.enableEventFlags();    // Enable all three pressure and temp event flag 
    
    //Set up LCD display & create special characters
    pinMode(backlightPin, OUTPUT);
    analogWrite(backlightPin, 255);   //Turn backlight to max  
    lcd.begin(numCols, numRows);      
    lcd.createChar(1, house);
    lcd.createChar(2, tree);
    lcd.createChar(3, latitude);
    lcd.createChar(4, longtitude);
    lcd.createChar(5, satelites);
    lcd.createChar(6, upArrow);
    lcd.createChar(7, rightArrow);
    lcd.home();
    lcd.print(" G'Day! Setting Up");
}

//------------------------------------------------------------------
//  NB In PlatformIO it is necessary to place functions before loop() in order that they are defined.
//  If placed after will get undefined function error.
//---------------------------------------------------------------

void getWeather()
{
  // Measure Relative Humidity from the Si7021
  humidity = sensor.getRH();
  // Measure Temperature from the Si7021
  tempf = sensor.getTempF();
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
  //Convert to centigrade
  tempc = ((tempf-32)/1.8);
  //Get temp from DS18B20
  DS18B20.requestTemperatures();
  externaltempc = DS18B20.getTempCByIndex(0);
  hpa = (myPressure.readPressure()/100);
}
 
//---------------------------------------------------------------

void DigitalToString (float f_val, int Mylength, int Mydecimals)
{
  //sprintf does not display strings (%f) on arduino
  //So this function converts floats to strings to then use sprintf %s
  //NB %s expects char* not str so need to use string.c_str() in sprintf function.
  dtostrf(f_val,Mylength,Mydecimals,dToString);
}

//-----------------------------------------------------------

void gps_data()
{ 
  //Get time data utc TIME
  if (gps.time.isUpdated())
  {
  hr = (gps.time.hour());
  min = (gps.time.minute());
  sec = (gps.time.second());
  }  
  
  //Get date data 
  if (gps.date.isUpdated())
  { 
  gpsYear = (gps.date.year());
  gpsMonth = (gps.date.month());
  gpsDay = (gps.date.day());
  }

  //Set current UTC time
  setTime(hr, min, sec, gpsDay, gpsMonth, gpsYear);
  //Add offset seconds TZ to get local time (WST = 8 * 3,600 secs = 28800)
  adjustTime(TZ);
  //Reset variables to local time via time library functions
  hr = hour();
  min = minute();
  sec = second();
  gpsYear = year();
  gpsMonth = month();
  gpsDay = day();

  //Adjust day to WST using modulo function 
  if (hr < 8)
    gpsDay = (gpsDay+1)%7;          //=Remainder of (day+1)/7
    
  //Routine to implement guassian algorithm for day of week
  int c,y,m,d;                      //century, year, month, day
  int cc,yy;            
  int dayofweek, monthofyear;    
      
  cc = gpsYear/100;                
  yy = gpsYear - ((gpsYear/100)*100); 
  c = (cc/4) - 2*cc-1; 
  y = 5*yy/4;
  m = 26*(gpsMonth+1)/10;
  d = gpsDay;
  dayofweek = (c+y+m+d)%7;        //Gives a value from 0 - 6
     
  WeekDay = DOW[dayofweek];       //Fetch day string from DOW array
  monthofyear = gpsMonth - 1;     //Adjust month as array starts at 0
  TheMonth = MOY[monthofyear];    //Fetch month string from array MOY

  if (gps.satellites.isUpdated())  Sats = (gps.satellites.value());
  if (gps.location.isUpdated())
  {
  Lat = (gps.location.lat());       
  Long = (gps.location.lng());
    
  //Convert Lat & Long to string via digitalToString()
  DigitalToString(Lat,7,3);
  LatStr = dToString;
  DigitalToString(Long,7,3);
  LongStr = dToString;
  }

 //Altitude 
 if (gps.altitude.isUpdated())  altitude = (gps.altitude.meters());

 //Distance & bearing routine
  distanceTo5Urbahns = 
  TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    Urbahns_Lat,
    Urbahns_Lon);
    distanceTo5Urbahns = (distanceTo5Urbahns/10);

  courseTo5Urbahns =
  TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    Urbahns_Lat, 
    Urbahns_Lon);
    bearingTo5Urbahns = (TinyGPSPlus::cardinal(courseTo5Urbahns));
    
  if (gps.charsProcessed() < 10)
    Serial.println(F("WARNING: No GPS data!!"));
}

//------------------------------------------------------------------
// This custom version of delay() ensures that the gps object
// is being "fed" while delay is on as standard delay() pauses computing.

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}

//----------------------------------------------------------------

void TZAdjust()
{

/*
Function to loop through 6 * 30 min adjustments on button press
Starts at 28800 and adds 1800 increments to 39600 (WST + 3hrs)
Then decrease from 39600 back to 28800 (WST)
There is a 5sec delay in running the void loop so will get 5 sec between actions
When led lights routine to increase decrease time is running
*/

//Monitor A0 which will give value between 0 (grd) & 1023 (5v)
//If not pressed TZButtonState will return 1023 as it is pulled high
//When button pressed A0 taken to ground value < 1023

  TZButtonState = analogRead(A0);
  if (TZButtonState == 1023) 
  {     
    //LED off no action required:
    digitalWrite(A1, LOW);
  }
   else 
  {
// < 1023 turn LED on and adjust time:
    digitalWrite(A1, HIGH);
//Start at 28800 Direction = 1 Easting : 0 = Westing 30m = 1,800s
    if ((TZDirection == 1))
    {
      TZ = (TZ + 1800);   //Increment time 30 min
    }
    if ((TZDirection == 0))
    {
      TZ = (TZ - 1800);   //Decrement time 30 min
    } 
    if ((TZ <= 28800)) (TZDirection = 1); //Move east +1800 second increments
    if ((TZ >= 39600)) (TZDirection = 0); //Move west -1800 second increments
  }
} 

//-----------------------------------------------------------------

void printInfo()
{
//This function prints the data out to the default Serial Port
  Serial.print(" Internal Temp:");
  Serial.print(tempc);
  Serial.print("C, ");
  Serial.print(" External Temp:");
  Serial.print(externaltempc);
  Serial.print("C, ");
  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("% ");
  Serial.print(hpa);
  Serial.println(" hpa");

  Serial.print(gpsDay);
  Serial.print("/");
  Serial.print(gpsMonth);
  Serial.print("/");
  Serial.println(gpsYear);

  Serial.print(hr);
  Serial.print(":");
  Serial.print(min);
  Serial.print(":");
  Serial.println(sec);

  Serial.print(WeekDay); 
  Serial.print ("-");
  Serial.print(TheMonth);
  Serial.print ("-");
  Serial.println(gpsDay);

  Serial.print("Satelites ");
  Serial.print(Sats);
  Serial.print(" C-ords : ");
  Serial.print(Lat, 6);
  Serial.print(":");
  Serial.println(Long, 6);
  Serial.print("Strings ");
  Serial.print(LatStr);
  Serial.print("/");
  Serial.println(LongStr);

  Serial.print("Alt ");
  Serial.print(altitude);
  Serial.print("m  DtP ");
  Serial.print(distanceTo5Urbahns);
  Serial.print("km ");
  Serial.print(" Bearing ");
  Serial.println(bearingTo5Urbahns);
  Serial.println("");

  Serial.print("Timezone ");
  Serial.print(" Direction = ");
  Serial.print(TZDirection);
  Serial.print(" Timezone seconds = ");
  Serial.println(TZ);

}

//---------------------------------------------------------------

void printToLCD()
{
//Print data to 20*4 LCD via lcdRow arrays 0-3
//%f floats not supported so convert float to string if decimal places required
//s% takes a char* not a std::string. Thus use string.c_str() which gives const char* to the contents of a std::string
//%.2d formats to display 2 numbers with 0 padding if num<2
sprintf(lcdRow0, " %s-%s-%.2d %.2d:%.2d:%.2d", WeekDay.c_str(), TheMonth.c_str(), gpsDay, hr, min, sec);
sprintf(lcdRow1, "%c%.2d %c%.2d H%d%% P%.4d", 1, tempc, 2, externaltempc, humidity,hpa); //1 & 2 are defined chars house & tree
sprintf(lcdRow2, "%c%d %c%7s %c%7s", 5, Sats, 3, LatStr.c_str(), 4, LongStr.c_str()); 
sprintf(lcdRow3, "m%c%-4d H%.4dkm %c%-3s", 6,altitude, distanceTo5Urbahns, 7, bearingTo5Urbahns);

lcd.setCursor(0,0);
lcd.print(lcdRow0);
lcd.setCursor(0,1);
lcd.print(lcdRow1);
lcd.setCursor(0,2);
lcd.print(lcdRow2); 
lcd.setCursor(0,3);
lcd.print(lcdRow3);

} 
  
//---------------------------------------------------------------

void loop()
{
  smartDelay(5000); //5 sec gap between readings
  gps_data();
  TZAdjust();
  getWeather();
  printInfo();
  
  //Turn on display between OnTime & OffTime
  if ((hr >= OnTime) && (hr < OffTime))   
  {
    lcd.display();
    analogWrite(backlightPin, 255); //Turn backlight to max    
    printToLCD();
  }
  else
  {
    lcd.noDisplay();               //Turn off display
    analogWrite(backlightPin, 5);  //Dim backlight
  }
}

