// Geo Fencing project with alarm
// Author: Marco Schwartz
//
// Inspired by the code from Tony DiCola
//
// Released under a MIT license:
// https://opensource.org/licenses/MIT

// Libraries
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"

// LED & Buzzer pins
const int ledPin = 6;
const int buzzerPin = 9;

// Alarm
int counter = 0;
bool alarm = false;

// Size of the geo fence (in meters)
const float maxDistance = 100;

// Latitude & longitude for distance measurement
float initialLatitude;
float initialLongitude;
float latitude, longitude, speed_kph, heading, altitude;

// FONA pins configuration
#define FONA_RX              2   // FONA serial RX pin (pin 2 for shield).
#define FONA_TX              3   // FONA serial TX pin (pin 3 for shield).
#define FONA_RST             4   // FONA reset pin (pin 4 for shield)

// FONA instance & configuration
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);     // FONA software serial connection.
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);                 // FONA library connection.

void setup() {
  
  // Initialize serial output.
  Serial.begin(115200);
  Serial.println(F("Geofencing with Adafruit IO & FONA808"));

  // Set alarm components
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  alarm = false;

  // Initialize the FONA module
  Serial.println(F("Initializing FONA....(may take 10 seconds)"));
  fonaSS.begin(4800);

  if (!fona.begin(fonaSS)) {
    halt(F("Couldn't find FONA"));
  }

  fonaSS.println("AT+CMEE=2");
  Serial.println(F("FONA is OK"));

  // Use the watchdog to simplify retry logic and make things more robust.
  // Enable this after FONA is intialized because FONA init takes about 8-9 seconds.
  Watchdog.enable(8000);
  Watchdog.reset();

  // Enable GPS.
  fona.enableGPS(true);

  // Wait a little bit to stabilize the connection.
  Watchdog.reset();
  delay(3000);

  // Initial GPS read
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  initialLatitude = latitude;
  initialLongitude = longitude;

  // Init counter
  counter = millis();
  
}

void loop() {
  
  // Watchdog reset at start of loop--make sure everything below takes less than 8 seconds in normal operation!
  Watchdog.reset();

  // Grab a GPS reading.
  float latitude, longitude, speed_kph, heading, altitude;
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  Serial.print("Latitude: ");
  printFloat(latitude, 5);
  Serial.println("");

  Serial.print("Longitude: ");
  printFloat(longitude, 5);
  Serial.println("");

  Serial.print("Initial latitude: ");
  printFloat(initialLatitude, 5);
  Serial.println("");

  Serial.print("Initial longitude: ");
  printFloat(initialLongitude, 5);
  Serial.println("");

  // Calculate distance between new & old coordinates
  float distance = distanceCoordinates(latitude, longitude, initialLatitude, initialLongitude);
  
  Serial.print("Distance: ");
  printFloat(distance, 5);
  Serial.println("");

  // Set alarm on?
  if (distance > maxDistance) {
    alarm = true;
  }

  // Handle alarm
  if (alarm == false) {

    if (millis() - counter > 5000) {
      digitalWrite(ledPin, HIGH);
    }

     if (millis() - counter > 5100) {
      digitalWrite(ledPin, LOW);
      counter = millis();
    }

    noTone(buzzerPin);
    
  } 
  else {
    
    if (millis() - counter > 100) {
      digitalWrite(ledPin, HIGH);
    }

     if (millis() - counter > 200) {
      digitalWrite(ledPin, LOW);
      counter = millis();
    }
  
    tone(buzzerPin, 1000);
  }

}

// Halt function called when an error occurs.  Will print an error and stop execution while
// doing a fast blink of the LED.  If the watchdog is enabled it will reset after 8 seconds.
void halt(const __FlashStringHelper *error) {
  Serial.println(error);
  while (1) {
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
}

void printFloat(float value, int places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}

// Calculate distance between two points
float distanceCoordinates(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}
