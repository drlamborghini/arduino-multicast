// 
// ArduinoMulticast
// 
// A project to read environmental sensors and system conditions 
// which then multicasts information as ASCII text
// 
// Intended for use with the SparkFun ESP32 Thing Plus C and the SparkFun BME280 environmential sensor 
// connected over I2C (Qwicc).
// 
// Also uses ESP32 ultra low power mode to extend battery life
// 
// The first version included an http server for testing
//  

#include <WebServer.h>
#include <Uri.h>
#include <HTTP_Method.h>

#include <WiFiScan.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiType.h>
#include <WiFiSTA.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFi.h>

// for sntp
#include "time.h"
#include "sntp.h"


// 
// local wifi credentials here:
//
const char* ssid = "ninemile";    // Enter SSID here
const char* password = "southwest15"; // Enter Password here

// Adjust the local Reference Pressure
// Nathan Seidle @ SparkFun Electronics
// March 23, 2018
// Feel like supporting our work? Buy a board from SparkFun!
// https://www.sparkfun.com/products/14348 - Qwiic Combo Board
// https://www.sparkfun.com/products/13676 - BME280 Breakout Board
// 'Sea level' pressure changes with high and low pressure weather movement. 
// This sketch demonstrates how to change sea level 101325Pa to a different value.
// See Issue 1: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/issues/1
// Google 'sea level pressure map' for more information:
// http://weather.unisys.com/surface/sfc_con.php?image=pr&inv=0&t=cur
// https://www.atmos.illinois.edu/weather/tree/viewer.pl?launch/sfcslp
// 29.92 inHg = 1.0 atm = 101325 Pa = 1013.25 mb


#include <Wire.h>
#include "SparkFunBME280.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> 

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Allow access to all the 17048 features
BME280 mySensor;

int setup_bme_sensor()
{
  Serial.begin(115200);
  Serial.println("Example showing alternate I2C addresses");

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  // Set up the MAX17048 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
  }

  if (mySensor.beginI2C() == false)    //Begin communication over I2C
  {
      Serial.println("The sensor did not respond. Please check wiring.");
      return -1;
//      while (1); //Freeze
  }

  mySensor.setReferencePressure(101200); //Adjust the sea level pressure used for altitude calculations

  return 0;
}

void print_battery_data()
{
  // Print the variables:
  Serial.print("Voltage: ");
  Serial.print(lipo.getVoltage(), 2);  // Print the battery voltage
  Serial.print("V");

  Serial.print(" Percentage: ");
  Serial.print(lipo.getSOC(), 2); // Print the battery state of charge with 2 decimal places
  Serial.print("%");

  Serial.print(" Change Rate: ");
  Serial.print(lipo.getChangeRate(), 2); // Print the battery change rate with 2 decimal places
  Serial.print("%/hr");

  Serial.println();
}

void print_sensor_data()
{
  Serial.print("--------------------------\n");
  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");

  float pressure = mySensor.readFloatPressure() / 3386.39; 
//  printf("pressure %1.2f", pressure);
  Serial.print(pressure, 2);

#if 0
  Serial.print(" Locally Adjusted Altitude: ");
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeFeet(), 1);
#endif

  Serial.print(" Temp: ");
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor.readTempF(), 2);

  Serial.println();

  delay(50);
}

#if 0
// Now, let us define the response (HTML Page) that will be sent back to the device/user which sent the request. The function handles the server that has been started and controls all the endpoint functions when receiving a request.
// In here we can see, the server will send a response code '200' with content as 'text/html' and finally the main HTML text. Below is the way HTML text is responded -
String SendHTML()
{
    String ptr = "<!DOCTYPE html> <html>\n";
    ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
    ptr +="<title>ESP32 Hello World</title>\n";
    ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
    ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
    ptr +="p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
    ptr +="</style>\n";
    ptr +="</head>\n";
    ptr +="<body>\n";
    ptr +="<div id=\"webpage\">\n";
    ptr +="<h1>Hello World, ";

    String countString = "";
    countString =+ count;
    ptr += "count = \n";
    ptr += countString;
    ptr += "</h1>\n";
    printf("countString %s\n", countString.c_str() );


    String temperatureString = "";
    temperatureString =+ mySensor.readTempF();
    ptr += "<h1>Conditions: \n";
    ptr += temperatureString;
    ptr += " deg F, ";
//    ptr += "</h1>\n";

    String pressureString = "";
    pressureString =+ mySensor.readFloatPressure() / 3386.39;
//    ptr += "<h1>pressure = \n";
    ptr += pressureString;
    ptr += " inHg, ";
 //   ptr += "</h1>\n";

    String humidityString = "";
    humidityString =+ mySensor.readFloatHumidity();
//    ptr += "<h1>humidity = \n";
    ptr += humidityString;
    ptr += " % humidity";
    ptr += "</h1>\n";

#if 1
    String batteryVoltageString = "";
    batteryVoltageString =+ lipo.getVoltage();
    ptr += "<h1>Battery  = \n";
    ptr += batteryVoltageString;
    ptr += " V, ";
    
    String batteryPercentString = "";
    batteryPercentString =+ lipo.getSOC();
    ptr += batteryPercentString;
    ptr += "% ";
    
    ptr += "</h1>\n";
#endif

    ptr +="</div>\n";
    ptr +="</body>\n";
    ptr +="</html>\n";
    return ptr;    
}
#endif

//
// timer wakeup configuration
//


// Simple Deep Sleep with Timer Wake Up
// =====================================
// ESP32 offers a deep sleep mode for effective power
// saving as power is an important factor for IoT
// applications. In this mode CPUs, most of the RAM,
// and all the digital peripherals which are clocked
// from APB_CLK are powered off. The only parts of
// the chip which can still be powered on are:
// RTC controller, RTC peripherals ,and RTC memories

// This code displays the most basic deep sleep with
// a timer to wake it up and how to store data in
// RTC memory to use it over reboots


#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  10            // Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;


// Method to print the reason by which ESP32
// has been awaken from sleep
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
}

void wakeup_timer_setup()
{
    //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    //  First we configure the wake up source
    //  We set our ESP32 to wake up every 5 seconds

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    // Next we decide what all peripherals to shut down/keep on
    // By default, ESP32 will automatically power down the peripherals
    // not needed by the wakeup source, but if you want to be a poweruser
    // this is for you. Read in detail at the API docs
    // http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    // Left the line commented as an example of how to configure peripherals.
    // The line below turns off all RTC peripherals in deep sleep.

    //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //Serial.println("Configured all RTC Peripherals to be powered down in sleep");
}

void go_to_sleep()
{
    // Now that we have setup a wake cause and if needed setup the
    // peripherals state in deep sleep, we can now start going to
    // deep sleep.
    // In the case that no wake up sources were provided but deep
    // sleep was started, it will sleep forever unless hardware
    // reset occurs.

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    Serial.println("Going to sleep now");
    Serial.flush(); 

#if 0
    esp_deep_sleep_start();
#else
    esp_light_sleep_start();
#endif
}


//
// WiFi and UDP setup for ESP32 
//

#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "ninemile";
const char * networkPswd = "southwest15";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "10.0.0.255";
const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

//
// form and send a packet
//
void send_UDP()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo))
    {
      Serial.println("No time available (yet)");
      return;
    }
  
#if 0
  struct tm
{
  int tm_sec;			/* Seconds.	[0-60] (1 leap second) */
  int tm_min;			/* Minutes.	[0-59] */
  int tm_hour;			/* Hours.	[0-23] */
  int tm_mday;			/* Day.		[1-31] */
  int tm_mon;			/* Month.	[0-11] */
  int tm_year;			/* Year	- 1900.  */
  int tm_wday;			/* Day of week.	[0-6] */
  int tm_yday;			/* Days in year.[0-365]	*/
  int tm_isdst;			/* DST.		[-1/0/1]*/
#endif

    if(connected)
    {
        udp.beginPacket(udpAddress,udpPort);
        udp.printf("Time: %d:%.2d:%.2d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        udp.printf("Seconds since boot: %lu\n", millis()/1000);
        udp.printf("Temperature: %1.2f deg F\n", mySensor.readTempF());
        udp.printf("Pressure: %1.2f inHg\n", mySensor.readFloatPressure() / 3386.39);
        udp.printf("Humidity: %1.2f\n", mySensor.readFloatHumidity());
        udp.printf("Voltage: %1.2f Vdc\n", lipo.getVoltage());
        udp.printf("Charge: %1.2f percent\n", lipo.getSOC());
        udp.printf("Rate: %1.2f percent/hour\n", lipo.getChangeRate());
        udp.endPacket();
    }
}

//
// sntp setup
//

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = (-5 * 3600);
const int   daylightOffset_sec = 3600;

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}



//
// Setup the BME280 and Wifi connections
//
void setup() 
{
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    // Set the terminal baud rate
    Serial.begin(115200);
    delay(100);

#if 1
  // set notification call-back function
  sntp_set_time_sync_notification_cb( timeavailable );

  /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  sntp_servermode_dhcp(1);    // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
   * A more convenient approach to handle TimeZones with daylightOffset 
   * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
   * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   */
  //configTzTime(time_zone, ntpServer1, ntpServer2);
#endif // if 0



    //Connect to the WiFi network
    connectToWiFi(networkName, networkPswd);
    Serial.println("Connecting to ");
    Serial.println(ssid);

    Serial.println("");
    Serial.println("WiFi connected..!");
    Serial.print("Got IP: ");
    Serial.println(WiFi.localIP());

    // ready the BME280
    setup_bme_sensor();
}

//  
// Use the on board LEDs as health indicators
// Multicast the environment and system info
// Put the system to sleep to extend battery life
//
void loop() 
{

#if 1
  // blink the LED as a health indicator
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(3000);                       
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
#else
  // blink the LED as a health indicator
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  print_sensor_data();
  print_battery_data();
  printLocalTime();     // it will take some time to sync time :)

  send_UDP();

  delay(1000);                       // wait for a second
  
  go_to_sleep();
}
