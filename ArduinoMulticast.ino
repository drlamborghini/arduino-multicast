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

// for BME280
#include <Wire.h>
#include "SparkFunBME280.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> 

// for sntp
#include "time.h"
#include "sntp.h"

// WiFi network name and password:
const char * networkSSID = "ninemile";
const char * networkPswd = "southwest15";

//IP address to send UDP data to, a network broadcast address
const char * udpAddress = "10.0.0.255";
const int udpPort = 3333;

//Are we currently connected?
boolean g_connected = false;
int g_connectCount = 0;
#define MAX_CONNECT_RETRIES 5

//The udp library class
WiFiUDP udp;

//sntp
struct tm g_timeInfo;
bool g_sntpSuccess = false;
RTC_DATA_ATTR bool nv_rtcTimeIsSet;

// RTC
#include <ESP32Time.h>
ESP32Time rtc(3600);  // offset in seconds GMT+1

// battery sensing
SFE_MAX1704X lipo(MAX1704X_MAX17048); // Allow access to all the 17048 features

// weather measurement
BME280 mySensor;

// lights
#define HOUR_SUN_RISE   6
#define HOUR_SUN_SET    19
#define IS_DAY_TIME(hour) ( (hour > HOUR_SUN_RISE) && (hour < HOUR_SUN_SET) )

#define VOLTAGE_LIGHTS_OFF 3.30
#define VOLTAGE_LOW_LIMIT 3.20

#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor for micro seconds to seconds 
#define SECONDS_PER_MINUTE 60
#define MILLISECONDS_PER_SECOND 1000
#define TIME_TO_SLEEP_MINS 5
#define TIME_TO_SLEEP_SECS (TIME_TO_SLEEP_MINS * SECONDS_PER_MINUTE)
#define HALF_HOUR_BOOT_COUNTS (30 / TIME_TO_SLEEP_MINS)
#define ONE_HOUR_BOOT_COUNTS (60 / TIME_TO_SLEEP_MINS)
#define LIGHTS_OFF_BOOT_MAX HALF_HOUR_BOOT_COUNTS

bool g_lightsAreOn = false;

// connections
#define CONNECTION_DELAY_MILLISECONDS 5000

RTC_DATA_ATTR int bootCount = 0;

// Adjust the local Reference Pressure
// 29.92 inHg = 1.0 atm = 101325 Pa = 1013.25 mb
int setup_bme_sensor()
{
  Serial.begin(115200);
 
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

  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");

  float pressure = mySensor.readFloatPressure() / 3386.39; 
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

// Simple Deep Sleep with Timer Wake Up
// =====================================
// In this mode CPUs, most of the RAM,
// and all the digital peripherals which are clocked
// from APB_CLK are powered off. The only parts of
// the chip which can still be powered on are:
// RTC controller, RTC peripherals ,and RTC memories

// This code displays the most basic deep sleep with
// a timer to wake it up and how to store data in
// RTC memory to use it over reboots

// Method to print the reason by which ESP32 has been awaken from sleep
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

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_SECS * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP_SECS) + " Seconds");

    // By default, ESP32 will automatically power down the peripherals
    // not needed by the wakeup source, but if you want to be a poweruser
    // this is for you. Read in detail at the API docs
    // http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    // Left the line commented as an example of how to configure peripherals.
    // The line below turns off all RTC peripherals in deep sleep.

    //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //Serial.println("Configured all RTC Peripherals to be powered down in sleep");
}

void go_to_light_sleep()
{
    // Now that we have setup a wake cause and if needed setup the
    // peripherals state in deep sleep, we can now start going to
    // deep sleep.
    // In the case that no wake up sources were provided but deep
    // sleep was started, it will sleep forever unless hardware
    // reset occurs.

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_SECS * uS_TO_S_FACTOR);
    Serial.println("Going to light sleep for " + String(TIME_TO_SLEEP_SECS) + " Seconds");
    Serial.flush(); 

    esp_light_sleep_start();
}

void go_to_deep_sleep(int timeToSleepMins)
{
    // Now that we have setup a wake cause and if needed setup the
    // peripherals state in deep sleep, we can now start going to
    // deep sleep.
    // In the case that no wake up sources were provided but deep
    // sleep was started, it will sleep forever unless hardware
    // reset occurs.
    
    int timeToSleepSecs = timeToSleepMins * SECONDS_PER_MINUTE;
    esp_sleep_enable_timer_wakeup(timeToSleepSecs * uS_TO_S_FACTOR);
    Serial.println("Going to deep sleep for " + String(TIME_TO_SLEEP_SECS) + " Seconds");
    Serial.flush(); 

    esp_deep_sleep_start();
}

void connectToWiFi(const char * ssidName, const char * pwd)
{
    Serial.println("Connecting to WiFi network: " + String(ssidName));

    // delete old config
    WiFi.disconnect(true);
    //register event handler
    WiFi.onEvent(wifi_event);
    
    //Initiate connection
    WiFi.begin(ssidName, pwd);

    Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void wifi_event(WiFiEvent_t event)
{
    switch(event) 
    {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          g_connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          g_connected = false;
          break;
      default: break;
    }
}

//
// form and send a packet
//
void send_UDP()
{
    printf("send_UDP\n");
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("%2.2d:%.2d:%.2d\t", g_timeInfo.tm_hour, g_timeInfo.tm_min, g_timeInfo.tm_sec);
    udp.printf("%1.2f\t", mySensor.readTempF());
    udp.printf("%1.2f\t", mySensor.readFloatPressure() / 3386.39);
    udp.printf("%1.2f\t", mySensor.readFloatHumidity());
    udp.printf("%s\t", g_lightsAreOn ? "ON" : "OFF");
    udp.printf("%1.2f\t", lipo.getVoltage());
    udp.printf("%1.2f\t", lipo.getSOC());
    udp.printf("%1.2f\t", lipo.getChangeRate());
    udp.endPacket();
}

//
// sntp setup
//
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = (-5 * 3600);
const int   daylightOffset_sec = 3600;
const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

void print_local_time()
{
    Serial.println(&g_timeInfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void time_available(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP!");
    getLocalTime(&g_timeInfo);
    Serial.println(&g_timeInfo, "time_available: %A, %B %d %Y %H:%M:%S");

//    rtc.setTime(g_timeInfo.tm_sec, g_timeInfo.tm_min, g_timeInfo.tm_hour, g_timeInfo.tm_mday, g_timeInfo.tm_mon, g_timeInfo.tm_year);  

    g_sntpSuccess = true;
}

void setup_SNTP()
{
    printf("Setting up SNTP\n");

    // set notification call-back function
    sntp_set_time_sync_notification_cb( time_available );

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

    // check to see if the time has bee set previously
    // g_timeIsSet  (g_timeIsSet == true) &&
    struct tm rtcTimeInfo = rtc.getTimeStruct();
    Serial.println(&rtcTimeInfo, "setup rtcTimeInfo: %A, %B %d %Y %H:%M:%S");

#if 0
        rtcTimeInfo.tm_sec = 0
        rtcTimeInfo.tm_min = 0;
        rtcTimeInfo.tm_hour = 0;
        rtcTimeInfo.tm_mday = 0;
        rtcTimeInfo.tm_mon = 0;
        rtcTimeInfo.tm_year = 0;
        rtcTimeInfo.tm_wday = 0;
        rtcTimeInfo.tm_yday = 0;
        rtcTimeInfo.tm_isdst = 0;
        rtc.setTime(0, 0, 0, 0, 0, 0);  

#endif        

#if 0
//    rtc.setTime(30, 24, 15, 17, 1, 2023);  // 17th Jan 2021 15:24:30
//nv_rtcTimeIsSet = false;

    if( (rtcTimeInfo.tm_year > 122) &&
        (rtcTimeInfo.tm_yday > 49) )
    {
        g_timeInfo.tm_sec = rtcTimeInfo.tm_sec;
        g_timeInfo.tm_min = rtcTimeInfo.tm_min;
        g_timeInfo.tm_hour = rtcTimeInfo.tm_hour;
        g_timeInfo.tm_mday = rtcTimeInfo.tm_mday;
        g_timeInfo.tm_mon = rtcTimeInfo.tm_mon;
        g_timeInfo.tm_year = rtcTimeInfo.tm_year;
        g_timeInfo.tm_wday = rtcTimeInfo.tm_wday;
        g_timeInfo.tm_yday = rtcTimeInfo.tm_yday;
        g_timeInfo.tm_isdst = rtcTimeInfo.tm_isdst;
        g_sntpSuccess = true;
        nv_rtcTimeIsSet = true;
        Serial.println(&g_timeInfo, "RTC time: %A, %B %d %Y %H:%M:%S");
    }
    else
    {
        nv_rtcTimeIsSet = false;
        setup_SNTP();
    }
#endif

    setup_SNTP();


    //Connect to the WiFi network
    connectToWiFi(networkSSID, networkPswd);
 
    // ready the BME280
    setup_bme_sensor();
}


//  
// Use the on board LEDs as health indicators
// Multicast the environment and system info
// Put the system to sleep to extend battery life
//
// 1. if not connected to wifi, wait and try again, if retries exceeded go to deep sleep
// 2. if connected, check SNTP data, if invalid wait and try again (todo: retries?)
// 3. if connected and SNTP data valid, read voltage
//      a. if voltage is below thresshold, go to deep sleep
//      b. if voltage is okay and time for lights ON, set the GPIO, go to light sleep
//      b, else go to deep sleep
void loop() 
{
    // flash the LED white momentarily to show start of the loop
    digitalWrite(RGB_BUILTIN, HIGH);   // Turn the RGB LED white
    delay(100);
    digitalWrite(RGB_BUILTIN, LOW);    // Turn the RGB LED off
    delay(100);
    neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue

    // if the charge drops to 0%, just deep sleep
    float batteryVoltageVdc = lipo.getVoltage();
    if(batteryVoltageVdc < VOLTAGE_LOW_LIMIT)
    {
        printf("\n === Zero Voltage Threshold: %1.2f ====\n", batteryVoltageVdc);
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
        bootCount = 0;
        go_to_deep_sleep(TIME_TO_SLEEP_MINS);
    }

    // during a light sleep, we may lose contact with Wifi, so reconnect
    if(!g_connected)
    {
        ++g_connectCount;
        printf("Not connected\n");
        connectToWiFi(networkSSID, networkPswd);
    }
    else
    {
        printf("Connected\n");
        g_connectCount = 0;
    }

    printf("CONNECTION_DELAY_MILLISECONDS %d\n", CONNECTION_DELAY_MILLISECONDS);
    delay(CONNECTION_DELAY_MILLISECONDS); 

    if(!getLocalTime(&g_timeInfo))
    {
        Serial.println("No time available (yet)");
        g_timeInfo.tm_min = 0;
        g_timeInfo.tm_sec = 0;
        g_timeInfo.tm_hour = 0;
    }
    else
    {
        Serial.println(&g_timeInfo, "Got local time: %A, %B %d %Y %H:%M:%S");
        //print_local_time();
    }

    printf("IS_DAY_TIME: %d [hour = %d,year = %d] g_sntpSuccess = %d\n", 
        IS_DAY_TIME(g_timeInfo.tm_hour), g_timeInfo.tm_hour, g_timeInfo.tm_year, g_sntpSuccess);    
    
    // connection is made, sntp time is valid, set the lights according to time and charge
    if( (g_sntpSuccess)  && (g_timeInfo.tm_year > 122) )
    {
#if 0        
        // do we need to set the RTC from SNTP?
        if(nv_rtcTimeIsSet == false)
        {
            nv_rtcTimeIsSet = true;
            rtc.setTime(g_timeInfo.tm_sec, g_timeInfo.tm_min, g_timeInfo.tm_hour, g_timeInfo.tm_mday, g_timeInfo.tm_mon, g_timeInfo.tm_year); 

  //void ESP32Time::setTime(int sc, int mn, int hr, int dy, int mt, int yr, int ms) {
  // seconds, minute, hour, day, month, year $ microseconds(optional)
  // ie setTime(20, 34, 8, 1, 4, 2021) = 8:34:20 1/4/2021
//            rtc.setTime(30, 24, 15, 17, 1, 2021);  // 17th Jan 2021 15:24:30
            Serial.println(&g_timeInfo, "Setting RTC time: %A, %B %d %Y %H:%M:%S");
        }
#endif
        // Battery is okay, check the time.
        if(!IS_DAY_TIME(g_timeInfo.tm_hour))
        {
            // if the charge drops below 25%, Lights out and deep sleep
            batteryVoltageVdc = lipo.getVoltage();
            if(batteryVoltageVdc < VOLTAGE_LIGHTS_OFF)
            {
                if(++bootCount > LIGHTS_OFF_BOOT_MAX)     // filter out edge noise with multiple samples
                {
                    digitalWrite(LED_BUILTIN, LOW);    //turn the LED off
                    neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
                    g_lightsAreOn = false;
                }
            }
            else // battery is above the LIGHTS OUT threshold, so we can turn on the lights    
            {
                bootCount = 0;  
                digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
                printf("Night Time, Lights ON, light sleep\n");
                g_lightsAreOn = true;
                neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
            }
        }   
        else    // its day time
        {
            printf("Day time, Lights OFF, deep sleep\n");
            digitalWrite(LED_BUILTIN, LOW);    //turn the LED off
            neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
            g_lightsAreOn = false;
        }

        // always send data
        send_UDP();
        delay(100);

        // set the lights
        if(g_lightsAreOn)
            go_to_light_sleep();
        else
            go_to_deep_sleep(TIME_TO_SLEEP_MINS);
    }
    else
    {
        printf("Waiting for SNTP\n");
    }

    // bail on the connection after max retries
    // try again after a deep sleep
    if(g_connectCount >= MAX_CONNECT_RETRIES)    
    {
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
        neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
        printf("Connection retries exceeded [%d], Lights OFF, deep sleep\n", g_connectCount);
        g_lightsAreOn = false;
        bootCount = 0;
        go_to_deep_sleep(TIME_TO_SLEEP_MINS);
    } 

#if 0 //experimental
    rtc.setTime(30, 24, 15, 17, 1, 2021);  // 17th Jan 2021 15:24:30

//  Serial.println(rtc.getTime());          //  (String) 15:24:38
//  Serial.println(rtc.getDate());          //  (String) Sun, Jan 17 2021
//  Serial.println(rtc.getDate(true));      //  (String) Sunday, January 17 2021
//  Serial.println(rtc.getDateTime());      //  (String) Sun, Jan 17 2021 15:24:38
//  Serial.println(rtc.getDateTime(true));  //  (String) Sunday, January 17 2021 15:24:38
//  Serial.println(rtc.getTimeDate());      //  (String) 15:24:38 Sun, Jan 17 2021
//  Serial.println(rtc.getTimeDate(true));  //  (String) 15:24:38 Sunday, January 17 2021
//
//  Serial.println(rtc.getMicros());        //  (long)    723546
//  Serial.println(rtc.getMillis());        //  (long)    723
//  Serial.println(rtc.getEpoch());         //  (long)    1609459200
//  Serial.println(rtc.getSecond());        //  (int)     38    (0-59)
//  Serial.println(rtc.getMinute());        //  (int)     24    (0-59)
//  Serial.println(rtc.getHour());          //  (int)     3     (0-12)
//  Serial.println(rtc.getHour(true));      //  (int)     15    (0-23)
//  Serial.println(rtc.getAmPm());          //  (String)  pm
//  Serial.println(rtc.getAmPm(true));      //  (String)  PM
//  Serial.println(rtc.getDay());           //  (int)     17    (1-31)
//  Serial.println(rtc.getDayofWeek());     //  (int)     0     (0-6)
//  Serial.println(rtc.getDayofYear());     //  (int)     16    (0-365)
//  Serial.println(rtc.getMonth());         //  (int)     0     (0-11)
//  Serial.println(rtc.getYear());          //  (int)     2021

//  Serial.println(rtc.getLocalEpoch());         //  (long)    1609459200 epoch without offset
    Serial.println(rtc.getTime("RTC Time: %A, %B %d %Y %H:%M:%S"));   // (String) returns time with specified format 
    // formating options  http://www.cplusplus.com/reference/ctime/strftime/


    struct tm timeinfo = rtc.getTimeStruct();
    Serial.println(&timeinfo, "RTC time: %A, %B %d %Y %H:%M:%S");   //  (tm struct) Sunday, January 17 2021 07:24:38
  
    delay(100);

#endif // experimental    

}
