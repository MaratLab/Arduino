#include <Arduino.h>
#include <HardwareSerial.h>
#include "libs/dht.h"
#include "libs/dht.cpp"

#if defined(ARDUINO_AVR_MEGA2560)
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EthernetUdp.h>
#endif

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd(0x3f); // declare lcd object: auto locate & config exapander chip
// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
byte deg[8] = {
  B01111,
  B01001,
  B01001,
  B01111,
  B00000,
  B00000,
  B00000,
};

#if defined(ARDUINO_AVR_MEGA2560)
  // the media access control (ethernet hardware) address for the shield:
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
  EthernetServer server(80);

  unsigned int UDPlocalPort = 8888;    // local port to listen for UDP packets
  char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server
  const int NTP_PACKET_SIZE = 48;      // NTP time stamp is in the first 48 bytes of the message
  byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
  // A UDP instance to let us send and receive packets over UDP
  EthernetUDP Udp;
  #define TIME_RESYNC_TIMEOUT 10*24*60*60*1000 // 10 days in milliseconds
  #define TIME_SYNC_TIMEOUT   60*1000 // 1 minute in milliseconds
  unsigned long last_time_update_millis;
  boolean time_is_set = false;
#endif

dht DHT;
#define DHT22_PIN 6
#define MEASURE_TIMEOUT 5000
float previous_temperature = 0.0f;
float previous_humidity = 0.0f;
unsigned long measure_checkpoint = 0; 
int eth_connected;
int eth_hardware_present;


void setup() 
{
    Serial.begin(115200);
    Serial.println("Monitor startup...");
  
    #if defined(ARDUINO_AVR_MEGA2560)
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("No ETH hardware");
    }
    else
    {
      eth_hardware_present = 1;
      // start the Ethernet connection:
      eth_connected = Ethernet.begin(mac, 5000L); // with timeout 
      
      if (eth_connected == 1)
      {
        time_is_set = sync_time();
        startWebServer();
      }
    }
    #endif
    
    int lcd_status = lcd.begin(LCD_COLS, LCD_ROWS);
    if(lcd_status) // non zero status means it was unsuccesful
    {
      lcd_status = -lcd_status; // convert negative status value to positive number

      // hd44780 has a fatalError() routine that blinks an led if possible
      // begin() failed so call fatalError() with the error code.
      hd44780::fatalError(lcd_status); // does not return
    }
    // initalization was successful, the backlight should be on now

    // Print a message to the LCD
    if (eth_connected == 1) lcd.print("ETH UP|LCD OK");
    else lcd.print("ETH DOWN|LCD OK");

    delay(2000);

}


void loop() {
  measure();

  if (eth_hardware_present == 1) checkEthConnection();

  if (eth_connected == 1)
  {
    listenForClients();
  
    if ((time_is_set == false && (last_time_update_millis - millis()) > TIME_SYNC_TIMEOUT))
    {
      time_is_set = sync_time();
    } 
    else if (time_is_set == true  && (last_time_update_millis - millis()) > TIME_RESYNC_TIMEOUT)
    {
      sync_time();
    }
  }
  
}

void measure()
{
  unsigned long time = millis();
  if (time - measure_checkpoint > MEASURE_TIMEOUT) {
    measure_checkpoint = millis();

    // Clean output in putty
    //Serial.print("\33[2K\r");
    //Serial.print("\033[2A"); // move N lines up
    
    // READ DATA
    Serial.print("DHT22, \t");
    int chk = DHT.read22(DHT22_PIN);
    switch (chk)
    {
      case DHTLIB_OK:  
      Serial.print("OK,\t"); 
      break;
      case DHTLIB_ERROR_CHECKSUM: 
      Serial.print("Checksum error,\t"); 
      break;
      case DHTLIB_ERROR_TIMEOUT: 
      Serial.print("Time out error,\t"); 
      break;
      default: 
      Serial.print("Unknown error,\t"); 
      break;
    }
    // DISPLAY DATA
    Serial.print(DHT.humidity, 1);
    Serial.print("%,\t");
    Serial.print(DHT.temperature, 1);
    Serial.print("°C");
    if (previous_temperature < DHT.temperature)
    {
      Serial.print("\e[31m↑ \e[39m");
    }
    else if (previous_temperature > DHT.temperature)
    {
      Serial.print("\e[34m↓ \e[39m");
    }
    previous_temperature = DHT.temperature;
    previous_humidity = DHT.humidity;
    Serial.println(" ");

    int lcd_status = lcd.setCursor(0, 1);
    if(lcd_status) // non zero status means it was unsuccesful
    {
      lcd_status = -lcd_status; // convert negative status to positive number
      // setCursor() failed so call fatalError() with the error code.
      hd44780::fatalError(lcd_status); // does not return
      Serial.print("bad lcd status");
    }

    // print uptime on lcd device: (time since last reset)
    LcdPrintMeasurements(lcd, DHT.temperature, DHT.humidity);
  }
}


void checkEthConnection()
{
  #if defined(ARDUINO_AVR_MEGA2560)
  int eth_maintain = Ethernet.maintain();
  if (eth_maintain == 4)  // rebind success
  {
    Serial.println("Ethernet maintain: rebind success");
    eth_connected = 1;
    startWebServer();
  }
  else if (eth_maintain == 3) // rebind fail
  {
    eth_connected = 0;
  }
  #endif
}

#if defined(ARDUINO_AVR_MEGA2560)
void startWebServer()
{
  Serial.println("Starting web server...");  
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}


void listenForClients()
{
  #if defined(ARDUINO_AVR_MEGA2560)
  // listen for incoming clients
  EthernetClient client = server.available();
  
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          //client.println("Refresh: 10");  // refresh the page automatically every 10 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<meta charset=\"UTF-8\">");
          
          // output sensor value
          client.print("Humidity and temperature are: ");
          client.print(previous_humidity);
          client.print("% ");
          client.print(previous_temperature);
          client.println("°C<br />");

          
          client.println("<a href=\"http://192.168.30.250/script.js\">Script.js</a>");

          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
  #endif
}


boolean sync_time() {
  #if defined(ARDUINO_AVR_MEGA2560)
  Udp.begin(UDPlocalPort);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  }

  last_time_update_millis = millis();
  #endif
}

// send an NTP request to the time server at the given address
void sendNTPpacket(char* address) {
  #if defined(ARDUINO_AVR_MEGA2560)
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
  #endif
}


void LcdPrintMeasurements(Print &outdev, float temperature, float humidity)
{
  unsigned int temp, decimal_temp, hum_percent;
  float ftemp;

  temp = (unsigned int) temperature;
  decimal_temp = (unsigned int)(temperature * 10) - temp * 10;//(unsigned int) (modff(temperature, &ftemp));
  hum_percent = (unsigned int) humidity;

  lcd.createChar(0, deg);
    
  outdev.write('T');
  outdev.write(':');
  outdev.print((int)temp);
  outdev.write('.');
  outdev.print((int)(decimal_temp));
  lcd.write(byte(0));
  outdev.write('C');
  outdev.write(' ');
  outdev.write('H');
  outdev.write(':');
  outdev.print((int)hum_percent);
  outdev.write('%');
}
