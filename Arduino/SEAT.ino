
/* SEAT - Arduino Due Satellite Elevation & Azimuth Tracker

  Steven Kalmar
  KD8QWT
  pista01@gmail.com
  
  This is written for the Arduino Due only.  No it won't run on the Mega, Uno, etc...


  V0.1 - 12/18/2015  More of a collection of code fragments than actual working code
  V0.2 - 1/1/2016      First attempt at bringing it all together


*/
#include <config.h>
#include <DuePWM.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <Adafruit_GPS.h>
#include <rtc_clock.h>
#include <Rotator.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>

uint32_t timer = millis();
uint32_t displaytimer = millis();


// GPS power pin to Arduino Due 3.3V output.
// GPS ground pin to Arduino Due ground.
// For hardware serial 1 (recommended):
//   GPS TX to Arduino Due Serial1 RX pin 19
//   GPS RX to Arduino Due Serial1 TX pin 18
#define gpsSerial Serial1

Adafruit_GPS GPS(&gpsSerial);

Rotator ROT;
//Elevation_CTL EL;


// this keeps track of whether we're using the interrupt for gps
// off by default!
boolean usingInterrupt = false;
//void useInterrupt(boolean); 



RTC_clock rtc_clock(XTAL);
char* daynames[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

int pin = 13;
//volatile int state = LOW;
int prevstate = 1;

int potTotal = 0;
int curPot = 0;
int potCnt = 0;

int sensorValue = 0;
float voltage;

//float target_deg;

float EL_target_deg;

boolean positionsaved = true;  //Flag to let us know if the el/az position has been saved to the SD card

float EL_prev_target_deg = 0;
float AZ_absolute_pos;    //This is to allow a pass from more than zero to cross over zero, and to the 350s and below.
//The encoder state and current position will go negative, but this var will be the absolute AZ heading
//This is what is returned to the tracking app

int last_az, last_el;

//boolean is_tracking = false;//Are we actively tracking
//boolean is_parking = false; //Flag for when we are parking (blocking)
boolean EL_forward_dir = true;
boolean AZ_dir_change = false; //Assume we aren't starting out with a direction change
boolean EL_dir_change = false;
//float current_deg;
String admin_report;
unsigned long admin_report_interval;
unsigned long admin_report_last_time = 0;

unsigned long EZ_prev_set_time;
unsigned long last_cmd_time;

String caldata;

String rotatorstatus = "initializing";


//int motor_speed = 80;

int cnt = 0;

unsigned long now_time;
int start_ct = 8;

String commandStr = ""; //Commandstring where incoming commands are stored
String adminCommandStr = ""; //Incoming admin commands

//PID



byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
String localip = "192.168.1.223";
IPAddress ip(192, 168, 1, 223);
IPAddress gateway(192, 168, 1, 5);
IPAddress subnet(255, 255, 255, 0);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(4533);
boolean alreadyConnected = false; // whether or not the client was connected previously

EthernetServer adminserver(8888);
EthernetClient adminclient;
EthernetServer webserver(80);
EthernetClient webclient;

EthernetClient client;

String HTTPHeader;


void setup()
{
  Serial.begin(115200);
  
  //**GPS**********
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  Serial.print("Initializing SEAT");
  gpsSerial.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
  delay(1000);
  //**GPS**********

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);
  //disconnect the ethernet controller
  digitalWrite(10,HIGH);
  
  //SD card
  pinMode(4,OUTPUT); 
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }

  getLastPosition();
  getDOFCalibrationData();

  // disconnect the SD card
  digitalWrite(4, HIGH);
  digitalWrite(10,LOW);
  delay(500);

  //Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  // start listening for clients
  server.begin();
  adminserver.begin();
  webserver.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  ROT.init(last_az, last_el, caldata);
  
 
#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);
  
  
  
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__



void loop()
{
  now_time = millis();
  //prevstate = state;
  

  
  if (!ROT.is_rot_moving()) {
    
    readGPS();
  }

  //Serial.print("M1 current: ");
  //Serial.println(md.getM1CurrentMilliamps());
  ROT.CalculateNewPos();

  //delay(100);
  // wait for a new client:
  if (!adminclient.connected()){  //There can be only one
    adminclient = adminserver.available();
  }
  client = server.available();
  
  webclient = webserver.available();

  if (webclient) {
    boolean currentLineIsBlank = true;
    while (webclient.connected()) {
      if (webclient.available()) {   // client data available to read
        unsigned long web_time = millis();
        char c = webclient.read(); // read 1 byte (character) from client
        /*
        GET / HTTP/1.1
        Host: 192.168.1.223
        User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:46.0) Gecko/20100101 Firefox/46.0
        Accept: text/html,application/xhtml+xml,application/xml;q=0.9,**;q=0.8
        Accept-Language: en-US,en;q=0.5
        Accept-Encoding: gzip, deflate
        Connection: keep-alive


        GET /?AZ-Kp=22&AZ-Ki=0.01&AZ-Kd=0.05&EL-Kp=20&EL-Ki=0.01&EL-Kd=0.05&UpdatePID=Update+PID+Values HTTP/1.1
        Host: 192.168.1.223
        User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:46.0) Gecko/20100101 Firefox/46.0
        Accept: text/html,application/xhtml+xml,application/xml;q=0.9,**;q=0.8
        Accept-Language: en-US,en;q=0.5
        Accept-Encoding: gzip, deflate
        Referer: http://192.168.1.223/
        Connection: keep-alive
        Cache-Control: max-age=0

        */

         HTTPHeader = HTTPHeader + c;
        
        // last line of client request is blank and ends with \n
        // respond to client only after last line received
        if (c == '\n' && currentLineIsBlank) {
          Serial.println(HTTPHeader);
          
          int pos = HTTPHeader.indexOf('/');
          int pos2 = HTTPHeader.indexOf('HTTP');
          
          if (pos2 < 10){
            //nothing sent, so send the web page     
             webclient.println(getWebPage());
          } else {
            //Prob got some data
            if (HTTPHeader.indexOf('PID') > 0 ){
              processPIDWebRequest(HTTPHeader.substring(pos+2));
            } else if (HTTPHeader.indexOf('calibrate') > 0 ){
              if (ROT.calibrateToZero() == true){
                rotatorstatus = "calibrated";
              } else {
                rotatorstatus = "NOT calibrated";
              }
            }
            
            
            String url = "http://" + localip;
            Serial.println(url);
            String data = "<meta http-equiv=";
            data = data + '"' + "refresh" + '"' + " content=" + '"' + "0" + ';' + " url=" + url + '"' + " />";
            webclient.println(data);
            //webclient.println(getWebPage());
            
          }
          // send a standard http response header
          /*
          webclient.println("HTTP/1.1 200 OK");
          webclient.println("Content-Type: text/html");
          webclient.println("Connection: close");
          webclient.println();
          // send web page
          webclient.println("<!DOCTYPE html>");
          webclient.println("<html>");
          webclient.println("<head>");
          webclient.println("<title>SEAT - Satellite Elavation/Azimuth Tracker</title>");
          webclient.println("</head>");
          webclient.println("<body>");
          webclient.println("<h1>Hello from SEAT!</h1>");
          webclient.println("<p>Current Status</p>");
          webclient.println("<p>AZ Current position: " + String(ROT.get_az_current_deg()) + "</p>");
          webclient.println("<p>AZTarget position: " + String(ROT.get_az_target_deg()) + "</p>");
          webclient.println("<p>EL Current position: " + String(ROT.get_el_current_deg()) + "</p>");
          webclient.println("<p>EL Target position: " + String(ROT.get_el_target_deg()) + "</p>");
          webclient.println("<p>AZ degrees per second: " + String(ROT.getAZMultiplier()*1000) + "</p>");
          webclient.println("<p>EL degrees per second: " + String(ROT.getELMultiplier()*1000) + "</p>");
          webclient.println("<p>AZ Encoder count: " + String(ROT.getAZEncoderCount()) + "</p>");
          webclient.println("<p>EL Encoder count: " + String(ROT.getELEncoderCount()) + "</p>");
          webclient.println("<p>AZ PID: " + ROT.get_PID("AZ","start") + " - EL:" + ROT.get_PID("EL","start") + "</p>");
          webclient.println("<p>Location: ");
          webclient.println(GPS.latitude, 4); webclient.println(GPS.lat);
          webclient.println(", ");
          webclient.println(GPS.longitude, 4); webclient.println(GPS.lon);
          webclient.println("</p>");
          webclient.println("<p>Altitude: "); webclient.println(GPS.altitude);
          webclient.println("</p>");
          webclient.println("<p>Fix: "); webclient.println((int)GPS.fix);
          webclient.println("</p>");
          webclient.println("</p>Satellites: "); webclient.println((int)GPS.satellites);
          webclient.println("</p>");
          webclient.println("<p>At the tone, the time will be ");
          webclient.println(rtc_clock.get_hours());
          webclient.println(":");
          webclient.println(rtc_clock.get_minutes());
          webclient.println(":");
          webclient.println(rtc_clock.get_seconds());
          webclient.println(" ");
          webclient.println(daynames[rtc_clock.get_day_of_week() - 1]);
          webclient.println(": ");
          webclient.println(rtc_clock.get_days());
          webclient.println(".");
          webclient.println(rtc_clock.get_months());
          webclient.println(".");
          webclient.println(rtc_clock.get_years());
          webclient.println(" UTC</p>");
          webclient.println("<p>Page generated in " + String(millis() - web_time) + " ms</p>");

          webclient.println("</body>");
          webclient.println("</html>");
          */
          HTTPHeader = "";
          break;
        }
        // every line of text received from the client ends with \r\n
        if (c == '\n') {
          // last character on line of received text
          // starting new line with next character read
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // a text character was received from client
          currentLineIsBlank = false;
        }
      } // end if (client.available())

    } // end while (client.connected())
    delay(1);      // give the web browser time to receive the data
    webclient.stop(); // close the connection


  }

  if (adminclient.available()) {

    char c = adminclient.read();
    if (c == '\n') {
      Serial.println(adminCommandStr);

      int pos = adminCommandStr.indexOf(",");
      String cmd = adminCommandStr.substring(0, pos);
      Serial.println("Admin Command " + cmd);
      cmd.trim();
      Serial.println(cmd.length());

      if (cmd == "ST") {
        adminclient.println("H:" + String(rtc_clock.get_hours()) + " M:" + String(rtc_clock.get_minutes()) + " S:" + String(rtc_clock.get_seconds()) + "\n");
        adminclient.println("Current position: " + String(ROT.get_az_current_deg()) + " - Target: " + String(ROT.get_az_target_deg()) + "\n");
        adminclient.println("Location: ");
        adminclient.println(GPS.latitude, 4); adminclient.print(GPS.lat);
        adminclient.println(" ");
        adminclient.println(GPS.longitude, 4); adminclient.println(GPS.lon);
        adminclient.println("\n");
        adminclient.println("Satellites: "); adminclient.println((int)GPS.satellites);
        adminclient.println("\n");

      }
      
      if (cmd == "STOP"){
        admin_report = "";
        adminclient.println("OK\n");
      }
      
      if (cmd == "TRACKCSV") {
        admin_report=cmd;
        int first = adminCommandStr.indexOf(",");
        String vals = adminCommandStr.substring(first + 1);
        vals.trim();
        admin_report_interval = vals.toFloat();
        adminclient.println("OK\n");
        
      }

      if (cmd == "TUNEAZ" || cmd == "TUNEEL") {
        int first = adminCommandStr.indexOf(",");
        //Serial.println(first);
        String vals = adminCommandStr.substring(first + 1);
        //Serial.println(vals);
        first = vals.indexOf(",");
        //Serial.println(first);
        int last = vals.lastIndexOf(",");
        //Serial.println(last);
        String Kpstr = vals.substring(0, first);
        String Kistr = vals.substring(first + 1, last);
        String Kdstr = vals.substring(last + 1);
        adminclient.print("Kp:" + Kpstr + "\r\n");
        adminclient.print("Ki:" + Kistr + "\r\n");
        adminclient.print("Kd:" + Kdstr + "\r\n");
        Serial.println("Changing PID to ");
        Serial.println("Kp:" + Kpstr);
        Serial.println("Ki:" + Kistr);
        Serial.println("Kd:" + Kdstr + "\n");

        /*Kp = Kpstr.toFloat();
        Ki = Kistr.toFloat();
        Kd = Kdstr.toFloat();*/
        if (cmd == "TUNEAZ"){
          ROT.set_new_PID("AZ", Kpstr.toFloat(), Kistr.toFloat(), Kdstr.toFloat());
        }
        if (cmd == "TUNEEL"){
          ROT.set_new_PID("EL", Kpstr.toFloat(), Kistr.toFloat(), Kdstr.toFloat());
        }
        
        
        adminclient.print("Done!\r\n");
      }



      adminCommandStr = "";
    } else {
      adminCommandStr += c; //and adds them to the command String
    }
    
    //Process any long running replies
    

  }
  
  if (admin_report == "TRACKCSV"){
    if (admin_report_last_time + admin_report_interval < millis()){
      //Dump the data to the client
      Serial.println("Sending " + admin_report + " every " + String(admin_report_interval) + "ms");
      
      String data = String(rtc_clock.get_hours()) + ":" + String(rtc_clock.get_minutes()) + ":" + String(rtc_clock.get_seconds()) + "," + String(ROT.get_az_target_deg()) + "," + String(ROT.get_az_current_deg()) + "," + String(ROT.get_el_target_deg()) + "," + String(ROT.get_el_current_deg()) + "," + ROT.getAZDegPerSec() + "," + ROT.getELDegPerSec();
      Serial.println(data);
      //adminclient.print("\r\n");
      adminclient.print(data + "\r\n");
      admin_report_last_time = millis();
    }
    
  }



  if (client.available()) {

    //while (client.connected()) {

    char c = client.read();
    //Serial.println(c);
    //Serial.write(c);

    if (c == '\n') {
      Serial.println(commandStr);

      String cmd = commandStr.substring(0, 1);
      Serial.println("Command " + cmd);
      if (cmd == "p") {
        last_cmd_time = millis();
        //Serial.println("H:" + String(rtc_clock.get_hours()) + " M:" + String(rtc_clock.get_minutes()) + " S:" + String(rtc_clock.get_seconds()));
        String aztmpval = String(ROT.get_az_current_deg());
        String eltmpval = String(ROT.get_el_current_deg());
        
        /*
        if (ROT.get_az_current_deg() < 0) {
          //aztmpval = "-" + aztmpval;
          eltmpval = "-" + eltmpval;
        } else {
          //aztmpval = "+" + aztmpval;
          eltmpval = "+" + eltmpval;
        }*/

        client.print(String(ROT.get_az_absolute_pos()) + "\n" + String(ROT.get_el_current_deg()) + "\n" + "0\n");
        //Serial.println("p " + aztmpval + "+" + eltmpval + "\n");
      }

      if (cmd == "P") {
        //AZ_prev_set_time
        //AZ_prev_target_deg
        last_cmd_time = millis();
        Serial.println("H:" + String(rtc_clock.get_hours()) + " M:" + String(rtc_clock.get_minutes()) + " S:" + String(rtc_clock.get_seconds()));

        int pos = commandStr.lastIndexOf(' ');
        String azval = commandStr.substring(1, pos - 1);
        String elval = commandStr.substring(pos);
        //Serial.println("AZ: " + String(AZ_target_deg));
        //Serial.println("EL: " + String(EL_target_deg));
        azval.trim();
        elval.trim();
        
        //EL_target_deg = elval.toFloat();
        
        ROT.track(azval.toFloat(),elval.toFloat()); //Send the position to the Rotator
        client.print("0\n");
        
        ROT.processPosition();
        


      }

      
      commandStr = "";
    } else {
      commandStr += c; //and adds them to the command String
    }
    //}
  }




  
  //cnt++;
  //if (displaytimer > millis())  displaytimer = millis();

  // approximately every 1 second, print out the current stats
  if (millis() > displaytimer + 1000) {
    displaytimer = millis(); // reset the timer
    //Serial.print("Encoder count: ");
    //Serial.println(state);
    //Serial.println((state-prevstate)*60);
    //Serial.println((state-prevstate));

    Serial.print("Current AZ position: ");
    Serial.println(ROT.get_az_current_deg());
    Serial.print("Target AZ position: ");
    Serial.println(ROT.get_az_target_deg());
    
    Serial.print("Current EL position: ");
    Serial.println(ROT.get_el_current_deg());
    Serial.print("Target EL position: ");
    Serial.println(ROT.get_el_target_deg());
    Serial.print("Mag EL: ");
    Serial.println(ROT.getMagEL());
    Serial.print("Heading: ");
    Serial.println(ROT.getMagHeading());
    //Serial.print("\nSensor Offsets");
    //ROT.displaySensorOffsets();
    //Serial.print("\nCalibration Status:");
    //ROT.displayCalStatus();
    //Serial.print("\nSensor Status:");
    //ROT.displaySensorStatus();
        //Serial.print("Motor speed: ");
    //Serial.println(motor_speed);
    //Serial.print("Forward direction: ");
    //Serial.println(String(AZ_forward_dir));
    //if (AZ_forward_dir) {
    //  Serial.println(AZ.get_current_deg() - target_deg);
    //} else {
    //  Serial.println(target_deg - AZ.get_current_deg());
    //}
    //Serial.println("Output: " + String(Output) + "\n");

    //Serial.println(sensorValue);
    /*
    Serial.print("At the third stroke, it will be ");
    Serial.print(rtc_clock.get_hours());
    Serial.print(":");
    Serial.print(rtc_clock.get_minutes());
    Serial.print(":");
    Serial.println(rtc_clock.get_seconds());
    Serial.print(" ");
    Serial.print(daynames[rtc_clock.get_day_of_week()-1]);
    Serial.print(": ");
    Serial.print(rtc_clock.get_days());
    Serial.print(".");
    Serial.print(rtc_clock.get_months());
    Serial.print(".");
    Serial.println(rtc_clock.get_years());
    */
    displaytimer = millis();
  }


  
  ROT.processPosition();
  //If we are moving and position has been saved, then flag the position as not saved so later when we stop moving we can then save
  if (ROT.is_rot_moving() && positionsaved){
    positionsaved = false;
  }
  
  if (!positionsaved){
    if (!ROT.is_rot_moving()){
      //So we aren't moving and we need to save the current position
      saveAZELPosition();
    }
  }
  
  if (millis() - last_cmd_time > DISABLE_TRACKING_DELAY && last_cmd_time > 0) {
    Serial.println("Turning tracking off");
    //is_tracking = false;
    ROT.disable_tracking();
    //EL.disable_tracking();

    
    ROT.park();
    
    //EL.park();
    //is_parking = false;
    last_cmd_time = 0;
  }
}



void readGPS() {
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO)
    //if (c) Serial.print(c);
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 5 seconds or so, print out the current stats
  if (millis() - timer > 5000) {
    timer = millis(); // reset the timer

    /*Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);*/
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      rtc_clock.set_time(GPS.hour, GPS.minute, GPS.seconds);
      rtc_clock.set_date(GPS.day, GPS.month, GPS.year);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      /*
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      */
    }
    
  }

}

String getWebPage(){
  adafruit_bno055_offsets_t caldata = ROT.getSensorOffsets();
  String webdata = "<!doctype html>";
  webdata = webdata + "<html>";
  webdata = webdata + "<head>";
  webdata = webdata + "	<title>SEAT - Satellite Azimuth/Elavtion Tracker</title>";
  webdata = webdata + "</head>";
  webdata = webdata + "<body>";
  webdata = webdata + "<h1>SEAT Controller</h1>";
  //webdata = webdata + '';
  webdata = webdata + "<p>Last Status Message - " + rotatorstatus + "</p>";
  webdata = webdata + "<p>" + rtc_clock.get_hours() + ":" + rtc_clock.get_minutes() + ":" + rtc_clock.get_seconds() + "</p>";
  webdata = webdata + "<p>" + ROT.getCalStatus() + "</p>";
  webdata = webdata + "<p>Accelerometer: " + caldata.accel_offset_x + " " + caldata.accel_offset_y + " " + caldata.accel_offset_z + "</p>";
  webdata = webdata + "<p>Gyro: " + caldata.gyro_offset_x + " " + caldata.gyro_offset_y + " " + caldata.gyro_offset_z + "</p>";
  webdata = webdata + "<p>Mag: " + caldata.mag_offset_x + " " + caldata.mag_offset_y + " " + caldata.mag_offset_z + "</p>";
  webdata = webdata + "<p>Accel Radius: " + caldata.accel_radius + "</p>";
  webdata = webdata + "<p>Mag Radius: " + caldata.mag_radius + "</p>";
  //webdata = webdata + '';
  webdata = webdata + "<table border=" + '"' + "1" + '"' + " cellpadding=" + '"' + "1" + '"' + " cellspacing=" + '"' + "1" + '"' + "style=" + '"' + "width: 500px;" + '"' + ">";
  webdata = webdata + "	<tbody>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td style=" + '"' + "text-align: center;" + '"' + "><strong>Data Point</strong></td>";
  webdata = webdata + "			<td style=" + '"' + "text-align: center;" + '"' + "><strong>Azimuth</strong></td>";
  webdata = webdata + "			<td style=" + '"' + "text-align: center;" + '"' + "><strong>Elavation</strong></td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>Current Position</td>";
  webdata = webdata + "			<td>" + String(ROT.get_az_current_deg()) + "</td>";
  webdata = webdata + "			<td>" + String(ROT.get_el_current_deg()) + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>Target Position</td>";
  webdata = webdata + "			<td>" + String(ROT.get_az_target_deg()) + "</td>";
  webdata = webdata + "			<td>" + String(ROT.get_el_target_deg()) + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>Degrees/Second</td>";
  webdata = webdata + "			<td>" + ROT.getAZDegPerSec() + "</td>";
  webdata = webdata + "			<td>" + ROT.getELDegPerSec() + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>Encoder Position</td>";
  webdata = webdata + "			<td>" + String(ROT.getAZEncoderCount()) + "</td>";
  webdata = webdata + "			<td>" + String(ROT.getELEncoderCount()) + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>Motor Current Draw</td>";
  webdata = webdata + "			<td>" + ROT.getAZMotorCurrent() + "</td>";
  webdata = webdata + "			<td>" + ROT.getELMotorCurrent() + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "		<tr>";
  webdata = webdata + "			<td>PID</td>";
  webdata = webdata + "			<td>" + ROT.get_PID("AZ") + "</td>";
  webdata = webdata + "			<td>" + ROT.get_PID("EL") + "</td>";
  webdata = webdata + "		</tr>";
  webdata = webdata + "	</tbody>";
  webdata = webdata + "</table>";
  //webdata = webdata + '';
  
  String data = ROT.get_PID("AZ");
  int first = data.indexOf(",");
  String azp = data.substring(0, first);
  int second = data.indexOf(",",first+1);
  String azi = data.substring(first + 1, second);
  String azd = data.substring(second+1);
  
  data = ROT.get_PID("EL");
  first = data.indexOf(",");
  String elp = data.substring(0, first);
  second = data.indexOf(",",first+1);
  String eli = data.substring(first + 1, second);
  String eld = data.substring(second+1);
  //String data1 = 
  //Serial.println(vals);
  //first = vals.indexOf(",");
  
 
  
  webdata = webdata + "<form method=" + '"' + "get" + '"' + " name=" + '"' + "PID" + '"' + ">";
  
  webdata = webdata + "<p>PID Values</p>";
  webdata = webdata + "<p>Azimuth -> <input name=" + '"' + "AZ-Kp" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + azp + '"' + "/><input name=" + '"' + "AZ-Ki" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + azi + '"' + " /><input name=" + '"' + "AZ-Kd" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + azd + '"' + "/></p>";
  //webdata = webdata + '';
  webdata = webdata + "<p>Elavation -> <input name=" + '"' + "EL-Kp" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + elp + '"' + " /><input name=" + '"' + "EL-Ki" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + eli + '"' + " /><input name=" + '"' + "EL-Kd" + '"' + " type=" + '"' + "text" + '"' + " value=" + '"' + eld + '"' + " /></p>";
  //webdata = webdata + "<meta http-equiv=" + '"' + "refresh" + '"' + " content=" + '"' + "0" + ';' + " url=" + url + '"' + " />";
  //webdata = webdata + "<input type=" + '"' + "hidden" + '"' + "name=" + '"' + "redirect" + '"' + " value=" + '"' + url + '"' + ">";
  
  //webdata = webdata + '';
  webdata = webdata + "<p><input name=" + '"' + "UpdatePID" + '"' + " type=" + '"' + "submit" + '"' + " value="+ '"' + "Update PID Values" + '"' + " /></p>";
  //webdata = webdata + '';
  webdata = webdata + "<p>&nbsp;</p>";
  webdata = webdata + "</form>";
  webdata = webdata + "</body>";
  webdata = webdata + "</html>";
  
  return webdata;

}


void processPIDWebRequest(String data){
  Serial.println("Got Data!: " + data);
  //Got Data!: AZ-Kp=22&AZ-Ki=0.01&AZ-Kd=0.05&EL-Kp=20&EL-Ki=0.01&EL-Kd=0.05&UpdatePID=Update+PID+Values HTTP/1.1

  int pos1 = data.indexOf("&");
  int pos2 = data.indexOf("&", pos1+1);
  int pos3 = data.indexOf("&", pos2+1);
  int pos4 = data.indexOf("&", pos3+1);
  int pos5 = data.indexOf("&", pos4+1);
  int pos6 = data.indexOf("&", pos5+1);
  
  String AZKp = data.substring(0+6,pos1);
  String AZKi = data.substring(pos1+7,pos2);
  String AZKd = data.substring(pos2+7,pos3);
  String ELKp = data.substring(pos3+7,pos4);
  String ELKi = data.substring(pos4+7,pos5);
  String ELKd = data.substring(pos5+7,pos6);
  
  Serial.println("AZ-Kp:" + AZKp + ",AZ-Ki:" + AZKi + ",AZ-Kd:" + AZKd + ",EL-Kp:" + ELKp + ",EL-Ki:" + ELKi + ",EL-Kd:" + ELKd);
  ROT.set_new_PID("AZ",AZKp.toFloat(),AZKi.toFloat(),AZKd.toFloat());
  ROT.set_new_PID("EL",ELKp.toFloat(),ELKi.toFloat(),ELKd.toFloat());
  
}


void getLastPosition(void){
  File myFile;
  char c;
  String data;
  // open the file for reading:
  myFile = SD.open("lastpos.txt");
  if (myFile) {
    //Serial.println("lastpos.txt:");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      
      c = myFile.read();
      data = data + c;
      Serial.println(data);
      //Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening lastpos.txt");
    c = 0;
  }
  Serial.println(data);
  
  int pos = data.indexOf(",");
  last_az = data.substring(0,pos).toFloat();
  last_el = data.substring(pos+1).toFloat();
  
  
}

void getDOFCalibrationData(void){
  File myFile;
  char c;
  
  // open the file for reading:
  myFile = SD.open("caldata.txt");
  if (myFile) {
    //Serial.println("lastpos.txt:");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      
      c = myFile.read();
      caldata = caldata + c;
      //Serial.println(caldata);
      //Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening caldata.txt");
    c = 0;
  }
  Serial.println(caldata);
  
}

void saveAZELPosition(void){
  Serial.println("Saving position...");
  //We must shut down ethernet so we can write to the SD card and do the saving
  //pinMode(10, OUTPUT);
  //disconnect the ethernet controller
  digitalWrite(10,HIGH);
  digitalWrite(4,LOW);
  
  //SD card
  //pinMode(4,OUTPUT); 
  //if (!SD.begin(4)) {
  //  Serial.println("SD activation failed!");
  //  return;
  //}
  
  
  File myFile;
  String val;
  String data;
  // open the file for reading:
  val = String(ROT.getAZEncoderCount()) + "," + String(ROT.getELEncoderCount());
  
  myFile = SD.open("lastpos.txt", FILE_WRITE);
  Serial.println("Saving " + val);
  if (myFile) {
    //az,el
    myFile.println(val);
    
  }
  
  myFile.close();
  // disconnect the SD card
  digitalWrite(4, HIGH);
  digitalWrite(10,LOW);
  delay(500);
  
  Serial.println("Saved position");
  positionsaved = true; //Saved, so save no more, untill it's not
}




