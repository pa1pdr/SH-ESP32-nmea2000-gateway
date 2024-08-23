#include <Arduino.h>

#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// SDA and SCL pins on SH-ESP32
#define SDA_PIN GPIO_NUM_16
#define SCL_PIN GPIO_NUM_17

#define SOURCE_ADDRESS 0x0
#define ESP_BTNR GPIO_NUM_0

#define WGX1_PORT 60001


//#define HASDISPLAY


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <sensesp_app_builder.h>
#include "include/ActisenseASCIIReader.h"

using namespace sensesp;

ReactESP app;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels



WiFiServer server (WGX1_PORT);


String hostname = "WGX1-GW";


auto counterActi = new SKOutputInt ("sensorDevice." + hostname +  ".ActiSense_PGNs","",new SKMetadata (""));
auto counterN2K = new SKOutputInt ("sensorDevice." + hostname +  ".N2K_PGNs","",new SKMetadata (""));


tActisenseASCIIReader actisense_reader;

TwoWire *i2c;
Adafruit_SSD1306 *display;
bool show_display = true;

tNMEA2000 *nmea2000;

class WiFiClientStream : public Stream {
public:
    WiFiClientStream(WiFiClient* client) : client(client) {}

    size_t write(uint8_t data) override {
        debugV ("writing [%02x]",data);
        return client->write(data);
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        debugV ("Writing [%s]",buffer);
        return client->write(buffer, size);
    }

    int available() override {
        return client->available();
    }

    int read() override {
        if (!client->connected() || !client->available()) {
            return -1;
        }
        int c = client->read();
        debugV ("C = %d %c",c,(char)c);
        return c;
    }

    int peek() override {
        if (!client->connected() || !client->available()) {
            return -1;
        }
        return client->peek();
    }

    void flush() override {
        client->flush();
    }

private:
    WiFiClient* client;
};

WiFiClient client;
//WiFiClientStream* wifiStream;
WiFiClient* wifiStream;



void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

/**
 * @brief turn on the display
 * 
 */
void displayOn() {
   display->ssd1306_command(SSD1306_DISPLAYON);  //not recommended
   show_display = true;
   // display->clearDisplay();
   // display->display();
}

/**
 * @brief turn off the display & save some penguins
 * 
 */
void displayOff() {
   show_display = false;
   display->clearDisplay();
   display->display();
   display->ssd1306_command(SSD1306_DISPLAYOFF);  //not recommended
}


unsigned long num_n2k_messages = 0;
unsigned long num_actisense_messages = 0;

/**
 * @brief handles an incoming NMEA2000 message from the bus
 * 
 * @param message 
 */
void HandleStreamN2kMsg(const tN2kMsg &message) {
  num_n2k_messages++;
  ToggleLed();
  char buffer[MAX_STREAM_MSG_BUF_LEN];

  // Send it off
  if (client.connected()) {
      message.ForceSource (SOURCE_ADDRESS);   // alter the source to mine
      actisense_reader.buildMessage (message,buffer,MAX_STREAM_MSG_BUF_LEN);
      debugI ("Forwarding N2K msg PGN %d, in Actisense ASCII [%s]",message.PGN,buffer);
      wifiStream->println (buffer);
  } else {
    debugE ("Cannot forward N2K msg PGN %d, no connection",message.PGN);
  }
}

/**
 * @brief handel an incoming Actisense message
 * 
 * @param message 
 */
void HandleStreamActisenseMsg(const tN2kMsg &message) {

  num_actisense_messages++;
  ToggleLed();
  debugI ("Forwarding Actisense msg PGN %d to N2K bus",message.PGN);
  nmea2000->SendMsg(message);

  #ifdef DEBUG_ENABLED
    if (message.PGN == 129794) {   // AIS extended
       uint8_t MessageID;
       tN2kAISRepeat Repeat;
       uint32_t UserID;
			 uint32_t IMOnumber;
       char    Callsign[80];
       char    Name[80];
       uint8_t VesselType;
       double Length;
			 double Beam; 
       double PosRefStbd; 
       double PosRefBow;
       uint16_t ETAdate;
       double ETAtime;
			 double Draught;
       char   Destination[80];
       tN2kAISVersion AISversion;
       tN2kGNSStype GNSStype;
			 tN2kAISDTE DTE;
       tN2kAISTransceiverInformation AISinfo;
       uint8_t SID;
       
       ParseN2kAISClassAStatic(message, MessageID, Repeat, UserID,
				    IMOnumber, Callsign, 80, Name, 80, VesselType, Length,
				    Beam, PosRefStbd, PosRefBow, ETAdate, ETAtime,
				    Draught, Destination, 80, AISversion, GNSStype,
				    DTE, AISinfo, SID);
    
       debugE ("Parsed AIS info for IMO %d, [%s] callsign [%s]",IMOnumber,Name,Callsign);

    }
  #endif
  debugI ("Done");
}

String can_state;

void PollCANStatus() {
  // CAN controller registers are SJA1000 compatible.
  // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUNNING";
      break;
    case 1:
      can_state = "BUS-OFF";
      // try to automatically recover by rebooting
      app.onDelay(2000, []() {
        esp_task_wdt_init(1, true);
        esp_task_wdt_add(NULL);
        while (true)
          ;
      });
      break;
  }
}

void setup() {

#ifndef DEBUG_DISABLED
  SetupSerialDebug(115200);
//  Debug.begin(hostname,22,DEBUG_LEVEL);
#else
  Serial.begin(115200);
#endif
  delay(100);
  pinMode (ESP_BTNR,INPUT_PULLDOWN);


// Setup Sensesp connection: we need to be connected to the network
  SensESPAppBuilder builder;


  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname(hostname)
                    ->enable_ota("mypassword")
                  //  ->enable_wifi_signal_sensor()
                    ->enable_free_mem_sensor()
                  //  ->enable_ip_address_sensor()
                    ->set_button_pin(ESP_BTNR)  // reset
                    ->get_app();

 

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeat(1000, []() { ToggleLed(); });

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // input the NMEA 2000 messages

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20240331",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 NMEA 2000 WGX-1 GW",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2024-07-12)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );

  // Set device information

  nmea2000->SetDeviceInformation(
      2024331,  // Unique number. Use e.g. Serial number.
      137,       // Device function=NMEA 2000 Wireless Gateway. See codes on
            // https://canboat.github.io/canboat/canboat.html#indirect-lookup-DEVICE_FUNCTION
      25,  // Makes it an internetwork device
           // https://canboat.github.io/canboat/canboat.html#lookup-DEVICE_CLASS
      641,  // Diverse Yacht Services. Just choosen free from code list on
            // https://canboat.github.io/canboat/canboat.html#lookup-INDUSTRY_CODE
      4  // Industry Group: Marine
  );

  nmea2000->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear text
  
  
  nmea2000->EnableForward(false);  // dont forward 
  
  nmea2000->SetForwardOwnMessages(false);  // do not echo own messages.
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndSend);
  nmea2000->SetMsgHandler(HandleStreamN2kMsg);

  nmea2000->Open();
  debugI("N2K initialized");
  actisense_reader.SetDefaultSource(75);
  actisense_reader.SetMsgHandler(HandleStreamActisenseMsg);

  // Data Server1
  app.onRepeat(1000, []() {
    if (WiFi.isConnected()) {
      if (!server) server.begin();
      if (server.hasClient()) {
        debugD ("Evaluating client connection");
        if (!client || !client.connected()) {
          debugI("We have a client!");
          client = server.available();
          // Create WiFiClientStream object
         // wifiStream = new WiFiClientStream(&client);
          wifiStream = &client;
          actisense_reader.SetReadStream(wifiStream);


          debugD ("client connected");
          nmea2000->SetForwardStream(wifiStream);
          debugD ("N2K Stream forwarded");
          debugI("New client connected on data server 1");
        } else {
          debugI ("client1 %s", client.connected()?"connected":"not connected");
        }
      } else {
           if (client && !client.connected()) {  /// no active client
            debugI ("closing connection");
            client.stop();
          }
      }
    }
  });

  app.onRepeat (600000L,[](){
      nmea2000->SendProductInformation();
      nmea2000->SendIsoAddressClaim();
  });


  // update the statistics on #processed messages
  app.onRepeat (2000, [](){
      counterActi->emit (num_actisense_messages);
      counterN2K->emit (num_n2k_messages);
  });

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    actisense_reader.ParseMessages();
    nmea2000->ParseMessages();
  });

#if 0
  // enable CAN status polling
  app.onRepeat(100, []() { PollCANStatus(); });
#endif

#ifdef HASDISPLAY
  // initialize the display
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(2);
  display->clearDisplay();
  display->display();

    // turn the display off 10 mins after startup
    app.onDelay (600000,[](){
      displayOff();       
    });

  // update results
  app.onRepeat(1000, []() {

    if (!show_display) return;

    char ip[20];
    display->clearDisplay();
    display->setTextSize(1);
    display->setCursor(0, 0);
    display->setTextColor(SSD1306_WHITE);
    display->printf("SH-ESP32 N2K USB GW\n");
    display->printf("CAN: %s\n", can_state.c_str());
    display->printf("Uptime: %lu\n", millis() / 1000);
    display->printf("RX: %d\n", num_n2k_messages);
    display->printf("TX: %d\n", num_actisense_messages);
    display->printf("");
    WiFi.localIP().toString().toCharArray (ip,sizeof(ip));
    display->printf("IP: %s\n", ip);
    display->printf("Status %s", client.connected()?"Connected":"Idle");


    display->display();

  });

#endif

#ifndef DEBUG_DISABLED
 /// FOR DEBUGGING REMOVE WHEN DONE
  app.onRepeat (1000,[](){
 
    // send a PGN 129029 message in ActiSense ASCII
 // const char *msg = "A000000.000 00FF2 1F805 FFE54D60606E304060352C8FC72B07587DBA743BF39000FFFFFFFFFFFFFF7F23FC104000FF7FFFFFFFFF0";
    const char *msg = "A000000.000 00FF2 1FB02 C5C827160EFFFFFFFFFFFFFFFFFFFFFF484F4F4745FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF474A06FA00AA006405FFFFFFFFFFFFFC0353542E50455445525342555247FFFFFFFFFFFFFFBCE0";
    char out[MAX_STREAM_MSG_BUF_LEN];
    tN2kMsg n2k_msg;

    if (client.connected()) {
      if (actisense_reader.ParseMessage (n2k_msg,(char*)msg)) {  // turn into n2k
        debugV ("Parsed debug message PGN %d PRIO %d SRC %02X DST %02X",n2k_msg.PGN,n2k_msg.Priority,n2k_msg.Source,n2k_msg.Destination);
        actisense_reader.buildMessage(n2k_msg,out,MAX_STREAM_MSG_BUF_LEN);  // turn into ascii again
        debugD ("Incoming message  [%s] l=%d",msg,strlen(msg));
        debugD ("Converted message [%s] l=%d",out,strlen(out));
        debugI ("Sending N2K msg PGN %s",out);
        //wifiStream->println (out);
        num_n2k_messages++;
      } else {
        debugI ("No client connected");
      }

      
    }


  });
#endif


  sensesp_app->start();

}

void loop() { 
  app.tick();
}
