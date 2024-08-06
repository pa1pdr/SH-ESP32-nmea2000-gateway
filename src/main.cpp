#include <Arduino.h>

#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// SDA and SCL pins on SH-ESP32
#define SDA_PIN GPIO_NUM_16
#define SCL_PIN GPIO_NUM_17

#define ESP_BTNR GPIO_NUM_0
// ports for the 3 WGX1 data servers
#define WGX1_PORT1 60001
#define WGX1_PORT2 60002
#define WGX1_PORT3 60003 

#include <ActisenseReader.h>
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



WiFiServer server1 (WGX1_PORT1);


String hostname = "WGX1-GW";


auto counterActi = new SKOutputInt ("sensorDevice." + hostname +  ".ActiSense_PGNs","",new SKMetadata (""));
auto counterN2K = new SKOutputInt ("sensorDevice." + hostname +  ".N2K_PGNs","",new SKMetadata (""));


tActisenseASCIIReader actisense_reader;

TwoWire *i2c;
Adafruit_SSD1306 *display;

tNMEA2000 *nmea2000;

class WiFiClientStream : public Stream {
public:
    WiFiClientStream(WiFiClient* client) : client(client) {}

    size_t write(uint8_t data) override {
        debugI ("writing [%s]",data);
        return client->write(data);
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        debugI ("Writing [%s]",buffer);
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
    u_char buffer[80];
    u_int len = 0;
};

WiFiClient client;
WiFiClientStream* wifiStream;


void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

int num_n2k_messages = 0;
void HandleStreamN2kMsg(const tN2kMsg &message) {
  num_n2k_messages++;
  ToggleLed();

  // UNTESTED
  if (client && client.connected() && wifiStream->available()) {
      debugD ("Sending N2K msg PGN %d",message.PGN);
      message.SendInActisenseFormat (wifiStream);
  }

}

int num_actisense_messages = 0;
void HandleStreamActisenseMsg(const tN2kMsg &message) {
  //N2kMsg.Print(&Serial);
  num_actisense_messages++;
  ToggleLed();
  debugD ("Actisense msg PGN %d",message.PGN);
  nmea2000->SendMsg(message);
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
#else
  Serial.begin(115200);
#endif
  delay(100);
  pinMode (ESP_BTNR,INPUT_PULLDOWN);


// TODO Setup Sensesp connection: we need to be connected to the network
  SensESPAppBuilder builder;


  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname(hostname)
                    ->enable_ota("mypassword")
                  //  ->enable_wifi_signal_sensor()
                  //  ->enable_free_mem_sensor()
                  //  ->enable_ip_address_sensor()
                    ->set_button_pin(ESP_BTNR)  // reset
                    ->get_app();

  // TODO Set up a port (or 2) where we can interact with WGX clients

  // TODO Add debugging capabilities over the network

  // MAYBE Cater for a touch button to enable the display and move onto
  // different pages

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
      "20210331",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 NMEA 2000 WGX-1 GW",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2024-07-12)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );

  // Set device information
  nmea2000->SetDeviceInformation(
      20210331,  // Unique number. Use e.g. Serial number.
      137,       // Device function=NMEA 2000 Wireless Gateway. See codes on
            // https://canboat.github.io/canboat/canboat.html#indirect-lookup-DEVICE_FUNCTION
      25,  // Makes it an internetwork device
           // https://canboat.github.io/canboat/canboat.html#lookup-DEVICE_CLASS
      641,  // Diverse Yacht Services. Just choosen free from code list on
            // https://canboat.github.io/canboat/canboat.html#lookup-INDUSTRY_CODE
      4  // Industry Group: Marine
  );

  nmea2000->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  // text

  nmea2000->EnableForward(true);
  // nmea2000->SetForwardType(tNMEA2000::fwdt_Actisense);         // Show bus data in actisens fmt
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
      if (!server1) server1.begin();
      if (server1.hasClient()) {
        debugD ("Evaluating client connection");
        if (!client || !client.connected()) {
          debugI("We have a client!");
          client = server1.available();
          // Create WiFiClientStream object
          wifiStream = new WiFiClientStream(&client);
          actisense_reader.SetReadStream(wifiStream);


          debugD ("client connected");
          nmea2000->SetForwardStream(wifiStream);
          debugD ("N2K Stream forwarded");
          debugI("New client connected on data server 1");
        } else {
          debugI ("client1 %s", client.connected()?"connected":"not connected");
        }
      } 
    }
  });




  app.onRepeat (1000, [](){
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

/// FOR DEBUGGING
  app.onRepeat (5000,[](){
    /* tN2kMsg N2kMsg;
     SetN2kTransmissionParameters(N2kMsg, 
                                 0,
                                 N2kTG_Forward,  // gear position
                                 N2kDoubleNA,    // oilpressure
                                 367.0);
    nmea2000->SendMsg(N2kMsg);
    */
    const char *msg = "A173321.107 23FF7 1F513 012F3070002F30709F";
    if (client.connected()) {
      debugD ("Sending N2K msg PGN %s",msg);
      wifiStream->println (msg);
    }
  });

  // update results
  app.onRepeat(1000, []() {
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
  //  display->printf("Status %s", client1?"Connected":"Idle");


    display->display();

   // num_n2k_messages = 0;
   // num_actisense_messages = 0;
  });

  sensesp_app->start();

}

void loop() { 
  app.tick();
}
