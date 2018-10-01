  #define DEBUG 0
#define CHANNEL 1
#define HIDE_SSID 1
#define PACKET_BUFFER_ENTRIES 128
// a locally administered mac the first byte is 'r' -> 0x7x 
// which fits one of the the valid ranges,  x2-xx-xx-xx-xx-xx
#define RECEIVER_MAC 'r', 'P', 'i', 'N', 'o', 'w'

/**
   Base on code from 
   Author: Arvind Ravulavaru <https://github.com/arvindr21>


   << This Device Slave >>

   Step 1 : ESPNow Init 
   Step 2 : Update the SSID with a prefix of `slave`
   Step 3 : Set AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : in callback when data arrives, put it into the ring buffer
   Step 6 : take data out of ringbuffer and send it to Serial
*/

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  extern "C" {
    #include <espnow.h>
    #include "user_interface.h"
  }
  #ifndef ESP_OK
    #define ESP_OK 0
  #endif
#else
  #include <esp_now.h>
  #include <WiFi.h>
#endif

#include <PubSubClient.h>

// Ring Buffer structure
struct _pktInfo {
  byte datacopy[256];
  int dataLen;
  uint8_t fromMAC[6]; 
  unsigned char occupied;
  unsigned char unused;// pad struct to a multiple of 4
};


// ring buffer
struct _pktInfo *packetBuffer = (struct _pktInfo *)calloc(PACKET_BUFFER_ENTRIES,sizeof(struct _pktInfo)); // nice big buffer for the packets
int writeIdx = 0;
int readIdx = 0;

// we are going to replace the MAC with a constant value (in the locally administered domain)
// see https://honeywellaidc.force.com/supportppr/s/article/Locally-Administered-MAC-addresses
uint8_t receiverMac[] = {RECEIVER_MAC};

const char deviceTopic[] = "ESPNOW/";

#define SENDTOPIC "ESPNow/key"
#define COMMANDTOPIC "ESPNow/command"
#define SERVICETOPIC "ESPNow/service"

int cpuFreq;
// set frequency to 160MHz up from 80
void preloop_update_frequency() {
#if defined(F_CPU) && (F_CPU == 160000000L)
#warning running at 160MHz
  REG_SET_BIT(0x3ff00014, BIT(0));
  ets_update_cpu_frequency(160);
  cpuFreq = 160;
#endif
}

void initMAC() {
  wifi_set_macaddr(SOFTAP_IF, &receiverMac[0]);
}

// Init ESP Now with fallback
void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("{\"topic\":\"INIT\",\"status\":\"ESPNow Init Success\"}");
  }
  else {
    Serial.println("{\"topic\":\"INIT\",\"status\":\"ESPNow Init Failed\"}");
    ESP.restart();
  }
}



void setup() {
  Serial.begin(115200);
  if (DEBUG) {
    Serial.println("{\"topic\":\"INIT\",\"FileSource\":\"" __FILE__ "\"");
    Serial.println("{\"topic\":\"INIT\",\"FileTimeStamp\":\"" __TIMESTAMP__ "\"");
    Serial.println("{\"topic\":\"INIT\",\"status\":\"AP Config failed\"}");
  }

  initMAC();
  
  //Set device in AP mode so it can receive
  WiFi.mode(WIFI_AP);

  // configure device in AP mode
  char* SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, HIDE_SSID);
  if (!result) {
    Serial.println("{\"topic\":\"INIT\",\"status\":\"AP Config failed\"}");
  } else {
    Serial.println("{\"topic\":\"INIT\",\"status\":\"AP Config success, broadcasting with AP " + String(SSID) + "\"}");
    Serial.println("{\"topic\":\"INIT\",\"status\":\"AP MAC address:" + WiFi.softAPmacAddress() +  + "\"}");
    Serial.println("{\"topic\":\"INFO\",\"version\":\"buffering style\"}");

  }
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
// made as fast as possible to allow for a decent amount of buffering
#ifdef ESP8266
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, unsigned char data_len) {
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  if (DEBUG) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Last Packet Recv from: "); Serial.println(macStr);
    Serial.print("Last Packet Recv len: "); Serial.println(data_len);
    Serial.print("Last Packet Recv Data: "); Serial.println((char *)data);
    Serial.println("");
  }

  // really need to handle dropping Packets!
  struct _pktInfo *writePtr = &packetBuffer[writeIdx];
  if (writePtr->occupied) {
    if (DEBUG) {
      Serial.printf("Packet at idx:%d is occupied\n", writeIdx);
    }
    return; // drop packet!
#warning should do something about dropped packets
  }
  if (DEBUG) {
    Serial.printf("copying from %04x to %04x\n",data, writePtr->datacopy);
    Serial.printf("base address is at:%04x\n", packetBuffer);
  }
  // dont change writeIdx until we're done copying data!
  memcpy(writePtr->datacopy,data,data_len);
  memcpy(writePtr->fromMAC, mac_addr, 6);
  writePtr->dataLen = data_len;
  writePtr->occupied = 1;

  // for write we check ahead one (read is the other way around)
  if (writeIdx == PACKET_BUFFER_ENTRIES-1) {
    writeIdx = 0;
  } else {
    ++writeIdx;
  }
  if (DEBUG) {
    Serial.printf("NEXT writeIdx is now %d at %04X\n", writeIdx,&writeIdx);
  }
}

void loop() {
  // Chill
  if (DEBUG) {
    Serial.print("read from ") ; Serial.println(readIdx);
    Serial.print("write to ");  Serial.println(writeIdx);
    Serial.printf("write to addr is %04X\n", &writeIdx);
  }

  while (packetBuffer[readIdx].occupied) {
    struct _pktInfo *readPtr = &packetBuffer[readIdx];
    if (DEBUG) {
      Serial.printf("Got Data: %s\n", readPtr->datacopy);
    }
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
      readPtr->fromMAC[0], readPtr->fromMAC[1], readPtr->fromMAC[2], readPtr->fromMAC[3], readPtr->fromMAC[4], readPtr->fromMAC[5]);
    String payload = "";
    if (readPtr->datacopy[0] == '{') { // already in JSON format, augment it with the mac address!
      payload += "{";
      payload += "\"mac\":\"";
      payload += &macStr[0];
      payload += "\",";
      
      payload += "\"idx\":";
      payload += readIdx;
      payload += ",";

      payload += (char *)&readPtr->datacopy[1]; // hacky but replaces the { with some useful stuff
      // assume the payload ends with "}"
    } else {
      payload += "{";
      payload += "\"topic\":\"ESPNOW\", \"data\":\"" + String((char*)readPtr->datacopy) + "\"";
      payload += ",";
      payload += "\"mac\":\""; 
      payload += &macStr[0];
      payload += "\",";
      payload += "\"idx\":";
      payload += readIdx;
      payload += "";
      payload += "}";
    }
    Serial.println(payload);
    memset(readPtr,0,sizeof(struct _pktInfo));

    readIdx++;
    if (readIdx == PACKET_BUFFER_ENTRIES) {
      if (DEBUG) {
        Serial.println("Resetting readIdx");
      }
      readIdx = 0;
    }
    if (DEBUG) {
      Serial.printf("readIdx is now %d\n", readIdx);
    }
  } // end while next idx is occupied

  if (DEBUG) {
    Serial.print("-");
    delay(250);
  } else {
    delay (10); // I wonder how fast this can be pushed
  }
}




