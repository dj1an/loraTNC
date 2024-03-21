#include <SPI.h>
#include <RadioLib.h>
#include "config.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiUDP udp;
unsigned char udpRxBuffer[255];
unsigned char loraRxBuffer[255];
unsigned int loraRxBufferLen = 0;

void kissResetState();
void onRadioDataAvailable();
void onSerialDataAvailable();
void onUdpDataAvailable();
void initWiFi();

uint16_t ax25crc16(unsigned char *data_p, uint16_t length);
unsigned char framedata[513];
unsigned char framep1[255];
unsigned char framep2[255];
uint16_t crc; 
int framelen = 0;

KissState kissState_;
KissCmd kissCmd_;

byte csmaP_ = DEFAULT_P;
long csmaSlotTime_ = DEFAULT_SLOT_TIME;
long csmaSlotTimePrev_ = 0;

int transmissionState = RADIOLIB_ERR_NONE;
// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;
volatile bool receivedFlag = false;
volatile bool operationDone = false;

void setTxFlag(void);
void setRxFlag(void);
void setFlag(void);
void radioloop(); 

//SPIClass spi(VSPI);
//SPISettings spiSettings(200000, MSBFIRST, SPI_MODE0);
//SX1278 radio = new Module(CFG_LORA_PIN_SS, CFG_LORA_PIN_DIO0, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO1, spi, spiSettings);

#ifdef HELTEC_V3
SX1262 radio = new Module(CFG_LORA_PIN_SS, CFG_LORA_PIN_DIO1, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO0);
#else
SX1278 radio = new Module(CFG_LORA_PIN_SS, CFG_LORA_PIN_DIO0, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO1);
#endif


void setup() {
  Serial.begin(CFG_BAUDRATE);
  while (!Serial);
  #ifdef HELTEC_V3
  SPI.begin(CFG_LORA_PIN_SCK, CFG_LORA_PIN_MISO, CFG_LORA_PIN_MOSI, CFG_LORA_PIN_SS);
  #else
  SPI.begin(CFG_LORA_PIN_SCK, CFG_LORA_PIN_MISO, CFG_LORA_PIN_MOSI, CFG_LORA_PIN_SS); //SCLK, MISO, MOSI, SS
  #endif

  //radio = new Module(CFG_LORA_PIN_SS, CFG_LORA_PIN_DIO0, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO1);
  
  //LoRa.setPins(CFG_LORA_PIN_SS, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO0);
  
  // while (!LoRa.begin(CFG_LORA_FREQ)) {
  //   delay(CONN_RETRY_SLEEP_MS);
  // }
  // LoRa.setSyncWord(CFG_LORA_SYNC_WORD);
  // LoRa.setSpreadingFactor(CFG_LORA_SF);
  // LoRa.setSignalBandwidth(CFG_LORA_BW);
  // LoRa.setCodingRate4(CFG_LORA_CR);
  // LoRa.setTxPower(CFG_LORA_PWR);
  // if (CFG_LORA_ENABLE_CRC) {
  //   LoRa.enableCrc();
  // }

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin(CFG_LORA_FREQ, CFG_LORA_BW, CFG_LORA_SF, CFG_LORA_CR, CFG_LORA_SYNC_WORD, CFG_LORA_PWR, 8, 0);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
    radio.setCRC(true);
    //radio.setPacketSentAction(setTxFlag);
    //radio.setPacketReceivedAction(setRxFlag);
    #ifdef HELTEC_V3
      radio.setDio1Action(setFlag);
    #else
      radio.setDio0Action(setFlag, RISING);
    #endif
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    //while (true);
  }

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  
  initWiFi();
  Serial.print("Wifi RSSI: ");
  Serial.println(WiFi.RSSI());
  udp.begin(AXUDP_LISTEN_PORT);

  Serial.println("Setup Finish");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  delay(300);  digitalWrite(LED_PIN, LOW);  delay(100);  
  digitalWrite(LED_PIN, HIGH);  delay(100);  digitalWrite(LED_PIN, LOW);  delay(100);
  digitalWrite(LED_PIN, HIGH);  delay(300);  digitalWrite(LED_PIN, LOW);
}

void loop() {
  radioloop(); 
  long currentTime = millis();
  //if (LoRa.parsePacket() > 0) {
    if (false){
    onRadioDataAvailable();
  } else if (currentTime > csmaSlotTimePrev_ + csmaSlotTime_ && random(0, 255) < csmaP_) {
    if (Serial.available()) {
      onSerialDataAvailable();
    }
    csmaSlotTimePrev_ = currentTime;
    
  } else if (udp.parsePacket() > 0){
      onUdpDataAvailable();
  }
  
  ArduinoOTA.handle();
  delay(LOOP_SLEEP_MS);
}

void onUdpDataAvailable(){
  digitalWrite(LED_PIN, HIGH);
  int len = udp.read(udpRxBuffer, 255);
  if (len > 2){
    udpRxBuffer[len] = '\0';
  }

  //Serial.printf("UDP packet contents: %x\n", udpRxBuffer);
 
  if (len > 253){
    // split packet here.. todo...
    return;
  }

  //LoRa.beginPacket();
  //uint16_t crc = ax25crc16(framedata,  framelen);
  // for (size_t i = 0; i < len; i++) {
  //   LoRa.write(udpRxBuffer[i]);
  // }

  //LoRa.write((uint8_t)(crc >> 8)); // high byte
  //LoRa.write((uint8_t)(crc)); // low byte
  //LoRa.endPacket();

  transmissionState = radio.startTransmit(udpRxBuffer, len);
  transmittedFlag = true;
  digitalWrite(LED_PIN, LOW);
}


void kissResetState()
{
  kissCmd_ = KissCmd::NoCmd;
  kissState_ = KissState::Void;
}

void onRadioDataAvailable() 
{
  digitalWrite(LED_PIN, HIGH);

  //while (LoRa.available()) {
  // while (false){
  //   byte rxByte = LoRa.read();
  //   loraRxBuffer[loraRxBufferLen] = rxByte;
  //   loraRxBufferLen++;
  // }

  // write rx data to Serial, cut 2 bytes CRC
  Serial.write(KissMarker::Fend);
  Serial.write(KissCmd::Data);
  for (size_t i = 0; i < loraRxBufferLen-2; i++) {
    if (loraRxBuffer[i] == KissMarker::Fend) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfend);
    }
    else if (loraRxBuffer[i] == KissMarker::Fesc) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfesc);
    }
    else {
      Serial.write(loraRxBuffer[i]);
    }
  }
  Serial.write(KissMarker::Fend);


  loraRxBuffer[loraRxBufferLen] = '\0';
  //debug
  // udp.beginPacket("255.255.255.255",3333);
  // udp.printf("Lora Rx: len:%d\n", loraRxBufferLen);
  // for (size_t i = 0; i < loraRxBufferLen; i++) {
  //   udp.printf("%x ", loraRxBuffer[i]);
  // }
  // uint16_t crc = ax25crc16(loraRxBuffer,  loraRxBufferLen-2);
  // uint8_t crchi = (uint8_t)(crc >> 8); // high byte
  // uint8_t crclo = (uint8_t)(crc); // low byte
  // udp.printf("\ncrc: %x %x\n", crchi, crclo);
  // udp.printf("\n------------------------\n");
  // udp.endPacket();

  //axudp
  //const char * udpTarget = AXUDP_TARGET;
  udp.beginPacket(AXUDP_TARGET, AXUDP_SEND_PORT);
  udp.write(loraRxBuffer, loraRxBufferLen);
  udp.endPacket();

  digitalWrite(LED_PIN, LOW);
  loraRxBuffer[0] = '\0'; // clear
  loraRxBufferLen = 0;
}

void onSerialDataAvailable() 
{ 
  while (Serial.available()) {
    digitalWrite(LED_PIN, HIGH);
    int rxResult = Serial.read();
    if (rxResult == -1) break;
    
    byte rxByte = (byte)rxResult;

    switch (kissState_) {
      case KissState::Void:
        if (rxByte == KissMarker::Fend) {
          kissCmd_ = KissCmd::NoCmd;
          kissState_ = KissState::GetCmd;
        }
        break;
      case KissState::GetCmd:
        if (rxByte != KissMarker::Fend) {
          if (rxByte == KissCmd::Data) {
            //LoRa.beginPacket();
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetData;
          }
          else if (rxByte == KissCmd::P) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetP;
          }
          else if (rxByte == KissCmd::SlotTime) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetSlotTime;
          }
          else {
            kissResetState();
          }
        }
        break;
      case KissState::GetP:
        csmaP_ = rxByte;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetSlotTime:
        csmaSlotTime_ = (long)rxByte * 10;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetData:
        if (rxByte == KissMarker::Fesc) {
          kissState_ = KissState::Escape;
        }
        else if (rxByte == KissMarker::Fend) {
          if (kissCmd_ == KissCmd::Data) {
            

            if (framelen > 253){
              // split Frame todo...
              // memcpy(framedata, framep1, 253);
              // uint16_t p1Crc = ax25crc16(framep1,  253);
              // uint8_t p1CrcH = (uint8_t)(p1Crc >> 8); // high byte
              // uint8_t p1CrcL = (uint8_t)(p1Crc); // low byte
              

            } else {
              // dont split

              // LoRa.beginPacket();
              uint16_t crc = ax25crc16(framedata,  framelen);
              framedata[framelen] = (uint8_t)(crc >> 8); // high byte
              framelen++;
              framedata[framelen] = (uint8_t)(crc); // low byte
              framelen++;
              // for (size_t i = 0; i < framelen; i++) {
              //   LoRa.write(framedata[i]);
              // }
              // // add ax25 crc
              // LoRa.write((uint8_t)(crc >> 8)); // high byte
              // LoRa.write((uint8_t)(crc)); // low byte
              // LoRa.endPacket();

              transmissionState = radio.startTransmit(framedata, framelen);
              transmittedFlag = true;
            }

            // debug
            // udp.beginPacket(udpAddress,udpPort);
            // udp.printf("Rx: ");
            // udp.printf("data: %s len: %d crc: %x", framedata, framelen, crc);
            // udp.printf("\n------------------------\n");
            // udp.endPacket();
            framedata[0] = '\0'; // clear
            framelen = 0;
          }
          kissResetState();
        }
        else if (kissCmd_ == KissCmd::Data) {
          //LoRa.write(rxByte);
          //udp.printf("%x ", rxByte);
          framedata[framelen] = rxByte;
          framelen++;
        }
        break;
      case KissState::Escape:
        if (rxByte == KissMarker::Tfend) {
          //LoRa.write(KissMarker::Fend);
          kissState_ = KissState::GetData;
          framedata[framelen] = KissMarker::Fend;
          framelen++;
        }
        else if (rxByte == KissMarker::Tfesc) {
          //LoRa.write(KissMarker::Fesc);
          kissState_ = KissState::GetData;
          framedata[framelen] = KissMarker::Fesc;
          framelen++;
        }
        else {
          kissResetState();
        }
        break;
      default:
        break;
    }
  }
  digitalWrite(LED_PIN, LOW);
}


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(OTA_HOSTNAME); //define hostname
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi ..");
  int cnt = 1;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    cnt += 1;

    if (cnt > 10){
      Serial.print("WiFi Connect Failed...");
      return;
    }
    delay(1000);
  }
  digitalWrite(2, LOW);
  Serial.println(WiFi.localIP());

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      char progBuffer[24];
      sprintf(progBuffer, "OTA Updating... %u%%", (progress / (total / 100)));

    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA Ready");

}


uint16_t ax25crc16(unsigned char *data_p, uint16_t length) {
  uint16_t crc = 0xFFFF;
  uint32_t data;
  uint16_t crc16_table[] = {
          0x0000, 0x1081, 0x2102, 0x3183,
          0x4204, 0x5285, 0x6306, 0x7387,
          0x8408, 0x9489, 0xa50a, 0xb58b,
          0xc60c, 0xd68d, 0xe70e, 0xf78f
  };

  while(length--){
      crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p & 0xf)];
      crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p++ >> 4)];
  }

  data = crc;
  crc = (crc << 8) | (data >> 8 & 0xff); // do byte swap here that is needed by AX25 standard
  return (~crc);
}


// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setTxFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

void radioloop(){
  
if (operationDone){
  operationDone = false;

  if(transmittedFlag) {
    // reset flag
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("Tx failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();
    radio.startReceive();
  } else {
    
    //if(receivedFlag) {
    // reset flag
    //receivedFlag = false;

    // you can read received data as an Arduino String
    //String str;
    //int state = radio.readData(str);

    // you can also read received data as byte array
    
      //byte byteArr[8];
      loraRxBufferLen = radio.getPacketLength();
      int state = radio.readData(loraRxBuffer, loraRxBufferLen);
    
    
    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1278] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1278] Data:\t\t"));
      //Serial.println(str);
      onRadioDataAvailable();
      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1278] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1278] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1278] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);

    }
  }
}
}

