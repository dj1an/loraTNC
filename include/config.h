enum KissMarker {
  Fend = 0xc0,
  Fesc = 0xdb,
  Tfend = 0xdc,
  Tfesc = 0xdd
};

enum KissState {
  Void = 0,
  GetCmd,
  GetData,
  GetP,
  GetSlotTime,
  Escape
};

enum KissCmd {
  Data = 0x00,
  P = 0x02,
  SlotTime = 0x03,
  NoCmd = 0x80
};

#define DEFAULT_P             128
#define DEFAULT_SLOT_TIME     500
#define CFG_BAUDRATE          115200

// #define CFG_LORA_PIN_SS       16
// #define CFG_LORA_PIN_RST      14
// #define CFG_LORA_PIN_DIO0     26
// #define CFG_LORA_PIN_MOSI     23
// #define CFG_LORA_PIN_MISO     19
// #define CFG_LORA_PIN_SCK      18

// ttgo
#define CFG_LORA_PIN_SS       18
#define CFG_LORA_PIN_RST      23
#define CFG_LORA_PIN_DIO0     26
#define CFG_LORA_PIN_MOSI     27
#define CFG_LORA_PIN_MISO     19
#define CFG_LORA_PIN_SCK      5

//dev board
// #define CFG_LORA_PIN_SS       5
// #define CFG_LORA_PIN_RST      22
// #define CFG_LORA_PIN_DIO0     26
// #define CFG_LORA_PIN_MOSI     23
// #define CFG_LORA_PIN_MISO     19
// #define CFG_LORA_PIN_SCK      18

#define CFG_LORA_FREQ         434.900E6
#define CFG_LORA_SYNC_WORD    0x12
#define CFG_LORA_BW           125e3
#define CFG_LORA_SF           12
#define CFG_LORA_CR           5
#define CFG_LORA_PWR          17
#define CFG_LORA_ENABLE_CRC   true

#define CONN_RETRY_SLEEP_MS   1000
#define LOOP_SLEEP_MS         10

#define LED_PIN               25
#define WIFI_SSID "wlan"
#define WIFI_PASS "wlan12345"
#define AXUDP_LISTEN_PORT 9002
#define AXUDP_SEND_PORT 9001
#define AXUDP_TARGET "192.168.2.197"
#define OTA_HOSTNAME "loraTNC"