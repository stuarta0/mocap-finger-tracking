#include <bluefruit.h>

constexpr int ANALOG_RESOLUTION = 12;  // 10 or 12-bit
constexpr int ANALOG_BOUND = (1 << ANALOG_RESOLUTION) - 1;

// Hand definition
enum Hand { LEFT, RIGHT };
constexpr Hand HAND = Hand::LEFT;

// Set FPS for how frequently readings should be taken
constexpr size_t FPS = 10;
constexpr size_t DELAY_TIME = 1000 / FPS;  // delayTime calculated for loop()

// Simulate readings in a sine wave
constexpr bool SIMULATED = false;
unsigned long time;

// Calculate the full range of each finger during setupTime at power on if SETUP_RANGE = true.
// User must open and close their fist during this time to detect the full range for each finger.
constexpr bool SETUP_RANGE = false;
constexpr size_t SETUP_TIME = 10000; //ms
bool setupComplete = !SETUP_RANGE;
uint16_t mins[] = { ANALOG_BOUND, ANALOG_BOUND, ANALOG_BOUND, ANALOG_BOUND, ANALOG_BOUND };
uint16_t maxs[] = { 0, 0, 0, 0, 0 };

// BLE Services
BLEDis  bledis;  // Device Information Service

// Hand tracking Service and Characteristic using custom 16-bit UUIDs (our hardware doesn't support 128-bit UUID filtering)
BLEUuid serviceUUID  (0x691A);
BLEUuid leftCharUUID (0x691B);
BLEUuid rightCharUUID(0x691C);

BLEService handTrackingService = BLEService(serviceUUID);  // Central receiver must use the same UUID
BLECharacteristic handDataChar = BLECharacteristic(HAND == Hand::LEFT ? leftCharUUID : rightCharUUID);

void setup()
{
  analogReadResolution(ANALOG_RESOLUTION);

  pinMode(LED_BUILTIN, OUTPUT);

  // Track fingers
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);  
  pinMode(A4, INPUT);

  // Not necessary on peripheral, but used for debug
  Serial.begin(115200);
  
  Serial.println("Hand Tracker Transmitter (Peripheral)");
  Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("HandTrackerTX");
  Bluefruit.setConnLedInterval(250);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("mocap-finger-tracker");
  bledis.setModel("Transmitter");
  bledis.begin();

  // Start custom service and characteristic
  handTrackingService.begin();

  handDataChar.setProperties(CHR_PROPS_NOTIFY);
  handDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // Notify only
  handDataChar.setFixedLen(8); // packet is 8 packed bytes
  handDataChar.begin();

  // Set up and start advertising

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Add custom service UUID
  Bluefruit.Advertising.addService(handTrackingService);
  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode

  // If we're not running the setup, start advertising immediately
  if (!SETUP_RANGE)
  {
    // Otherwise we won't start advertising until we've finished calibrating
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  // if (strcmp(central_name, "HandTrackerRX") != 0)
  // {
  //   Serial.println("This was not the connection we were looking for. Disconnecting.");
  //   connection->disconnect();
  // }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}

/*******************************************************
 * APPLICATION CODE BELOW                              *
 *******************************************************/

uint32_t readFinger(uint32_t pin) {
  if (SIMULATED) {
    // 360 degrees in 2 seconds = 180 d/s = 180 * (time / 1000)
    // Each finger is 90 degrees apart, or 90/5 = 18deg or 0.1745 rad
    return (sin((time / 1000.0) * PI + pin * 0.1745) / 2.0 + 0.5) * 4096;
  }
  return analogRead(pin);
}

void packBitArray(const uint32_t* input, size_t inputSize, uint8_t stride, uint8_t* output) {
  int bitmask = (1 << stride) - 1;
  size_t outIndex = 0;
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;

  for (size_t i = 0; i < inputSize; ++i) {
    // if stride is 12, bitmask is 00001111 11111111
    // therefore our packing looks like:
    // input array indices: [0000 0000] [1111 0000] [1111 1111] [2222 2222] ...
    // output array bytes:  [8765 4321] [4321 CBA9] [CBA9 8765] ...
    bitBuffer |= ((uint32_t)(input[i] & bitmask)) << bitsInBuffer;
    bitsInBuffer += stride;

    while (bitsInBuffer >= 8) {
      output[outIndex++] = bitBuffer & 0xFF;
      bitBuffer >>= 8;
      bitsInBuffer -= 8;
    }
  }

  // Flush any remaining bits
  if (bitsInBuffer > 0) {
    output[outIndex++] = bitBuffer & ((1 << bitsInBuffer) - 1);
  }
}

size_t scale(size_t value, size_t mini, size_t maxi, size_t range)
{
  if (!SETUP_RANGE) return value;
  return max(0, min(range, ((value - mini) * range) / (maxi - mini)));
}

void loop()
{
  time = millis();
  if (!setupComplete)
  {
    if (time < SETUP_TIME)
    {
      digitalWrite(LED_BUILTIN, (time / 1000) % 2 == 0 ? HIGH : LOW);

      uint32_t data[] = {
        readFinger(A0),
        readFinger(A1),
        readFinger(A2),
        readFinger(A3),
        readFinger(A4)
      };

      // Calculate the full range of each finger during setup
      // User must open and close their fist during this time.
      for (auto i=0; i<5; i++) {
        mins[i] = min(mins[i], data[i]);
        maxs[i] = max(maxs[i], data[i]);
      }
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);

      // create a small buffer around the mins and maxs so we don't clip
      uint8_t buffer = ANALOG_BOUND * 0.02;
      Serial.println(buffer);
      for (auto i=0; i<5; i++) {
        mins[i] = max(mins[i] - buffer, 0);
        maxs[i] = min(maxs[i] + buffer, ANALOG_BOUND);
        Serial.printf("%u:%u-%u, ", i, mins[i], maxs[i]);
      }
      Serial.println();
      setupComplete = true;

      // Now we've finished calibration, start advertising
      Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
    }
  }
  else if (Bluefruit.connected())
  {
    uint32_t data[5];
    for (uint8_t i=0; i<5; i++) {
      // Pin A0 = 0, A1 = 1, etc.
      data[i] = scale(readFinger(i), mins[i], maxs[i], 1024);
    }

    // Pack and send the finger data.
    // The XIAO ADC is 12 bit, which packs 5 values into 8 bytes (with 4 padded).
    //     (12 bits / 8 bit byte) * 5 fingers = 8 bytes.
    //     (10 bits / 8 bit byte) * 5 fingers = 7 bytes.
    // If we had 10 values (5 fingers + 5 splay), we only need 15 bytes rather than 20 if we pack them this way.
    // If we had 10 values, but only 10 bit ADC, we only need (10/8)*10 = 13 bytes.
    uint8_t packed[8];
    packBitArray(data, 5, 12, packed);
    handDataChar.notify(packed, sizeof(packed));
  }
  else 
  {
    // while we're waiting to connect, delay longer
    delay(200);
  }

  delay(DELAY_TIME);
}

