/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>

constexpr uint8_t PAYLOAD_BYTES = 8;

// Serial format
enum Format { OPEN_GLOVES, MOCAP_FUSION, DEBUG };
constexpr Format SERIAL_FORMAT = Format::MOCAP_FUSION;

// Custom hand tracking UUIDs used by peripheral and central
BLEUuid handTrackingServiceUuid(0x691A);
BLEUuid leftHandCharacteristicUuid(0x691B);
BLEUuid rightHandCharacteristicUuid(0x691C);

// Struct containing peripheral info
typedef struct
{
  BLEClientCharacteristic chr;
  uint8_t data[PAYLOAD_BYTES];
} Hand;

typedef struct
{
  char name[16+1];

  uint16_t conn_handle;

  // Each tracker needs its own client service
  BLEClientService service;
  
  // A tracker can handle both hands, or we can have a separate tracker for each
  Hand leftHand;
  Hand rightHand;
} HandTracker;

// Either 1 or 2 trackers depending on whether we have two separate trackers for each hand, or a combo
HandTracker trackers[2];

// Lookups for which trackers contain our left/right data
int8_t leftHandIdx = -1;
int8_t rightHandIdx = -1;

// If the serial output is out-of-order, swizzle our data array indexes to provide the expected order
// Mocap Fusion has the 1st and 2nd index swapped
const uint8_t SWIZZLE_LEFT[] = { 0, 2, 1, 3, 4 };
const uint8_t SWIZZLE_RIGHT[] = { 0, 2, 1, 3, 4 };

// Flag to tell us when to write data out
bool newData = false;

const uint8_t EMPTY[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  println("Hand Tracking Central Device");
  println("----------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 4);
  Bluefruit.setName("HandTrackerRX");

  // Init peripheral pool
  for (uint8_t idx=0; idx < 2; idx++)
  {
    // Invalid all connection handle
    trackers[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    
    trackers[idx].service = BLEClientService(handTrackingServiceUuid);
    trackers[idx].service.begin();
    
    trackers[idx].leftHand.chr = BLEClientCharacteristic(leftHandCharacteristicUuid);
    trackers[idx].leftHand.chr.setNotifyCallback(hand_notify_callback);
    trackers[idx].leftHand.chr.begin();

    trackers[idx].rightHand.chr = BLEClientCharacteristic(rightHandCharacteristicUuid);
    trackers[idx].rightHand.chr.setNotifyCallback(hand_notify_callback);
    trackers[idx].rightHand.chr.begin();
  }

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept HRM service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(handTrackingServiceUuid);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

void print(const char* data)
{
  if (SERIAL_FORMAT == Format::DEBUG) {
    Serial.print(data);
  }
}
void println(const char* data)
{
  if (SERIAL_FORMAT == Format::DEBUG) {
    Serial.println(data);
  }
}
int printf(const char* data, ...)
{
  va_list args;
  if (SERIAL_FORMAT == Format::DEBUG) {
    return Serial.printf(data, args);
  }
  return 0;
}

void printOpenGlovesData(uint8_t* data, uint16_t len)
{
  // for (uint16_t i = 0; i < len; i++) {
  //   if (data[i] < 0x10) Serial.print("0");
  //   Serial.print(data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  uint32_t unpacked[5];
  unpackBitArray(data, 5, 12, unpacked);
  Serial.printf("A%uB%uC%uD%uE%u", unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[4]);
}

void loop()
{
  if (newData)
  {
    if (SERIAL_FORMAT == Format::MOCAP_FUSION)
    {
      Serial.write(rightHandIdx >= 0 ? trackers[rightHandIdx].rightHand.data : EMPTY, PAYLOAD_BYTES);
      Serial.write(leftHandIdx >= 0 ? trackers[leftHandIdx].leftHand.data : EMPTY, PAYLOAD_BYTES);
      Serial.println();
    }
    // else if (SERIAL_FORMAT == Format::OPEN_GLOVES)
    // {
    //   // Actually need two serial ports via the Serial method. Otherwise OPEN_GLOVES works over BT serial.
    // }
    else if (SERIAL_FORMAT == Format::DEBUG || SERIAL_FORMAT == Format::OPEN_GLOVES)
    {
      if (leftHandIdx >= 0) {
        Serial.print("L ");
        printOpenGlovesData(trackers[leftHandIdx].leftHand.data, PAYLOAD_BYTES);
      } else {
        Serial.print("L EMPTY");
      }
      Serial.println();

      if (rightHandIdx >= 0) {
        Serial.print("R ");
        printOpenGlovesData(trackers[rightHandIdx].rightHand.data, PAYLOAD_BYTES);
      } else {
        Serial.print("R EMPTY");
      }
      Serial.println();
      Serial.println();
    }

    newData = false;
  }
  delay(50); //16
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with hand tracking service advertised
  Bluefruit.Central.connect(report);
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Exceeded the number of connections
  if ( id < 0 ) {
    println("Too many hands! Ignoring connection...");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  // Pointer to peer = address of tracker at [id]
  HandTracker* peer = &trackers[id];
  peer->conn_handle = conn_handle;
  //connection_num++;

  println("Connected");
  print("Discovering hand tracking service... ");

  // If hand tracking service is not found, disconnect and return
  if ( !peer->service.discover(conn_handle) )
  {
    println("Found NONE");
    // disconnect since we couldn't find the hand tracking service
    Bluefruit.disconnect(conn_handle);
    return;
  }

  // Once service is found, we continue to discover each individual hand characteristic
  println("Found it");
  
  print("Discovering left hand characteristic... ");
  if ( peer->leftHand.chr.discover() )
  {
    println("Found it");
    leftHandIdx = id;
    if ( peer->leftHand.chr.enableNotify() )
      println("Ready to receive left hand values");
    else
      println("Couldn't enable notify for left hand values. Increase DEBUG LEVEL for troubleshooting");
  }
  else
  {
    println("not found !!!");
  }

  print("Discovering right hand characteristic... ");
  if ( peer->rightHand.chr.discover() )
  {
    println("Found it");
    rightHandIdx = id;
    if ( peer->rightHand.chr.enableNotify() )
      println("Ready to receive right hand values");
    else
      println("Couldn't enable notify for right hand values. Increase DEBUG LEVEL for troubleshooting");
  }
  else
  {
    println("not found !!!");
  }
  
  if (leftHandIdx < 0 || rightHandIdx < 0)
  {
    println("Continue scanning for more peripherals");
    Bluefruit.Scanner.start(0);
  }
  else
  {
    println("Stop scanning for more peripherals, we've found our left and right hands.");
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  // (void) conn_handle;
  // (void) reason;

  // Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  (void) conn_handle;
  (void) reason;

  //connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  trackers[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  if (leftHandIdx == id) leftHandIdx = -1;
  if (rightHandIdx == id) rightHandIdx = -1;

  //Serial.print(trackers[id].name);
  printf("Disconnected %u!\n", id);
}

void hand_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  uint8_t* target = nullptr;
  const uint8_t* swizzle = nullptr;

  // which hand are we looking at?
  for (size_t i=0; i<2; i++)
  {
    auto& peer = trackers[i];
    if (chr == &peer.leftHand.chr) {
      target = peer.leftHand.data;
      swizzle = SWIZZLE_LEFT;
      break;
    }
    if (chr == &peer.rightHand.chr) {
      target = peer.rightHand.data;
      swizzle = SWIZZLE_RIGHT;
      break;
    }
  }

  if (target != nullptr) {
    // Since we may need to reorder the array, we need to unpack it, swizzle, pack it, and copy to target
    uint32_t tmp;
    uint32_t unpacked[5], reordered[5];
    unpackBitArray(data, 5, 12, unpacked);
    for (auto i=0; i<5; i++)
    {
      // Mocap Fusion only supports 10-bit (1024), so if present, divide 12-bit value (4096) by 4
      reordered[i] = unpacked[swizzle[i]] / (SERIAL_FORMAT == Format::MOCAP_FUSION ? 4 : 1);
    }
    packBitArray(reordered, 5, 12, data); // overwrite mutable data param

    memcpy(target, data, len);
    newData = true;
  }
}

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle)
{
  for(int id=0; id<BLE_MAX_CONNECTION; id++)
  {
    if (conn_handle == trackers[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}


void packBitArray(const uint32_t* input, size_t inputSize, uint8_t stride, uint8_t* output) {
  size_t outIndex = 0;
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;

  for (size_t i = 0; i < inputSize; ++i) {
    bitBuffer |= ((uint32_t)(input[i] & ((1 << stride) - 1))) << bitsInBuffer;
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

void unpackBitArray(const uint8_t* input, size_t outputSize, uint8_t stride, uint32_t* output) {
  size_t inIndex = 0;
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;

  for (size_t i = 0; i < outputSize; ++i) {
    while (bitsInBuffer < stride) {
      bitBuffer |= ((uint32_t)input[inIndex++]) << bitsInBuffer;
      bitsInBuffer += 8;
    }
    output[i] = bitBuffer & ((1 << stride) - 1);
    bitBuffer >>= stride;
    bitsInBuffer -= stride;
  }
}
