#include <Arduino.h>
// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>

// #define SERVICE_UUID        "13204743-8aac-4791-b134-ae77491306b7"
// #define CHARACTERISTIC_UUID "974f9fff-e2e0-452f-a32d-618b3a3635bb"

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting BLE work!");

//   // Initialize BLE
//   BLEDevice::init("Tiemen esp32");
//   BLEServer *pServer = BLEDevice::createServer();
//   BLEService *pService = pServer->createService(SERVICE_UUID);
//   BLECharacteristic *pCharacteristic = pService->createCharacteristic(
//                                          CHARACTERISTIC_UUID,
//                                          BLECharacteristic::PROPERTY_READ |
//                                          BLECharacteristic::PROPERTY_WRITE
//                                        );

//   // Set initial value for the characteristic
//   pCharacteristic->setValue("van Tiemen Proost");
//   pService->start();

//   // Start advertising
//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(SERVICE_UUID);
//   pAdvertising->setScanResponse(true);
//   pAdvertising->setMinPreferred(0x06);  // Functions that help with iPhone connection issues
//   pAdvertising->setMinPreferred(0x12);
//   BLEDevice::startAdvertising();
//   Serial.println("Characteristic defined! Now you can read it in your phone!");
// }

// void loop() {
//   if (Serial.available()) {
//     SerialBT.write(Serial.read());
//   }
//   if (SerialBT.available()) {
//     Serial.write(SerialBT.read());
//   }
//   delay(20);
// }


/*
  Complete Getting Started Guide: https://RandomNerdTutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
  Ported to Arduino ESP32 by Evandro Copercini
*/
// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEScan.h>
// #include <BLEAdvertisedDevice.h>

// int scanTime = 5; // in seconds
// BLEScan* pBLEScan;

// class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//   void onResult(BLEAdvertisedDevice advertisedDevice) {
//     Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
//   }
// };

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Scanning...");

//   BLEDevice::init("");
//   pBLEScan = BLEDevice::getScan(); //create new scan
//   pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//   pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
//   pBLEScan->setInterval(100);
//   pBLEScan->setWindow(99);  // less or equal setInterval value
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   BLEScanResults foundDevices = pBLEScan->start(scanTime, false); // updated line
//   Serial.print("Devices found: ");
//   Serial.println(foundDevices.getCount());
//   Serial.println("Scan done!");
//   pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory
//   delay(2000);
// }

// #include "BluetoothSerial.h"

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// BluetoothSerial SerialBT;

// void setup() {
//   Serial.begin(115200);
//   SerialBT.begin("ESP32tiemen"); //Bluetooth device name
//   Serial.println("The device started, now you can pair it with bluetooth!");
// }

// void loop() {
//   if (Serial.available()) {
//     SerialBT.write(Serial.read());
//   }
//   if (SerialBT.available()) {
//     Serial.write(SerialBT.read());
//   }
//   delay(20);
// }

// #include "BluetoothSerial.h"
// #include <OneWire.h>
// #include <DallasTemperature.h>

// // Check if Bluetooth configs are enabled
// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// // Bluetooth Serial object
// BluetoothSerial SerialBT;

// // GPIO where LED is connected to
// const int ledPin =  D3;

// // GPIO where the DS18B20 is connected to
// const int oneWireBus = 32;          
// // Setup a oneWire instance to communicate with any OneWire devices
// OneWire oneWire(oneWireBus);
// // Pass our oneWire reference to Dallas Temperature sensor 
// DallasTemperature sensors(&oneWire);

// // Handle received and sent messages
// String message = "";
// char incomingChar;
// String temperatureString = "";

// // Timer: auxiliar variables
// unsigned long previousMillis = 0;    // Stores last time temperature was published
// const long interval = 10000;         // interval at which to publish sensor readings

// void setup() {
//   pinMode(ledPin, OUTPUT);
//   Serial.begin(115200);
//   // Bluetooth device name
//   SerialBT.begin("espTiemenProost");
//   Serial.println("The device started, now you can pair it with bluetooth!");
// }

// void loop() {
//   unsigned long currentMillis = millis();
//   // Send temperature readings
//   if (currentMillis - previousMillis >= interval){
//     previousMillis = currentMillis;
//     sensors.requestTemperatures(); 
//     temperatureString = String(sensors.getTempCByIndex(0)) + "C  " +  String(sensors.getTempFByIndex(0)) + "F";
//     SerialBT.println(temperatureString); 
//   }
//   // Read received messages (LED control command)
//   if (SerialBT.available()){
//     char incomingChar = SerialBT.read();
//     if (incomingChar != '\n'){
//       message += String(incomingChar);
//     }
//     else{
//       message = "";
//     }
//     Serial.write(incomingChar);  
//   }
//   // Check received message and control output accordingly
//   if (message =="aan"){
//     digitalWrite(ledPin, HIGH);
//   }
//   else if (message =="uit"){
//     digitalWrite(ledPin, LOW);
//   }
//   delay(20);
// }


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const int ledPin = 2; // Use the appropriate GPIO pin for your setup

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string ledvalue  = pLedCharacteristic->getValue(); 
    String value = String(ledvalue.c_str());
    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Create the BLE Device
  BLEDevice::init("TIEMEN");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("New value notified: ");
    Serial.println(value);
    delay(3000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}