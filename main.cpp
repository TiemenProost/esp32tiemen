#include <Arduino.h>
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

// Pin configuratie voor RGB LED
const int redPin = D1;    // Pin voor Rood
const int greenPin = D2;  // Pin voor Groen
const int bluePin = D3;   // Pin voor Blauw

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
    std::string ledvalue = pLedCharacteristic->getValue(); 
    if (ledvalue.length() > 0) {
      Serial.print("Characteristic event, written: ");
      int receivedValue = static_cast<int>(ledvalue[0]);
      Serial.println(receivedValue); // Print de ontvangen waarde

      // Reset de RGB LED (zet alles uit)
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);

 if (receivedValue == 1) {
    digitalWrite(greenPin, HIGH); // Zet groen aan
    digitalWrite(redPin, LOW);    // Zet rood uit
    digitalWrite(bluePin, LOW);   // Zet blauw uit
    Serial.println("groen is aan"); // Correcte functie voor printen
} else if (receivedValue == 2) {
    digitalWrite(greenPin, LOW);   // Zet groen uit
    digitalWrite(redPin, HIGH);     // Zet rood aan
    digitalWrite(bluePin, LOW);     // Zet blauw uit
    Serial.println("rood is aan"); // Correcte functie voor printen
} else if (receivedValue == 3) {
    digitalWrite(greenPin, LOW);    // Zet groen uit
    digitalWrite(redPin, LOW);      // Zet rood uit
    digitalWrite(bluePin, HIGH);    // Zet blauw aan
    Serial.println("blauw is aan");  // Correcte functie voor printen
}

    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
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

  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
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
    delay(3000);
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
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}
