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

// Definieer de pinnen voor de RGB LED
const int redPin = D1;    // Verander de pin als dat nodig is
const int greenPin = D2;  // Verander de pin als dat nodig is
const int bluePin = D3;   // Verander de pin als dat nodig is

// Zie de volgende voor het genereren van UUID's: https://www.uuidgenerator.net/
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
    std::string ledValue = pLedCharacteristic->getValue(); 
    
    if (ledValue.length() == 3) {  // Verwacht 3 bytes voor RGB
      int redValue = ledValue[0];
      int greenValue = ledValue[1];
      int blueValue = ledValue[2];

      // Print de waarden voor debugging
      Serial.print("Red: ");
      Serial.print(redValue);
      Serial.print(", Green: ");
      Serial.print(greenValue);
      Serial.print(", Blue: ");
      Serial.println(blueValue);

      // Zet de juiste helderheid op de RGB LED
      analogWrite(redPin, redValue);
      analogWrite(greenPin, greenValue);
      analogWrite(bluePin, blueValue);
    }
  }
};

void setup() {
  Serial.begin(115200);
  
  // Stel de pinnen in als OUTPUT
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Maak het BLE apparaat aan
  BLEDevice::init("TIEMEN");

  // Maak de BLE server aan
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Maak de BLE service aan
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Maak een BLE characteristic aan
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Maak de LED characteristic aan
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Registreer de callback voor de LED characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Voeg een descriptor toe aan de characteristics
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start de service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // Zet de waarde naar 0x00 om deze parameter niet te adverteren
  BLEDevice::startAdvertising();
  Serial.println("Wachten op een client verbinding om te notificeren...");
}

void loop() {
  // Notify veranderde waarde
  if (deviceConnected) {
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("Nieuwe waarde genotificeerd: ");
    Serial.println(value);
    delay(3000); // Vertraag om congestie van het bluetooth stack te voorkomen
  }

  // Bij disconnectie
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Apparaat losgekoppeld.");
    delay(500); // Geef het bluetooth stack de kans om zich klaar te maken
    pServer->startAdvertising(); // Start advertising opnieuw
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  // Bij connectie
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Apparaat verbonden.");
  }
}
