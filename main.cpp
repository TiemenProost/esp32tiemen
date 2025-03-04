/**！
 * @file activityDetect.ino
 * @brief Motion detection, can detect whether the module is moving
 * @n It’s necessary to go into low power mode before using this function. Then call setActMode() to make the chip in sleep mode. 
 * @n In this state, the measurement rate is 12.5hz.
 * @n When the acceleration change in a certain direction is detected to exceed the threshold, the measurement rate will be increased 
 * @n to the normal rate we set before. The threshold can be set by the setWakeUpThreshold() function.
 * @n But if the move stops moving, also, the acceleration change in the three directions is less than the threshold, the chip will turn into sleep
 * @n mode after a period of time. This duration time can be set by the setWakeUpDur() function.
 * @n When using SPI, chip select pin can be modified by changing the value of LIS2DW12_CS.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2021-01-16
 * @url https://github.com/DFRobot/DFRobot_LIS
 *//**！
 * @file activityDetect.ino
 * @brief Motion detection, can detect whether the module is moving
 * @n It’s necessary to go into low power mode before using this function. Then call setActMode() to make the chip in sleep mode. 
 * @n In this state, the measurement rate is 12.5hz.
 * @n When the acceleration change in a certain direction is detected to exceed the threshold, the measurement rate will be increased 
 * @n to the normal rate we set before. The threshold can be set by the setWakeUpThreshold() function.
 * @n But if the move stops moving, also, the acceleration change in the three directions is less than the threshold, the chip will turn into sleep
 * @n mode after a period of time. This duration time can be set by the setWakeUpDur() function.
 * @n When using SPI, chip select pin can be modified by changing the value of LIS2DW12_CS.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2021-01-16
 * @url https://github.com/DFRobot/DFRobot_LIS
 */






 #include <DFRobot_LIS2DW12.h>
 #include <Wire.h>
 #include <Arduino.h>
 #include <ESP32Servo.h>
 #include <BLEDevice.h>
 #include <BLEServer.h>
 #include <BLEUtils.h>
 #include <BLE2902.h>
 
 int counter = 0;
 Servo myServo;  // Create a servo object
 
 // I2C communication
 DFRobot_LIS2DW12_I2C acce(&Wire, 0x18);
 
 // Bluetooth variables
 BLEServer* pServer = NULL;
 BLECharacteristic* pSensorCharacteristic = NULL;
 BLECharacteristic* pServoCharacteristic = NULL;
 bool deviceConnected = false;
 bool oldDeviceConnected = false;
 uint32_t value = 0;
 
 #define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
 #define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
 #define SERVO_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
 
 class MyServerCallbacks: public BLEServerCallbacks {
     void onConnect(BLEServer* pServer) {
         deviceConnected = true;
     };
 
     void onDisconnect(BLEServer* pServer) {
         deviceConnected = false;
     }
 };
 
 class MyServoCharacteristicCallbacks : public BLECharacteristicCallbacks {
     void onWrite(BLECharacteristic* pServoCharacteristic) {
         std::string servoValue = pServoCharacteristic->getValue(); 
         if (servoValue.length() > 0) {
             int angle = static_cast<int>(servoValue[0]);
             myServo.write(angle);  // Move the servo to the received angle
             Serial.print("Servo moved to: ");
             Serial.println(angle);
         }
     }
 };
 
 void setup(void) {
     Serial.begin(9600);
 
     // Initialize accelerometer
     while (!acce.begin()) {
         Serial.println("Communication failed, check the connection and I2C address.");
         delay(1000);
     }
     Serial.print("Chip ID: ");
     Serial.println(acce.getID(), HEX);
 
     // Accelerometer configuration
     acce.softReset();
     acce.setRange(DFRobot_LIS2DW12::e2_g);
     acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
     acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_10);
     acce.setWakeUpDur(2);
     acce.setWakeUpThreshold(0.2);
     acce.setPowerMode(DFRobot_LIS2DW12::eHighPerformanceLowNoise_14bit);
     acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
     acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
     acce.setDataRate(DFRobot_LIS2DW12::eRate_400hz);
 
     
     myServo.attach(D2);
 
     // Initialize Bluetooth
     BLEDevice::init("TIEMEN");
     pServer = BLEDevice::createServer();
     pServer->setCallbacks(new MyServerCallbacks());
 
     BLEService *pService = pServer->createService(SERVICE_UUID);
 
     pSensorCharacteristic = pService->createCharacteristic(
         SENSOR_CHARACTERISTIC_UUID,
         BLECharacteristic::PROPERTY_READ   |
         BLECharacteristic::PROPERTY_NOTIFY
     );
 
     pServoCharacteristic = pService->createCharacteristic(
         SERVO_CHARACTERISTIC_UUID,
         BLECharacteristic::PROPERTY_WRITE
     );
 
     pServoCharacteristic->setCallbacks(new MyServoCharacteristicCallbacks());
 
     pSensorCharacteristic->addDescriptor(new BLE2902());
     pServoCharacteristic->addDescriptor(new BLE2902());
 
     pService->start();
 
     BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
     pAdvertising->addServiceUUID(SERVICE_UUID);
     pAdvertising->setScanResponse(false);
     pAdvertising->setMinPreferred(0x0);  
     BLEDevice::startAdvertising();
     Serial.println("Waiting for a client connection to notify...");
 }
 
 void checkMotion() {
     if (acce.actDetected()) {
         Serial.println("Activity Detected!");
         Serial.print("x: ");
         Serial.print(acce.readAccX());
         Serial.print(" mg \t y: ");
         Serial.print(acce.readAccY());
         Serial.print(" mg \t z: ");
         Serial.print(acce.readAccZ());
         Serial.println(" mg");
 
         counter++;
         Serial.print("Counter: ");
         Serial.println(counter);
 
         if (deviceConnected) {
             String sensorData = "Motion Detected! Counter: " + String(counter);
             pSensorCharacteristic->setValue(sensorData.c_str());
             pSensorCharacteristic->notify();
         }
 
         
 
         delay(500);
     }
 }
 
 void rotateServo() {
     static unsigned long lastRotationTime = 0;
     static int angle = 0;
 
     if (millis() - lastRotationTime >= 10000) {  // 10 seconds have passed
         angle = (angle + 90) % 180;  // Increment angle by 90 degrees, wrap around at 180
         myServo.write(angle);  // Move the servo to the new angle
         lastRotationTime = millis();  // Update the last rotation time
         Serial.print("Servo rotated to: ");
         Serial.println(angle);
     }
 }
 
 void loop(void) {
     checkMotion();
     rotateServo();
 
     if (!deviceConnected && oldDeviceConnected) {
         delay(500); // give the bluetooth stack the chance to get things ready
         pServer->startAdvertising(); // restart advertising
         Serial.println("Start advertising");
         oldDeviceConnected = deviceConnected;
     }
     if (deviceConnected && !oldDeviceConnected) {
         oldDeviceConnected = deviceConnected;
     }
 }