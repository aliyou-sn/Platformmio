#include <Arduino.h>

#include <Wire.h>
#include <WiFi.h>
// #include "SparkFunLSM6DS3.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ArduinoNvs.h"

#define VERSION "0.0.1"
#define USB_SER  Serial
#define MUX_RESET_PIN   8

#define PCA9548_A       5
#define PCA9548_B       4  //18
#define PCA9548_C       3  //19

//#define LSM6D_SDO       6
//#define LSM6D_CS        7
//#define LSM6D_INT1      10
//#define LSM6D_SDO_HIGH
//
//#ifndef LSM6D_SDO_HIGH
//#define LSM6D_I2C_ADDR  0x6A
//#else
//#define LSM6D_I2C_ADDR 0x6B
//#endif
//
//LSM6DS3 lsm6d ( I2C_MODE, LSM6D_I2C_ADDR );

#define BOARD_SDA       1 //8 
#define BOARD_SCL       0  //9

uint8_t sensor_addresses[] = {0x30, 0x35};


#define WIFI_SSID_NVS_KEY "WIFI_SSID"
#define WIFI_PASS_NVS_KEY "WIFI_PASS"

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
uint8_t cmd[255];
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

WiFiServer server(8080);
WiFiClient client;

int PCAselectChannel(uint8_t channel) {
  uint8_t a1, b1, c1;
  int ret = 0;
  switch (channel) {
    case 0: {
        a1 = 0;
        b1 = 0;
        c1 = 0;
        break;
      }
    case 1: {
        a1 = 1;
        b1 = 0;
        c1 = 0;
        break;
      }
    case 2: {
        a1 = 0;
        b1 = 1;
        c1 = 0;
        break;
      }
    case 3: {
        a1 = 1;
        b1 = 1;
        c1 = 0;
        break;
      }
    case 4: {
        a1 = 0;
        b1 = 0;
        c1 = 1;
        break;
      }
    case 5: {
        a1 = 1;
        b1 = 0;
        c1 = 1;
        break;
      }
    case 6: {
        a1 = 0;
        b1 = 1;
        c1 = 1;
        break;
      }
    case 7: {
        a1 = 1;
        b1 = 1;
        c1 = 1;
        break;
      }
  }

  digitalWrite(PCA9548_A, a1);
  digitalWrite(PCA9548_B, b1);
  digitalWrite(PCA9548_C, c1);

  uint8_t base_addr = 0x70;
  uint8_t pca9548addr = (a1 << 0) | (b1 << 1) | (c1 << 2) | base_addr;
  if (channel > 7 && channel < 0) {
    return -1;
  }

  Wire.beginTransmission(pca9548addr);
  uint8_t channel_byte = (1 << channel);
  if (Wire.write(channel_byte) != 0) {
    ret = 0;
  } else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
}

int count_no_of_sensors() {
  int count = 0;
  for (uint8_t channel = 0; channel < 8; channel++) {
    if (PCAselectChannel(channel) == 0) {
      for (uint8_t sensor = 0; sensor < 2; sensor++) {
        Wire.beginTransmission(sensor_addresses[sensor]);
        if (Wire.endTransmission() == 0) {
          count++;
        }
      }
    }
  }
  return count;
}

uint8_t MMC5633_Get_PID(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t reg_addr = 0x39;
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  uint8_t buff[1];
  Wire.readBytes(buff, 1);
  if (Wire.endTransmission() == 0) {
    return buff[0];
  }
  return 0;
}

int MMC5633_write_reg(uint8_t address, uint8_t reg_addr, uint8_t value)
{
  int ret = -1;
  uint8_t reg_data[2] = {reg_addr, value};
  Wire.beginTransmission(address);
  Wire.write(reg_data, 2);
  int c = Wire.endTransmission();
  if (c == 0) {
    ret = 0;
  } else {
    ret = -1;
  }
  return ret;
}

uint8_t MMC_read_status_reg(uint8_t address) {
  uint8_t status = 0x00;
  Wire.beginTransmission(address);
  uint8_t reg_addr =  0x18;
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  Wire.readBytes(&status, 1);
  Wire.endTransmission();
  return status;
}

void MMC_read_register(uint8_t address, uint8_t reg_addr, uint8_t * buf, uint8_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(address, len);
  Wire.readBytes(buf, len);
  Wire.endTransmission();
}

void ReadSensors() {
  USB_SER.println("CHANNEL,SENSOR_ADDR,PRODUCT_ID,X,Y,Z\r\n");
  for (uint8_t channel = 0; channel < 4; channel++) {//for (uint8_t channel = 0; channel < 8; channel++) {
    if (PCAselectChannel(channel) == 0) {
      for (uint8_t sensor = 0; sensor < 1; sensor++) { //for (uint8_t sensor = 0; sensor < 2; sensor++) {
        uint8_t PID = MMC5633_Get_PID(sensor_addresses[sensor]);
        delay(1);
        if (MMC5633_write_reg(sensor_addresses[sensor], 0x1B, 0x21) == -1) {
          USB_SER.printf("%d,0x%02X,%d,NA,NA,NA\r\n", \
                         channel, sensor_addresses[sensor], \
                         PID);
          continue;
        }
        delay(1);
        while ((MMC_read_status_reg(sensor_addresses[sensor]) & 0x40) != 0x40);
        uint8_t buf[9];
        MMC_read_register(sensor_addresses[sensor], 0x00, buf, 9);
        //SysTick_DelayTicks(200);
        int16_t Xout = (int16_t)((buf[0] << 12) | (buf[1] << 4) | (buf[6]));
        int16_t Yout = (int16_t)((buf[2] << 12) | (buf[3] << 4) | (buf[7]));
        int16_t Zout = (int16_t)((buf[4] << 12) | (buf[5] << 4) | (buf[8]));
        USB_SER.printf("%d,0x%02X,%d,%d,%d,%d\r\n", \
                       channel, sensor_addresses[sensor], \
                       PID, Xout, Yout, Zout);
        delay(1);
      }
    } else {
      USB_SER.printf("------Mux not responded------\r\n");
    }
  }
}
  /*uint8_t AccelId = 0;
  lsm6d.readRegister(&AccelId, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

  if (AccelId != 0) {
    //Accel data
    int16_t xl_x = lsm6d.readRawAccelX();
    int16_t xl_y = lsm6d.readRawAccelY();
    int16_t xl_z = lsm6d.readRawAccelZ();
    USB_SER.printf("LSM6D Accel,0x%02X,%d,%d,%d,%d\r\n", \
                   LSM6D_I2C_ADDR, \
                   AccelId, xl_x, xl_y, xl_z);
    //Gyro data
    int16_t g_x = lsm6d.readRawGyroX();
    int16_t g_y = lsm6d.readRawGyroY();
    int16_t g_z = lsm6d.readRawGyroZ();
    USB_SER.printf("LSM6D Gyro,0x%02X,%d,%d,%d,%d\r\n", \
                   LSM6D_I2C_ADDR, \
                   AccelId, g_x, g_y, g_z);
  } else {
    USB_SER.printf("LSM6D Accel,0x%02X,%d,NA,NA,NA\r\n", \
                   LSM6D_I2C_ADDR, \
                   AccelId);
    USB_SER.printf("LSM6D Gyro,0x%02X,%d,NA,NA,NA\r\n", \
                   LSM6D_I2C_ADDR, \
                   AccelId);
  }*/
 // USB_SER.printf("LSM6D Accel,0,0,NA,NA,NA\r\n");
 // USB_SER.printf("LSM6D Gyro,0,0,NA,NA,NA\r\n");
//}


void Ble_write_str(char * str) {
  pTxCharacteristic->setValue((uint8_t *)str, strlen(str));
  pTxCharacteristic->notify();
}

#define TCP_PRINT 0
#define BLE_PRINT 1
#define SERIAL_PRINT 2
void printToStream(uint8_t stream, char * message) {
  switch (stream) {
    case TCP_PRINT: {
        client.print(message);
      }
      break;
    case BLE_PRINT: {
        Ble_write_str(message);
        delay(50);
      }
      break;
    case SERIAL_PRINT: {
        USB_SER.print(message);
      }
      break;
  }
}


void ReadSensorsPrintToStream(uint8_t stream) {
  char dbg_msg[100];
  printToStream(stream, "CHANNEL,SENSOR_ADDR,PRODUCT_ID,X,Y,Z\r\n");
  for (uint8_t channel = 0; channel < 8; channel++) {
    if (PCAselectChannel(channel) == 0) {
      for (uint8_t sensor = 0; sensor < 2; sensor++) {
        uint8_t PID = MMC5633_Get_PID(sensor_addresses[sensor]);
        delay(1);
        if (MMC5633_write_reg(sensor_addresses[sensor], 0x1B, 0x21) == -1) {
          sprintf(dbg_msg, "%d,0x%02X,%d,NA,NA,NA\r\n", \
                  channel, sensor_addresses[sensor], \
                  PID);
          printToStream(stream, dbg_msg);
          continue;
        }

        delay(1);
        while ((MMC_read_status_reg(sensor_addresses[sensor]) & 0x40) != 0x40);
        uint8_t buf[9];
        MMC_read_register(sensor_addresses[sensor], 0x00, buf, 9);
        //SysTick_DelayTicks(200);
        int16_t Xout = (int16_t)((buf[0] << 12) | (buf[1] << 4) | (buf[6]));
        int16_t Yout = (int16_t)((buf[2] << 12) | (buf[3] << 4) | (buf[7]));
        int16_t Zout = (int16_t)((buf[4] << 12) | (buf[5] << 4) | (buf[8]));
        sprintf(dbg_msg, "%d,0x%02X,%d,%d,%d,%d\r\n", \
                channel, sensor_addresses[sensor], \
                PID, Xout, Yout, Zout);
        printToStream(stream, dbg_msg);
        delay(1);
      }
    } else {
      printToStream(stream, "------Mux not responded------\r\n");
    }
  }
}
 //*uint8_t AccelId = 0;
 // lsm6d.readRegister(&AccelId, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

  
//
//  USB_SER.printf("LSM6D Accel,0,0,NA,NA,NA\r\n");
//  USB_SER.printf("LSM6D Gyro,0,0,NA,NA,NA\r\n");
//}

void saveWiFi(String ssid, String password){
   NVS.setString(WIFI_SSID_NVS_KEY , ssid);
   NVS.setString(WIFI_PASS_NVS_KEY , password);
}
unsigned long scheduleReset = 0;
void parseCommandAndStreamOutput(uint8_t *cmd, uint8_t stream) {
  char dbg_msg[100];
  if (cmd[0] == 'V') {
    printToStream(stream, VERSION);
  } else if (cmd[0] == 'T') {
    int c = count_no_of_sensors();
    sprintf(dbg_msg, "Sensors found : %d\r\n", c);
    printToStream(stream, dbg_msg);
  } else if (cmd[0] == 'M') {
    ReadSensorsPrintToStream(stream);
  } else if (cmd[0] == 'W') {
    uint8_t mux_idx = cmd[1];
    uint8_t channel = cmd[2];
    uint8_t sensor_addr = cmd[3];
    uint8_t reg = cmd[4];
    uint8_t val = cmd[5];
    if (channel >= 0 && channel <= 7) {
      if (sensor_addr == 0x35 || sensor_addr == 0x30) {
        //WriteMagneticSensorRegister(mux_idx, channel, sensor_idx, reg, val);
        if (PCAselectChannel(channel) == 0) {
          if (MMC5633_write_reg(sensor_addr, reg, val ) == 0) {
            printToStream(stream, "Reg Write Success\r\n");
          } else {
            printToStream(stream, "Reg Write Failed\r\n");
          }
        } else {
          printToStream(stream, "Reg Write Failed\r\n");
        }
      } else {
        printToStream(stream, "Invalid sensor_addr to Write\r\n");
      }
    } else {
      printToStream(stream, "Invalid Mux channel to Write\r\n");
    }
  } else if (cmd[0] == 'R') {
    uint8_t mux_idx = cmd[1];
    uint8_t channel = cmd[2];
    uint8_t sensor_addr = cmd[3];
    uint8_t reg = cmd[4];
    uint8_t val = 0;
    if (channel >= 0 && channel <= 7) {
      if (sensor_addr == 0x35 || sensor_addr == 0x30) {
        //WriteMagneticSensorRegister(mux_idx, channel, sensor_idx, reg, val);
        if (PCAselectChannel(channel) == 0) {
          MMC_read_register(sensor_addr, reg, &val, 1);
          sprintf(dbg_msg, "Reg 0x%02X - val 0x%02X\r\n", reg, val);
          printToStream(stream, dbg_msg);
        } else {
          printToStream(stream, "Reg read Failed\r\n");
        }
      } else {
        printToStream(stream, "Invalid sensor_addr to read\r\n");
      }
    } else {
      printToStream(stream, "Invalid Mux channel to read\r\n");
    }
  }else if (cmd[0] == 'C') {
    uint8_t ssid_len = cmd[1];
    char ssid_str[ssid_len+1];
    memcpy(ssid_str,&cmd[2], ssid_len);
    ssid_str[ssid_len] = '\0';
    printToStream(stream, ssid_str);
    printToStream(stream, "\r\n");
    uint8_t pass_len_cursor = 1+ssid_len+1;
    uint8_t pass_len = cmd[pass_len_cursor];
    char pass_str[pass_len+1];
    memcpy(pass_str,&cmd[pass_len_cursor+1], pass_len);
    pass_str[pass_len] = '\0';
    printToStream(stream, pass_str);
    printToStream(stream, "\r\n");
    saveWiFi(String(ssid_str), String(pass_str));
    printToStream(stream, "WiFi Settings saved. \r\n Restarting....\r\n");
    scheduleReset = millis();
  }else if (cmd[0] == 'I') {
    String ip = WiFi.localIP().toString();
    sprintf(dbg_msg, "Ip Address: %s", ip.c_str());
    printToStream(stream, dbg_msg);
  }
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->startAdvertising();
    }
};

uint8_t ble_command_pending = 0;
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        for(int i = 0; i < rxValue.length(); i++){
          cmd[i] = rxValue[i];
        }

        ble_command_pending = 1;
      }
    }
};

void setup() {
  USB_SER.begin(115200);
  NVS.begin();
  Wire.setPins(BOARD_SDA, BOARD_SCL);
  Wire.begin();
  Wire.setClock(400000);//Sets clock at specific rate 400khz
  pinMode(MUX_RESET_PIN, OUTPUT);
  digitalWrite(MUX_RESET_PIN, LOW); //Disable the Mux
//  pinMode(2, OUTPUT);
//  pinMode(4, OUTPUT);
//  digitalWrite(2, HIGH);
//  digitalWrite(4, HIGH);
  pinMode(PCA9548_A, OUTPUT);
  pinMode(PCA9548_B, OUTPUT);
  pinMode(PCA9548_C, OUTPUT);
 //   pinMode(LSM6D_SDO, OUTPUT);
  
//  #ifndef LSM6D_SDO_HIGH
//    digitalWrite(LSM6D_SDO, LOW);
//  #else
//    digitalWrite(LSM6D_SDO, HIGH);
//  #endif
//
//  if ( lsm6d.begin() != IMU_SUCCESS )
//  {
//    USB_SER.print("LSM6D not initialized\n");
//  }
//  else
//  {
//    USB_SER.print("LSM6D initialized\n");
//  }

  

  String ssid = NVS.getString(WIFI_SSID_NVS_KEY);
  String password = NVS.getString(WIFI_PASS_NVS_KEY);
  if((ssid != "") && (password != "")){
    USB_SER.print("Connecting to ");
    USB_SER.println(ssid);
    WiFi.begin(ssid.c_str(), password.c_str());

    int retry = 20;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      retry--;
      if(retry == 0){
        break;
      }
      USB_SER.print(".");
    }
    if(WiFi.status() == WL_CONNECTED){
      USB_SER.println("");
      USB_SER.println("WiFi connected.");
      USB_SER.println("IP address: ");
      USB_SER.println(WiFi.localIP());
    }else{
      USB_SER.println("");
      USB_SER.println("Could not connect to wifi");
    }
  }
  // Create the BLE Device
  BLEDevice::init("ESP32-SENSORS");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
                                          );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  if(WiFi.status() == WL_CONNECTED){
    server.begin();
  }
}

//
int i = 0;
void loop() {
  if((scheduleReset > 0) && ((millis() - scheduleReset) > 3000) ){
    ESP.restart();
  }
  if(WiFi.status() == WL_CONNECTED){
    if (!client.connected()) { //if a client disconnected then only wait for new client
      client = server.available();
    }
  
    if (client) {
      //    while (client.connected()) {
      i = 0;
      if (client.available() > 0) {
        while (client.available()) {
          cmd[i] = client.read();
          i++;
        }
  
        parseCommandAndStreamOutput(cmd, TCP_PRINT);
      }
      //    }
    }
  }
  // put your main code here, to run repeatedly:
  if (USB_SER.available() > 0) {
    i = 0;
    while (USB_SER.available()) {
      cmd[i] = Serial.read();
      i++;
    }
    parseCommandAndStreamOutput(cmd, SERIAL_PRINT);
  }

  if(ble_command_pending){
    ble_command_pending = 0;
    uint8_t invalid = 0;
    if(cmd[0] == 'T'){
      USB_SER.println("T received from BLE");
    }else if(cmd[0] == 'M'){
      USB_SER.println("M received from BLE");
    }else if(cmd[0] == 'V'){
      USB_SER.println("V received from BLE");
    }else if(cmd[0] == 'W'){
      USB_SER.println("W received from BLE");
    }else if(cmd[0] == 'R'){
      USB_SER.println("R received from BLE");
    }else if(cmd[0] == 'C'){
      USB_SER.println("C received from BLE");
    }else if(cmd[0] == 'I'){
      USB_SER.println("I received from BLE");
    }else{
      invalid = 1;
    }

    if(!invalid){
      parseCommandAndStreamOutput(cmd, BLE_PRINT);
    }else{
      USB_SER.println("Invalid received from BLE");
      USB_SER.println(cmd[0]);
    }
  }
}
