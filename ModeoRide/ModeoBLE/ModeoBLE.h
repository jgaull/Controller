/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

#ifndef ModeoBLE_h
#define ModeoBLE_h

#include "Arduino.h"

#define CAN_READ_BLE_READ 0
#define CAN_READ_WRITE_BLE_READ 1
#define CAN_READ_BLE_BLE_READ_WRITE 2
#define CAN_READ_WRITE_BLE_READ_WRITE 3

class ModeoBLE {
    
    public:
    
    ModeoBLE(int numProperties);
    
    void startup();
    void shutdown();
    void update();
    
    void registerProperty(byte identifier, bool eepromSave);
    void registerPropertyWithCallback(byte identifier, byte readWritePermissions, void (*callback)(unsigned short, unsigned short));
    void setValueForProperty(unsigned short value, byte propertyId);
    unsigned short getValueForProperty(byte identifier);
    
    void clearEEPROM();
    void saveValueForProperty(unsigned short value, byte identifier);
    
    private:
    
    int indexForProperty(byte identifier);
    
    void performBluetoothReceive();
    void performConnect();
    void performDisconnect();
    void getPropertyValue();
    void writeGetProperty();
    void clearBLEBuffer();
    void setPropertyValue();
    void writeProperty();
    
    void retrieveCalibrations();
    void storeCalibrations();
    
};

#endif