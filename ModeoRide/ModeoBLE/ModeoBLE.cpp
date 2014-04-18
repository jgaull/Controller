/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

//Sensors
#define SENSOR_RIDER_EFFORT 0
#define SENSOR_CURRENT_STRAIN 1
#define SENSOR_SPEED 2
#define SENSOR_RAW_STRAIN 3
#define SENSOR_TORQUE_APPLIED 4
#define SENSOR_MOTOR_TEMP 5
#define SENSOR_BATTERY_VOLTAGE 6
#define SENSOR_FILTERED_RIDER_EFFORT 7

#define NUM_SENSORS 8 //Is there a better way to do this?

//Request Identifiers
#define REQUEST_CONNECT 10
#define REQUEST_DISCONNECT 11
#define REQUEST_GET_PROPERTY_VALUE 12
#define REQUEST_SET_PROPERTY_VALUE 13
#define REQUEST_ADD_BEZIER 14
#define REQUEST_GET_SENSOR_VALUE 15
#define REQUEST_WRITE_PROPERTY 16
#define REQUEST_WRITE_BEZIER 17
#define REQUEST_WRITE_GET_PROPERTY 18

#include <AltSoftSerial.h>
#include <EEPROM.h>

#include "Arduino.h"
#include "ModeoBLE.h"

struct Property {
    unsigned short value;
    byte identifier;
    short readWritePermissions;
    bool pendingSave;
    bool eepromSave;
    void (*callback)(unsigned short, unsigned short);
} ;

Property _properties[25];
byte _propertiesLength = 0;
AltSoftSerial _bleMini;

Property _propertyPendingSave;
byte _propertyIdentifierForPropertyPendingSave;

byte _numProperties = 0;
byte _lastAvailable = 0;

ModeoBLE::ModeoBLE(int numProperties) {
    Serial.println("ModeoBLE Constructed.");
    
    _numProperties = numProperties;
    
    _bleMini.begin(57600);
}

void ModeoBLE::startup() {
    Serial.println("ModeoBLE Startup!");
    
    if (_numProperties == _propertiesLength) {
        retrieveCalibrations();
    }
    else {
        Serial.println("Not enough properties registered to start BLE.");
    }
}

void ModeoBLE::shutdown() {
    Serial.println("ModeoBLE Shutdown!");
}

void ModeoBLE::update() {
    //Serial.println("ModeoBLE Update");
    performBluetoothReceive();
}

void ModeoBLE::registerProperty(byte identifier, bool eepromSave) {
    
    if (indexForProperty(identifier) == -1) {
        _properties[_propertiesLength].identifier = identifier;
        _properties[_propertiesLength].readWritePermissions = 0;
        _properties[_propertiesLength].pendingSave = false;
        _properties[_propertiesLength].eepromSave = eepromSave;
        
        _propertiesLength++;
        
        Serial.println("Property Registered.");
    }
    else {
        Serial.println("Property has already been registered");
    }
}

/*
void ModeoBLE::registerPropertyWithCallback(byte identifier, byte readWritePermissions, void (*callback)(unsigned short, unsigned short)) {
    
}*/

void ModeoBLE::setValueForProperty(unsigned short value, byte identifier) {
    Serial.println("Set Value");
    
    int index = indexForProperty(identifier);
    if (index != -1) {
        
        if (_properties[index].value != value) {
            _properties[index].value = value;
            _properties[index].pendingSave = true;
        }
    }
}


unsigned short ModeoBLE::getValueForProperty(byte identifier) {
    Serial.println("Get Value");
    
    int index = indexForProperty(identifier);
    if (index != -1) {
        return _properties[index].value;
    }
}

void ModeoBLE::clearEEPROM() {
    Serial.println("Clear EEPROM");
}

void ModeoBLE::saveValueForProperty(unsigned short value, byte identifier) {
    int index = indexForProperty(identifier);
    if (index != -1) {
        bool eepromSave = _properties[index].eepromSave;
        
        _properties[index].value = value;
        _properties[index].eepromSave = true;
        _properties[index].pendingSave = true;
        
        storeCalibrations();
        
        _properties[index].eepromSave = eepromSave;
    }
}

int ModeoBLE::indexForProperty(byte identifier) {
    for (byte i = 0; i < _propertiesLength; i++) {
        if (_properties[i].identifier == identifier) {
            return i;
        }
    }
    
    return -1;
}

void ModeoBLE::performBluetoothReceive() {
    byte currentlyAvailable = _bleMini.available();
    
    if ( currentlyAvailable > 0 && currentlyAvailable == _lastAvailable ) {
        byte identifier = _bleMini.read();
        
        //Serial.print("identifier: ");
        //Serial.println(identifier);
        
        switch(identifier) {
            case REQUEST_CONNECT:
                Serial.println("connect");
                performConnect();
                break;
                
            case REQUEST_DISCONNECT:
                Serial.println("disconnect");
                performDisconnect();
                break;
                
            case REQUEST_GET_PROPERTY_VALUE:
                Serial.println("get property");
                getPropertyValue();
                break;
                
            case REQUEST_SET_PROPERTY_VALUE:
                Serial.println("set property");
                setPropertyValue();
                break;
                
            case REQUEST_ADD_BEZIER:
                Serial.println("bezier");
                //addBezier();
                break;
                
            case REQUEST_GET_SENSOR_VALUE:
                Serial.println("get sensor value");
                //getSensorValue();
                break;
                
            case REQUEST_WRITE_PROPERTY:
                Serial.println("write property");
                writeProperty();
                break;
                
            case REQUEST_WRITE_BEZIER:
                Serial.println("write bezier");
                //writeBezier();
                break;
                
            case REQUEST_WRITE_GET_PROPERTY:
                Serial.println("write get property");
                writeGetProperty();
                break;
                
            default:
                Serial.print("Uknown command: ");
                Serial.println(identifier);
        }
        
        clearBLEBuffer();
        _lastAvailable = 0;
    }
    else {
        _lastAvailable = currentlyAvailable;
    }
}

void ModeoBLE::performConnect() {
    //stopSensorUpdates();
    _bleMini.write((byte)REQUEST_CONNECT);
    _bleMini.write(1);
}

void ModeoBLE::performDisconnect() {
    //storeCalibrations();
    //stopSensorUpdates();
    _bleMini.write(REQUEST_DISCONNECT);
    _bleMini.write(1);
}

void ModeoBLE::getPropertyValue() {
    if ( _bleMini.available() >= 1) {
        byte propertyIdentifier = _bleMini.read();
        
        if ( propertyIdentifier < _numProperties) {
            _bleMini.write(REQUEST_GET_PROPERTY_VALUE);
            _bleMini.write(propertyIdentifier);
            _bleMini.write(_properties[propertyIdentifier].value);
            _bleMini.write(_properties[propertyIdentifier].value >> 8);
        }
        else {
            clearBLEBuffer();
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::writeGetProperty() {
    if ( _bleMini.available() >= 3 ) {
        byte propertyIdentifier = _bleMini.read();
        byte data1 = _bleMini.read();
        byte data2 = _bleMini.read();
        unsigned short value = (data2 << 8) + data1;
        
        if ( _properties[propertyIdentifier].value == value ) {
            _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
            _bleMini.write(1);
        }
        else {
            _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
            _bleMini.write((byte)0);
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::setPropertyValue() {
    if ( _bleMini.available() >= 3) {
        
        byte propertyIdentifier = _bleMini.read();
        byte data1 = _bleMini.read();
        byte data2 = _bleMini.read();
        unsigned short value = (data2 << 8) + data1;
        
        Property newProperty;
        
        newProperty.eepromSave = _properties[propertyIdentifier].eepromSave;
        newProperty.value = _properties[propertyIdentifier].value;
        newProperty.pendingSave = _properties[propertyIdentifier].pendingSave;
        
        newProperty.value = value;
        
        if (_properties[propertyIdentifier].value != newProperty.value) {
            newProperty.pendingSave = true;
        }
        
        _propertyPendingSave = newProperty;
        _propertyIdentifierForPropertyPendingSave = propertyIdentifier;
        
        _bleMini.write(REQUEST_SET_PROPERTY_VALUE);
        _bleMini.write(propertyIdentifier);
        _bleMini.write(data1);
        _bleMini.write(data2);
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::writeProperty() {
    if ( _bleMini.available() >= 1 ) {
        byte propertyIdentifier = _bleMini.read();
        
        boolean success = false;
        if ( _propertyIdentifierForPropertyPendingSave == propertyIdentifier) {
            _properties[propertyIdentifier].value = _propertyPendingSave.value;
            _properties[propertyIdentifier].pendingSave = _propertyPendingSave.pendingSave;
            success = true;
        }
        
        _bleMini.write(REQUEST_WRITE_PROPERTY);
        _bleMini.write(success);
    }
    else {
        clearBLEBuffer();
    }
}

/*
void getSensorValue() {
    if (_bleMini.available() >= 1) {
        
        byte sensorIdentifier = _bleMini.read();
        
        unsigned short value = sensors[sensorIdentifier].value;
        
        _bleMini.write(REQUEST_GET_SENSOR_VALUE);
        _bleMini.write(sensorIdentifier);
        _bleMini.write(value);
        _bleMini.write(value >> 8);
    }
    else {
        clearBLEBuffer();
    }
}

void addBezier() {
    Bezier bezier;
    byte headerSize = 4;
    
    if ( _bleMini.available() >= headerSize ) {
        byte header[headerSize];
        
        for (int i = 0; i < headerSize; i++) {
            header[i] = _bleMini.read();
        }
        
        bezier.type = header[0];
        bezier.maxX = header[1];
        bezier.maxY = header[2];
        bezier.numPoints = header[3];
        bezier.cacheIsValid = false;
        
        byte bodySize = header[3] * 2;
        byte body[bodySize];
        
        if (_bleMini.available() >= bodySize) {
            
            for (byte i = 0; i < bodySize; i += 2) {
                byte pointX = _bleMini.read();
                byte pointY = _bleMini.read();
                
                body[i] = pointX;
                body[i + 1] = pointY;
                
                bezier.points[i / 2].x = pointX;
                bezier.points[i / 2].y = pointY;
            }
            
            bezierPendingSave = bezier;
            
            byte messageData[headerSize + bodySize];
            for (byte i = 0; i < headerSize; i++) {
                messageData[i] = header[i];
            }
            
            for (byte i = 0; i < bodySize; i++) {
                messageData[i + headerSize] = body[i];
            }
            
            BLEMini.write(REQUEST_ADD_BEZIER);
            for (byte i = 0; i < headerSize + bodySize; i++) {
                _bleMini.write(messageData[i]);
            }
        }
        else {
            /*
             Serial.print("Needs ");
             Serial.print(bezier.numPoints * 2);
             Serial.print(" bytes for body. Has ");
             Serial.print(BLEMini.available());
             Serial.println(" bytes.");
             //
            clearBLEBuffer();
        }
    }
    else {
        //Serial.print("not enough bytes for header: ");
        //Serial.println(BLEMini.available());
        clearBLEBuffer();
    }
}

void writeBezier() {
    
    if (_bleMini.available() >= 1) {
        byte bezierType = _bleMini.read();
        
        boolean success = false;
        if (bezierType == bezierPendingSave.type) {
            
            switch(bezierPendingSave.type) {
                    
                case CURVE_TYPE_ASSIST:
                    assist = bezierPendingSave;
                    break;
                    
                case CURVE_TYPE_DAMPING:
                    damping = bezierPendingSave;
                    break;
                    
                case CURVE_TYPE_REGEN:
                    regen = bezierPendingSave;
                    break;
                    
                    //default:
                    //Serial.println("Unrecognized Curve Identifier");
            }
            
            success = true;
            
            _bleMini.write(REQUEST_WRITE_BEZIER);
            _bleMini.write(success);
        }
        else {
            //Serial.println("bezier type did not match. flail.");
        }
    }
    else {
        clearBLEBuffer();
    }
}
*/

void ModeoBLE::clearBLEBuffer() {
    while(_bleMini.available() > 0) {
        _bleMini.read();
    }
}


//EEPROM Shit
void ModeoBLE::retrieveCalibrations() {
    
    //Serial.println("Started property restore");
    
    for (byte i = 0; i < _numProperties; i++) {
        
        if (_properties[i].eepromSave) {
            int lsb = i * 2 + 1;
            int msb = i * 2;
            _properties[i].value = (EEPROM.read(msb) << 8) + EEPROM.read(lsb);
        }
    }
    
    /*
     Serial.print("Restored ");
     Serial.print(propertyCount);
     Serial.println(" properties.");
     */
}


void ModeoBLE::storeCalibrations() {
    //Serial.println("Started property save");
    
    byte propertyCount = 0;
    for (byte i = 0; i < _numProperties; i++) {
        
        if (_properties[i].eepromSave && _properties[i].pendingSave) {
            int lsb = i * 2 + 1;
            int msb = i * 2;
            EEPROM.write(msb, _properties[i].value >> 8);
            EEPROM.write(lsb, _properties[i].value);
            
            propertyCount++;
        }
    }
    
    /*
     Serial.print("Saved ");
     Serial.print(propertyCount);
     Serial.println(" properties.");
     //*/
}