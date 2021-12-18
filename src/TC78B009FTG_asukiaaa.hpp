#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <string_asukiaaa.h>
#include <wire_asukiaaa.h>

#include "./TC78B009FTG_asukiaaa/Registers.hpp"

#define TC78B009FTG_ASUKIAAA_VERSION_MAJOR 1
#define TC78B009FTG_ASUKIAAA_VERSION_MINOR 0
#define TC78B009FTG_ASUKIAAA_VERSION_PATCH 0

namespace TC78B009FTG_asukiaaa {

namespace DeviceAddresses {
enum Values {
  ID1HighID2Low = 0b0101001,
  ID1LowID2High = 0b0101101,
  ID1HighID2High = 0b0110010,
  InitialOnRegister = ID1HighID2Low,
};
};  // namespace DeviceAddresses

// enum class ControlMode {
//   Stop,
//   Rpm,
//   Duty,
// };

class CoreI2c {
 public:
  CoreI2c(uint8_t deviceAddress, int pinStandby = -1)
      : deviceAddress(deviceAddress), pinStandby(pinStandby) {}

  void setWire(TwoWire* wire) { this->wire = wire; }

  void begin() {
    if (wire == NULL) {
      wire = &Wire;
      wire->begin();
    }
    if (pinStandby >= 0) {
      pinMode(pinStandby, OUTPUT);
      digitalWrite(pinStandby, HIGH);
      delay(100);
    }
  }

  // int writeDefaultConfig() {
  //   // default config of blushless_7_click
  //   static const uint8_t defaultConfigData[] = {
  //       // 0x00, 0x00,
  //       0x0F, 0x33, 0x00, 0xCC, 0x33, 0x08, 0x12, 0x00, 0x00, 0x00, 0x7F,
  //       0x7F, 0x37, 0x11, 0x10, 0xC3, 0x01, 0x02, 0xA2, 0x0C, 0x82, 0x3F,
  //       0x40,
  //   };
  //   static const uint8_t defaultConfigLen = sizeof(defaultConfigData);
  //   return wire_asukiaaa::writeBytes(wire, deviceAddress, 2,
  //   defaultConfigData,
  //                                    defaultConfigLen);
  // }

  void writeConfigExampleTable8p47OpenLoop() {
    Registers::R03 r03;
    r03.startDuty = 102;  // 0.2
    writeRegister(r03);
    Registers::R02 r02;
    r02.stopDuty = 46;  // 0.18
    writeRegister(r02);
    Registers::R05 r05;
    r05.maxDuty = 204;  // 0.9
    writeRegister(r05);
    Registers::R06to07 r06to07;
    readRegister(&r06to07);
    r06to07.startRpm = 51;  // 0.1
    writeRegister(r06to07);
    Registers::R08to09 r08to09;
    readRegister(&r08to09);
    r08to09.speedSlop = 1241;  // 0.95
    writeRegister(r08to09);
  }

  void printRegisters(Stream* serial) {
    for (int i = 0; i < 30; ++i) {
      Registers::Base* reg = NULL;
      switch (i) {
        case 0:
          reg = new Registers::R00();
          break;
        case 1:
          reg = new Registers::R01();
          break;
        case 2:
          reg = new Registers::R02();
          break;
        case 3:
          reg = new Registers::R03();
          break;
        case 4:
          reg = new Registers::R04();
          break;
        case 5:
          reg = new Registers::R05();
          break;
        case 6:
          reg = new Registers::R06to07();
          break;
        case 8:
          reg = new Registers::R08to09();
          break;
        case 10:
          reg = new Registers::R10to11();
          break;
        case 12:
          reg = new Registers::R12();
          break;
        case 13:
          reg = new Registers::R13();
          break;
        case 14:
          reg = new Registers::R14();
          break;
        case 15:
          reg = new Registers::R15();
          break;
        case 16:
          reg = new Registers::R16();
          break;
        case 17:
          reg = new Registers::R17();
          break;
        case 18:
          reg = new Registers::R18();
          break;
        case 19:
          reg = new Registers::R19();
          break;
        case 20:
          reg = new Registers::R20();
          break;
        case 21:
          reg = new Registers::R21();
          break;
        case 22:
          reg = new Registers::R22();
          break;
        case 23:
          reg = new Registers::R23();
          break;
        case 24:
          reg = new Registers::R24();
          break;
        case 25:
          reg = new Registers::R25();
          break;
        case 27:
          reg = new Registers::R27to28();
          break;
        case 29:
          reg = new Registers::R29to30();
          break;
      }
      if (reg == NULL) {
        continue;
      }
      serial->print(
          "R" + string_asukiaaa::padNumStart(reg->startAddress, 2, '0') + ' ');
      auto result = readRegister(reg, serial);
      serial->print(' ');
      if (result != 0) {
        serial->println("Cannot read");
      } else {
        reg->print(serial);
        serial->println();
      }
      delete reg;
    }
  }

  int readRegister(Registers::Base* reg, Stream* serial = NULL) {
    uint8_t len = reg->length;
    uint8_t data[len];
    auto result = wire_asukiaaa::readBytes(wire, deviceAddress,
                                           reg->startAddress, data, len);
    if (result != 0) {
      return result;
    }
    if (serial != NULL) {
      for (int i = 0; i < len; ++i) {
        if (i != 0) {
          serial->print(' ');
        }
        serial->print(string_asukiaaa::padStart(String(data[i], BIN), 8, '0'));
      }
    }
    reg->parse(data, len);
    return result;
  }

  int writeRegister(Registers::Base& reg) {
    uint8_t len = reg.length;
    uint8_t data[len];
    reg.toBytes(data, len);
    return wire_asukiaaa::writeBytes(wire, deviceAddress, reg.startAddress,
                                     data, len);
  }

  // void writeByMode(ControlMode mode) {  // brushless7_control_mode_set
  //   Registers::R02 r02;
  //   readRegister(&r02);
  //   Registers::R08to09 r08;
  //   readRegister(&r08);
  //   Registers::R10to11 r10;
  //   readRegister(&r10);
  //   switch (mode) {
  //     case ControlMode::Rpm:  // BRUSHLESS7_CTRL_TYPE_RPM
  //       r02.noStop = true;
  //       r08.maxOpen = false;
  //       r08.maxOff = false;
  //       r10.openLoop = false;
  //       break;
  //     case ControlMode::Duty:  // BRUSHLESS7_CTRL_TYPE_DUTY
  //       r02.noStop = true;
  //       r08.maxOpen = false;
  //       r08.maxOff = true;
  //       r10.openLoop = true;
  //     default:  // BRUSHLESS7_CTRL_TYPE_STOP case ControlMode::Stop:
  //       r02.noStop = false;
  //       r08.maxOpen = false;
  //       r08.maxOff = false;
  //       break;
  //   }
  //   writeRegister(r02);
  //   writeRegister(r08);
  //   writeRegister(r10);
  // }

 private:
  TwoWire* wire = NULL;
  const uint8_t deviceAddress;
  const int pinStandby;
};

}  // namespace TC78B009FTG_asukiaaa
