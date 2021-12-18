#pragma once
#include <string_asukiaaa.h>

#include "./ValueTypes.hpp"
#include "./utils.hpp"

namespace TC78B009FTG_asukiaaa {
namespace Registers {

class Base {
 public:
  const uint8_t startAddress;
  const uint8_t length;
  Base(uint8_t startAddress, uint8_t length = 1)
      : startAddress(startAddress), length(length) {}
  virtual ~Base() = default;
  virtual void parse(uint8_t* dataArr, uint8_t length) = 0;
  virtual void print(Stream* serial) = 0;
  virtual void toBytes(uint8_t* data, uint8_t length) = 0;

  void println(Stream* serial) {
    print(serial);
    serial->println();
  }
};

class BaseOneValue : public Base {
 public:
  BaseOneValue(uint8_t startAddress, String label)
      : Base(startAddress), valueLabel(label) {}

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    *valueP() = dataArr[0];
  }

  virtual void print(Stream* serial) {
    serial->print(valueLabel + " " + String(*valueP()));
  }

  void toBytes(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    dataArr[0] = *valueP();
  }

 private:
  const String valueLabel;
  virtual uint8_t* valueP() = 0;
};

class R00 : public Base {
 public:
  R00() : Base(0) {}
  bool chargePumpLow = false;
  bool temperature = false;
  bool overcurrent = false;
  bool maximumRotationNumber = false;
  bool minimumRotationNumber = false;
  bool startup = false;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    chargePumpLow = (data & 0b100000) != 0;
    temperature = (data & 0b10000) != 0;
    overcurrent = (data & 0b1000) != 0;
    maximumRotationNumber = (data & 0b100) != 0;
    minimumRotationNumber = (data & 0b10) != 0;
    startup = (data & 0b1) != 0;
  }

  void toBytes(uint8_t* dataArr, uint8_t length) {
    // do nothing because r0 is read only register
    return;
  }

  void print(Stream* serial) {
    serial->print("chargePumpLow " + string_asukiaaa::trueFalse(chargePumpLow) +
                  ", ");
    serial->print("temperature " + string_asukiaaa::trueFalse(temperature) +
                  ", ");
    serial->print("overcurrent " + string_asukiaaa::trueFalse(overcurrent) +
                  ", ");
    serial->print("maximumRotationNumber " +
                  string_asukiaaa::trueFalse(maximumRotationNumber) + ", ");
    serial->print("minimumRotationNumber " +
                  string_asukiaaa::trueFalse(minimumRotationNumber) + ", ");
    serial->print("startup " + string_asukiaaa::trueFalse(startup));
  }
};

class R01 : public BaseOneValue {
 public:
  R01() : BaseOneValue(1, "userId") {}
  uint8_t userId = 0;

 private:
  uint8_t* valueP() { return &userId; }
};

class R02 : public Base {
 public:
  R02() : Base(2) {}
  bool noStop = false;
  uint8_t stopDuty = 0;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    noStop = bitRead(byte, 7);
    stopDuty = byte & 0x7f;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (noStop) {
      byte |= bit(7);
    }
    byte |= stopDuty & 0x7f;
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("noStop " + string_asukiaaa::trueFalse(noStop) + ", ");
    serial->print("stopDuty " + String(stopDuty));
  }
};

class R03 : public BaseOneValue {
 public:
  R03() : BaseOneValue(3, "startDuty") {}
  uint8_t startDuty = 0;
  void setStartDutyByRate(float rate) {
    uint16_t val = rate * 0x200;
    startDuty = val > 0xff ? 0xff : val;
  }

  void print(Stream* serial) {
    serial->print("startDuty " + String(startDuty) + " (" +
                  String((float)startDuty / 0x200) + ")");
  }

 private:
  uint8_t* valueP() { return &startDuty; }
};

class R04 : public BaseOneValue {
 public:
  R04() : BaseOneValue(4, "changeDuty") {}
  uint8_t changeDuty = 0;
  void setChangeDutyByRate(float rate) { changeDuty = rate * 0xff; }
  void print(Stream* serial) {
    String strChangeDuty = changeDuty == 0
                               ? "not used"
                               : "(" + String((float)changeDuty / 0x100) + ")";
    serial->print("changeDuty " + String(changeDuty) + " " + strChangeDuty);
  }

 private:
  uint8_t* valueP() { return &changeDuty; }
};

class R05 : public BaseOneValue {
 public:
  R05() : BaseOneValue(5, "maxDuty") {}
  uint8_t maxDuty = 0;
  void setMaxDutyByRate(float rate) {
    maxDuty = rate < 0.5 ? 0 : ((uint16_t)(rate * 0x200) - 0x101);
  }
  void print(Stream* serial) {
    serial->print("maxDuty " + String(maxDuty) + " (" +
                  String(((float)maxDuty + 0x101) / 0x200) + ")");
  }

 private:
  uint8_t* valueP() { return &maxDuty; }
};

class R06to07 : public Base {
 public:
  R06to07() : Base(6, 2) {}
  uint16_t startRpm = 0;
  uint8_t maxDutyHys = 0;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    startRpm = (((uint16_t)data[1] & 0x0f) << 8) | data[0];
    maxDutyHys = data[1] & 0b1111;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    data[0] = startRpm & 0xff;
    data[1] = ((startRpm >> 8) & 0x0f) | (maxDutyHys & 0x0f);
  }

  void print(Stream* serial) {
    serial->print("startRpm " + String(startRpm) + ", ");
    serial->print("maxDutyHys " + String(maxDutyHys));
  }
};

class R08to09 : public Base {
 public:
  R08to09() : Base(8, 2) {}
  uint16_t speedSlop = 0;
  bool maxOpen = false;
  bool maxOff = false;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    speedSlop = ((uint16_t)(data[1] & 0xfc) << 6) | data[0];
    maxOpen = bitRead(data[1], 1);
    maxOff = bitRead(data[1], 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    data[0] = speedSlop & 0x00ff;
    data[1] = ((speedSlop >> 6) & 0xfc) | (maxOpen ? bit(1) : 0) |
              (maxOff ? bit(0) : 0);
  }

  void print(Stream* serial) {
    serial->print("speedSlop " + String(speedSlop) + ", ");
    serial->print("maxOpen " + string_asukiaaa::trueFalse(maxOpen) + ", ");
    serial->print("maxOff " + string_asukiaaa::trueFalse(maxOff));
  }
};

class R10to11 : public Base {
 public:
  R10to11() : Base(10, 2) {}
  uint16_t speedSlop2 = 0;
  bool vcpMask = false;
  bool openLoop = false;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    speedSlop2 = (uint16_t)data[0] << 6 | data[1] >> 2;
    vcpMask = bitRead(data[1], 1);
    openLoop = bitRead(data[1], 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    data[0] = (speedSlop2 >> 6) & 0xff;
    data[1] = ((speedSlop2 << 2) & 0xff) | (vcpMask ? bit(1) : 0) |
              (openLoop ? bit(0) : 0);
  }

  void print(Stream* serial) {
    serial->print("speedSlop2 " + String(speedSlop2) + ", ");
    serial->print("vcpMask " + string_asukiaaa::trueFalse(vcpMask) + ", ");
    serial->print("openLoop " + string_asukiaaa::trueFalse(openLoop));
  }
};

class R12 : public Base {
 public:
  R12() : Base(12) {}
  ValueTypes::KX kix;
  uint8_t ki;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    kix = (ValueTypes::KX)bitRead(byte, 7);
    ki = byte & 0x7f;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)kix << 7;
    byte |= ki & 0x7f;
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("kix " + utils::getStrOfKX(kix) + ", ");
    serial->print("ki " + String(ki));
  }
};

class R13 : public Base {
 public:
  R13() : Base(13) {}
  ValueTypes::KX kpx;
  uint8_t kp;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    kpx = (ValueTypes::KX)bitRead(byte, 7);
    kp = byte & 0x7f;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)kpx << 7;
    byte |= kp & 0x7f;
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("kpx " + utils::getStrOfKX(kpx) + ", ");
    serial->print("kp " + String(kp));
  }
};

class R14 : public Base {
 public:
  R14() : Base(14) {}
  bool stbyMode;
  bool dir;
  ValueTypes::PolePair polePair;
  ValueTypes::MaxSpeedRpm maxSpeedRpm;
  uint8_t fgOn;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    stbyMode = bitRead(data, 7);
    dir = bitRead(data, 6);
    polePair = (ValueTypes::PolePair)((data >> 3) & 0b111);
    maxSpeedRpm = (ValueTypes::MaxSpeedRpm)((data >> 1) & 0b11);
    fgOn = data & 0b1;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (stbyMode) {
      byte |= bit(7);
    }
    if (dir) {
      byte |= bit(6);
    }
    byte |= (uint8_t)polePair << 3;
    byte |= (uint8_t)maxSpeedRpm << 1;
    if (fgOn) {
      byte |= 1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("stbyMode " + string_asukiaaa::trueFalse(stbyMode) + ", ");
    serial->print("dir " + string_asukiaaa::trueFalse(dir) + ", ");
    auto numPole = ((uint8_t)polePair + 1) * 2;
    serial->print("polePair " + String(numPole) + ", ");
    using ValueTypes::MaxSpeedRpm;
    String strRpm = maxSpeedRpm == MaxSpeedRpm::rpm4096    ? "4096"
                    : maxSpeedRpm == MaxSpeedRpm::rpm8192  ? "8192"
                    : maxSpeedRpm == MaxSpeedRpm::rpm16834 ? "16834"
                                                           : "32768";
    serial->print("maxSpeedRpm " + strRpm + "rpm, ");
    serial->print("fgOn " + string_asukiaaa::trueFalse(fgOn) + ", ");
  }
};

class R15 : public Base {
 public:
  R15() : Base(15) {}
  ValueTypes::FGSel fgSel;
  ValueTypes::SpeedInputSelect speedInputSelect;
  ValueTypes::SpeedInv speedInv;
  ValueTypes::Latch latch;
  ValueTypes::OCPFilterTime ocpFilterTime;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    fgSel = (ValueTypes::FGSel)((data >> 5) & 0b111);
    speedInputSelect = (ValueTypes::SpeedInputSelect)bitRead(data, 4);
    speedInv = (ValueTypes::SpeedInv)bitRead(data, 3);
    latch = (ValueTypes::Latch)bitRead(data, 2);
    ocpFilterTime = (ValueTypes::OCPFilterTime)(data & 0b11);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)fgSel << 5;
    if (speedInputSelect == ValueTypes::SpeedInputSelect::pwm) {
      byte |= bit(4);
    }
    if (speedInv == ValueTypes::SpeedInv::negative) {
      byte |= bit(3);
    }
    if (latch == ValueTypes::Latch::latch) {
      byte |= bit(2);
    }
    byte |= (uint8_t)ocpFilterTime & 0b11;
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::FGSel;
    String strFgSel = fgSel == FGSel::ppr1       ? "2ppr"
                      : fgSel == FGSel::ppr1div2 ? "1/2ppr"
                      : fgSel == FGSel::ppr1div3 ? "1/3ppr"
                      : fgSel == FGSel::ppr2     ? "2ppr"
                      : fgSel == FGSel::ppr2div3 ? "2/3ppr"
                      : fgSel == FGSel::ppr2p4   ? "2.4ppr"
                      : fgSel == FGSel::ppr3     ? "3ppr"
                                                 : "alert";
    serial->print("fgSel " + strFgSel + ", ");
    serial->print(
        "speedInputSelect " +
        String(speedInputSelect == ValueTypes::SpeedInputSelect::analogVoltage
                   ? "analogVoltage"
                   : "pwm") +
        ", ");
    serial->print("speedInv " +
                  String(speedInv == ValueTypes::SpeedInv::positive
                             ? "positive"
                             : "negatime") +
                  ", ");
    serial->print(
        "latch " +
        String(latch == ValueTypes::Latch::latch ? "latch" : "auto resetart") +
        ", ");
    using ValueTypes::OCPFilterTime;
    String strOcpFilterTime = ocpFilterTime == OCPFilterTime::ns500   ? "500ns"
                              : ocpFilterTime == OCPFilterTime::ns666 ? "666ns"
                              : ocpFilterTime == OCPFilterTime::ns750 ? "750ns"
                                                                      : "none";
    serial->print("OCP filter time " + strOcpFilterTime);
  }
};

class R16 : public Base {
 public:
  R16() : Base(16) {}
  bool disableLock;
  ValueTypes::DutyChangeLimit dutyChangeLimit;
  ValueTypes::StartCurrentLimit startCurrentLimit;
  bool disableOCP;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    disableLock = bitRead(byte, 7) != 0;
    dutyChangeLimit = (ValueTypes::DutyChangeLimit)((byte >> 4) & 0b111);
    startCurrentLimit = (ValueTypes::StartCurrentLimit)((byte >> 1) & 0b111);
    disableOCP = bitRead(byte, 0) != 0;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (disableLock) {
      byte |= bit(7);
    }
    byte |= (uint8_t)dutyChangeLimit << 4;
    byte |= (uint8_t)startCurrentLimit << 1;
    if (disableOCP) {
      byte |= 0b1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("disableLock " + string_asukiaaa::trueFalse(disableLock) +
                  ", ");
    using ValueTypes::DutyChangeLimit;
    String strDuty = dutyChangeLimit == DutyChangeLimit::x10div8   ? "10/8"
                     : dutyChangeLimit == DutyChangeLimit::x20div8 ? "30/8"
                     : dutyChangeLimit == DutyChangeLimit::x2div8  ? "2/8"
                     : dutyChangeLimit == DutyChangeLimit::x3vid8  ? "3/8"
                     : dutyChangeLimit == DutyChangeLimit::x4div8  ? "4/8"
                     : dutyChangeLimit == DutyChangeLimit::x56div8 ? "56/8"
                     : dutyChangeLimit == DutyChangeLimit::x6div8
                         ? "6/8"
                         : "64/8 for closed loop disable for open loop";
    serial->print("dutyChangeLimit " + strDuty + ", ");
    using ValueTypes::StartCurrentLimit;
    String strStart = startCurrentLimit == StartCurrentLimit::x0p125   ? "0.125"
                      : startCurrentLimit == StartCurrentLimit::x0p25  ? "0.25"
                      : startCurrentLimit == StartCurrentLimit::x0p375 ? "0.375"
                      : startCurrentLimit == StartCurrentLimit::x0p5   ? "0.5"
                      : startCurrentLimit == StartCurrentLimit::x0p625 ? "0.625"
                      : startCurrentLimit == StartCurrentLimit::x0p75  ? "0.75"
                      : startCurrentLimit == StartCurrentLimit::x0p875 ? "0.875"
                                                                       : "1";
    serial->print("startCurrentLimit x" + strStart + ", ");
    serial->print("disableOCP " + string_asukiaaa::trueFalse(disableOCP));
  }
};

class R17 : public Base {
 public:
  R17() : Base(17) {}
  ValueTypes::SSAddSel ssAddSel;
  ValueTypes::SSUpSel ssUpSel;
  ValueTypes::SSDutyChangeLimit ssDutyChangeLimit;
  ValueTypes::DutyUpTime dutyUpTime;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    ssAddSel = (ValueTypes::SSAddSel)((data >> 6) & 0b11);
    ssUpSel = (ValueTypes::SSUpSel)((data >> 4) & 0b11);
    ssDutyChangeLimit = (ValueTypes::SSDutyChangeLimit)((data >> 1) & 0b111);
    dutyUpTime = (ValueTypes::DutyUpTime)bitRead(data, 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)ssAddSel << 6;
    byte |= (uint8_t)ssUpSel << 4;
    byte |= (uint8_t)ssDutyChangeLimit << 1;
    byte |= (uint8_t)dutyUpTime;
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::SSAddSel;
    String strAddSel = ssAddSel == SSAddSel::x0p0   ? "0"
                       : ssAddSel == SSAddSel::x0p3 ? "0.3"
                       : ssAddSel == SSAddSel::x0p4 ? "0.4"
                                                    : "0.5";
    serial->print("ssAddSel x" + strAddSel + ", ");
    using ValueTypes::SSUpSel;
    String strUpSel = ssUpSel == SSUpSel::x0p01   ? "0.01"
                      : ssUpSel == SSUpSel::x0p02 ? "0.02"
                      : ssUpSel == SSUpSel::x0p05 ? "0.05"
                                                  : "0.10";
    serial->print("ssUpSel x" + strUpSel + ", ");
    using ValueTypes::SSDutyChangeLimit;
    String strDutyChangeLimit =
        ssDutyChangeLimit == SSDutyChangeLimit::percent0p17   ? "0.17"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent0p20 ? "0.20"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent0p55 ? "0.55"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent1p11 ? "1.11"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent1p84 ? "1.84"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent2p76 ? "2.76"
        : ssDutyChangeLimit == SSDutyChangeLimit::percent3p69 ? "3.69"
                                                              : "5.53";
    serial->print("ssDutyChangeLimit " + strDutyChangeLimit + "%, ");
    String strDutyUpTime =
        dutyUpTime == ValueTypes::DutyUpTime::ms2p7 ? "2.7" : "10.8";
    serial->print("dutyUpTime " + strDutyUpTime + "ms");
  }
};

class R18 : public Base {
 public:
  R18() : Base(18) {}
  ValueTypes::RPMChangeLimit rpmChangeLimit;
  bool invBrake;
  bool disableOverCurrentDetection;
  ValueTypes::AnalogFilter analogFilter;
  bool disableAutoDeatTimeControl;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    rpmChangeLimit = (ValueTypes::RPMChangeLimit)(byte >> 5);
    invBrake = bitRead(byte, 4) != 0;
    disableOverCurrentDetection = bitRead(byte, 3) != 0;
    analogFilter = (ValueTypes::AnalogFilter)((byte >> 1) & 0b11);
    disableAutoDeatTimeControl = bitRead(byte, 0) != 0;
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)rpmChangeLimit << 5;
    if (invBrake) {
      byte |= bit(4);
    }
    if (disableOverCurrentDetection) {
      byte |= bit(3);
    }
    byte |= (uint8_t)analogFilter << 1;
    if (disableAutoDeatTimeControl) {
      byte |= 0b1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::RPMChangeLimit;
    String strRpmChange =
        rpmChangeLimit == RPMChangeLimit::rpm10240  ? "10240rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm2200 ? "2200rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm3800 ? "3800rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm512  ? "512rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm5400 ? "5400rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm7000 ? "7000rpm"
        : rpmChangeLimit == RPMChangeLimit::rpm8600 ? "8600rpm"
                                                    : "no limit";
    serial->print("rpmChangeLimit " + strRpmChange + ", ");
    serial->print("invBrake " + string_asukiaaa::trueFalse(invBrake) + ", ");
    serial->print("disableOverCurrentDetection " +
                  string_asukiaaa::trueFalse(disableOverCurrentDetection) +
                  ", ");
    using ValueTypes::AnalogFilter;
    String strAnalogFilter = analogFilter == AnalogFilter::kHz100   ? "kHz"
                             : analogFilter == AnalogFilter::kHz200 ? "200kHz"
                             : analogFilter == AnalogFilter::kHz50  ? "50kHz"
                                                                    : "none";
    serial->print("analogFilter " + strAnalogFilter + ", ");
    serial->print("disableAutoDeadTimecontrol " +
                  string_asukiaaa::trueFalse(disableAutoDeatTimeControl) +
                  ", ");
  }
};

class R19 : public Base {
 public:
  R19() : Base(19) {}
  uint8_t waitTime;
  bool waitMode;
  bool waitCon;
  bool lockBrk;
  ValueTypes::AlertPinConfig alertPinConfig;
  bool stdMask;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    waitTime = data >> 5;
    waitMode = bitRead(data, 4);
    waitCon = bitRead(data, 3);
    lockBrk = bitRead(data, 2);
    alertPinConfig = (ValueTypes::AlertPinConfig)bitRead(data, 1);
    stdMask = bitRead(data, 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= waitTime << 5;
    if (waitMode) {
      byte |= bit(4);
    }
    if (waitCon) {
      byte |= bit(3);
    }
    if (lockBrk) {
      byte |= bit(2);
    }
    if (alertPinConfig == ValueTypes::AlertPinConfig::lowOnAlert) {
      byte |= bit(1);
    }
    if (stdMask) {
      byte |= 1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("waitTime " + String(waitTime) + ", ");
    serial->print("waitMode " + string_asukiaaa::trueFalse(waitMode) + ", ");
    serial->print("waitCon " + string_asukiaaa::trueFalse(waitCon) + ", ");
    serial->print("lockBrk " + string_asukiaaa::trueFalse(lockBrk) + ", ");
    serial->print(
        "alertPinConfig " +
        String(alertPinConfig == ValueTypes::AlertPinConfig::highOnAlert
                   ? "high"
                   : "low") +
        " on alert, ");
    serial->print("waitMode " + string_asukiaaa::trueFalse(waitMode) + ", ");
  }
};

class R20 : public Base {  // TODO
 public:
  R20() : Base(20) {}
  ValueTypes::RestartTime restartTime;
  ValueTypes::PeriodOf1stDCExcitation periodOf1stDCExciation;
  ValueTypes::PeriodOf2ndDCExcitation periodOf2ndDCExciation;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    restartTime = (ValueTypes::RestartTime)(byte >> 5);
    periodOf1stDCExciation =
        (ValueTypes::PeriodOf1stDCExcitation)((byte >> 3) & 0b11);
    periodOf2ndDCExciation =
        (ValueTypes::PeriodOf2ndDCExcitation)(byte & 0b111);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)restartTime << 5;
    byte |= (uint8_t)periodOf1stDCExciation << 3;
    byte |= (uint8_t)periodOf2ndDCExciation;
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::RestartTime;
    String strRestartTime = restartTime == RestartTime::s0     ? "0s"
                            : restartTime == RestartTime::s0p5 ? "0.5s"
                            : restartTime == RestartTime::s10  ? "10s"
                            : restartTime == RestartTime::s1   ? "1s"
                            : restartTime == RestartTime::s1p5 ? "1.5s"
                            : restartTime == RestartTime::s2   ? "2s"
                            : restartTime == RestartTime::s4   ? "4s"
                                                               : "7s";
    serial->print("restartTime " + strRestartTime + ", ");
    using ValueTypes::PeriodOf1stDCExcitation;
    String str1st =
        periodOf1stDCExciation == PeriodOf1stDCExcitation::s0     ? "0s"
        : periodOf1stDCExciation == PeriodOf1stDCExcitation::s0p2 ? "0.2s"
        : periodOf1stDCExciation == PeriodOf1stDCExcitation::s0p5 ? "0.5s"
                                                                  : "1s";
    serial->print("periodOf1stDCExcitation " + str1st + ", ");
    using ValueTypes::PeriodOf2ndDCExcitation;
    String str2nd =
        periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p1   ? "0.1s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p2 ? "0.2s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p4 ? "0.4s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p6 ? "0.6s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p8 ? "0.8s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s1   ? "1s"
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s1p5 ? "1.5s"
                                                                  : "2s";
    serial->print("periodOf2ndDCExcitation " + str2nd);
  }
};

class R21 : public Base {
 public:
  R21() : Base(21) {}
  ValueTypes::LeadAngle leadAngle;
  ValueTypes::MaxRotationFrequency maxRotationFrequency;
  ValueTypes::MinRotationFrequency minRotationFrequency;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    leadAngle = (ValueTypes::LeadAngle)(data >> 4);
    maxRotationFrequency = (ValueTypes::MaxRotationFrequency)(data >> 2 & 0b11);
    minRotationFrequency = (ValueTypes::MinRotationFrequency)(data & 0b11);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= ((uint8_t)leadAngle << 4);
    byte |= ((uint8_t)maxRotationFrequency << 2);
    byte |= (uint8_t)minRotationFrequency;
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::LeadAngle;
    String strLeadAngle = leadAngle == LeadAngle::deg0       ? "0deg"
                          : leadAngle == LeadAngle::deg11p25 ? "11.25deg"
                          : leadAngle == LeadAngle::deg15    ? "15deg"
                          : leadAngle == LeadAngle::deg18p75 ? "18.75deg"
                          : leadAngle == LeadAngle::deg22p5  ? "22.5deg"
                          : leadAngle == LeadAngle::deg26p25 ? "26.25deg"
                          : leadAngle == LeadAngle::deg30    ? "30deg"
                          : leadAngle == LeadAngle::deg3p75  ? "3.75deg"
                          : leadAngle == LeadAngle::deg7p5   ? "7.5deg"
                          : leadAngle == LeadAngle::case9    ? "case0"
                          : leadAngle == LeadAngle::case10   ? "case10"
                          : leadAngle == LeadAngle::case11   ? "case11"
                          : leadAngle == LeadAngle::case12   ? "case12"
                          : leadAngle == LeadAngle::case13   ? "case13"
                          : leadAngle == LeadAngle::case14   ? "case14"
                                                             : "case15";
    serial->print("leadAngle " + strLeadAngle + ", ");
    using ValueTypes::MaxRotationFrequency;
    String strMax =
        maxRotationFrequency == MaxRotationFrequency::kHz0p75  ? "0.75kHz"
        : maxRotationFrequency == MaxRotationFrequency::kHz1p5 ? "1.5kHz"
        : maxRotationFrequency == MaxRotationFrequency::kHz3   ? "3kHz"
                                                               : "none";
    serial->print("maxRotationFrequency " + strMax + ", ");
    using ValueTypes::MinRotationFrequency;
    String strMin =
        minRotationFrequency == MinRotationFrequency::Hz1p6   ? "1.6Hz"
        : minRotationFrequency == MinRotationFrequency::Hz3p2 ? "3.2Hz"
        : minRotationFrequency == MinRotationFrequency::Hz6p4 ? "6.4Hz"
                                                              : "12.8Hz";
    serial->print("minRotationFrequency " + strMin);
  }
};

class R22 : public Base {
 public:
  R22() : Base(22) {}
  bool enableSoftSwitching;
  ValueTypes::SoftSwitchingAngle softSwitchingAngle;
  ValueTypes::OutputPWM outputPWM;
  ValueTypes::DeadTime deadTime;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t data = dataArr[0];
    enableSoftSwitching = bitRead(data, 7);
    softSwitchingAngle = (ValueTypes::SoftSwitchingAngle)((data >> 5) & 0b11);
    outputPWM = (ValueTypes::OutputPWM)((data >> 2) & 0b111);
    deadTime = (ValueTypes::DeadTime)(data & 0b11);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (enableSoftSwitching) {
      byte |= bit(7);
    }
    byte |= (uint8_t)softSwitchingAngle << 5;
    byte |= (uint8_t)outputPWM << 2;
    byte |= (uint8_t)deadTime;
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("enableSoftSwitching " +
                  string_asukiaaa::trueFalse(enableSoftSwitching) + ", ");
    using ValueTypes::SoftSwitchingAngle;
    String strSoftSwitchAngle =
        softSwitchingAngle == SoftSwitchingAngle::deg120   ? "120deg"
        : softSwitchingAngle == SoftSwitchingAngle::deg135 ? "135deg"
        : softSwitchingAngle == SoftSwitchingAngle::deg150
            ? "150deg"
            : "150deg for soft 142.5 for no soft";
    serial->print("softSwitchingAndle " + strSoftSwitchAngle + ", ");
    using ValueTypes::OutputPWM;
    String strOutputPWM = outputPWM == OutputPWM::kHz187p5  ? "187.5kHz"
                          : outputPWM == OutputPWM::kHz23p4 ? "23.4kHz"
                          : outputPWM == OutputPWM::kHz46p9 ? "46.9kHz"
                          : outputPWM == OutputPWM::kHz93p7 ? "93.7kHz"
                          : outputPWM == OutputPWM::case4   ? "case4"
                          : outputPWM == OutputPWM::case5   ? "case5"
                          : outputPWM == OutputPWM::case6   ? "case6"
                                                            : "case7";
    serial->print("outputPWM " + strOutputPWM + ", ");
    using ValueTypes::DeadTime;
    String strDeadTime = deadTime == DeadTime::ns250    ? "250ns"
                         : deadTime == DeadTime::ns500  ? "500ns"
                         : deadTime == DeadTime::ns1000 ? "1000ns"
                                                        : "1500ns";
    serial->print("deadTime " + strDeadTime);
  }
};

class R23 : public Base {
 public:
  R23() : Base(23) {}
  ValueTypes::ISDThreshold isdThreshold;
  ValueTypes::CurrentLimitVoltage currentLimitVoltage;
  ValueTypes::SourceCurrent sourceCurrent;
  ValueTypes::SincCurrent sincCurrent;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    isdThreshold = (ValueTypes::ISDThreshold)bitRead(byte, 7);
    currentLimitVoltage = (ValueTypes::CurrentLimitVoltage)bitRead(byte, 6);
    sourceCurrent = (ValueTypes::SourceCurrent)((byte >> 3) & 0b111);
    sincCurrent = (ValueTypes::SincCurrent)(byte & 0b111);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)isdThreshold << 7;
    byte |= (uint8_t)currentLimitVoltage << 6;
    byte |= (uint8_t)sourceCurrent << 3;
    byte |= (uint8_t)sincCurrent;
    data[0] = byte;
  }

  void print(Stream* serial) {
    String strIsdVolt =
        isdThreshold == ValueTypes::ISDThreshold::v1 ? "1V" : "0.5V";
    serial->print("ISD threshold " + strIsdVolt + ", ");
    String strCurrentLimitVolt =
        currentLimitVoltage == ValueTypes::CurrentLimitVoltage::v0p25
            ? "0.25V"
            : "0.125V";
    serial->print("currentLimitVoltage " + strCurrentLimitVolt + ", ");
    using ValueTypes::SourceCurrent;
    String strSourceCurrent =
        sourceCurrent == SourceCurrent::ma100    ? "100mA"
        : sourceCurrent == SourceCurrent::ma10   ? "10mA"
        : sourceCurrent == SourceCurrent::ma13p9 ? "13.9mA"
        : sourceCurrent == SourceCurrent::ma19p3 ? "19.3mA"
        : sourceCurrent == SourceCurrent::ma26p8 ? "26.8mA"
        : sourceCurrent == SourceCurrent::ma37p3 ? "37.3mA"
        : sourceCurrent == SourceCurrent::ma51p8 ? "51.8mA"
                                                 : "72.0mA";
    serial->print("sourceCurrent" + strSourceCurrent + ", ");
    using ValueTypes::SincCurrent;
    String strSincCurrent = sincCurrent == SincCurrent::ma103p6   ? "103.6mA"
                            : sincCurrent == SincCurrent::ma143p9 ? "143.9mA"
                            : sincCurrent == SincCurrent::ma20    ? "20mA"
                            : sincCurrent == SincCurrent::ma200   ? "200mA"
                            : sincCurrent == SincCurrent::ma27p8  ? "27.8mA"
                            : sincCurrent == SincCurrent::ma38p6  ? "38.6mA"
                            : sincCurrent == SincCurrent::ma53p7  ? "53.7mA"
                                                                  : "74.6mA";
    serial->print("sincCurrent" + strSincCurrent);
  }
};

class R24 : public Base {
 public:
  R24() : Base(24) {}
  ValueTypes::HysteresisVoltage hysteresisVoltage;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = data[0];
    hysteresisVoltage = (ValueTypes::HysteresisVoltage)(byte >> 6);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    byte |= (uint8_t)hysteresisVoltage << 6;
    data[0] = byte;
  }

  void print(Stream* serial) {
    using ValueTypes::HysteresisVoltage;
    String strVolt = hysteresisVoltage == HysteresisVoltage::mv100   ? "100mV"
                     : hysteresisVoltage == HysteresisVoltage::mv200 ? "200mV"
                     : hysteresisVoltage == HysteresisVoltage::mv300 ? "300mV"
                                                                     : "none";
    serial->print("hysteresisVoltage " + strVolt);
  }
};

class R25 : public BaseOneValue {
 public:
  R25() : BaseOneValue(25, "slaveAddress"){};
  uint8_t slaveAddress;
  void print(Stream* serial) {
    serial->print("slaveAddress " + String(slaveAddress) + " (0x" +
                  string_asukiaaa::padStart(String(slaveAddress, HEX), 2, '0') +
                  ")");
  }

 private:
  uint8_t* valueP() { return &slaveAddress; }
};

class R27to28 : public Base {
 public:
  R27to28() : Base(27, 2) {}
  uint16_t spd;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      Serial.println("skip parse r27");
      return;
    }
    // spd = (uint16_t)dataArr[0] << 2 | dataArr[1] >> 6;
    spd = (uint16_t)dataArr[1] << 8 | dataArr[0];
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    // data[0] = (spd >> 2) & 0xff;
    // data[1] = (spd << 6) & 0xff;
    data[0] = spd & 0xff;
    data[1] = (spd >> 6) & 0xc0;
  }

  void setSpeedByRate(float rate) {
    if (rate < 0) {
      rate = 0;
    }
    if (rate > 1) {
      rate = 1;
    }
    spd = 0x200 * rate;
  }

  void print(Stream* serial) {
    float rate = (float)spd / 0x200;
    serial->print("spd " + String(spd) + " (rate " + String(rate) + ")");
  }
};

class R29to30 : public Base {
 public:
  R29to30() : Base(29, 2) {}
  uint16_t hzCnt;

  void parse(uint8_t* dataArr, uint8_t length) {
    if (length < this->length) {
      return;
    }
    // hzCnt = (((uint16_t)dataArr[1]) << 8) | dataArr[0];
    hzCnt = (uint16_t)dataArr[1] << 8 | dataArr[0];
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    data[1] = hzCnt << 8;
    data[0] = hzCnt & 0xff;
  }

  void print(Stream* serial) {
    String strRotationFreq =
        hzCnt == 0xffff ? "stop" : String((float)250000.0 / hzCnt) + "Hz";
    serial->print("hzCnt " + String(hzCnt) + " " + strRotationFreq);
  }
};

class R86 : public Base {
 public:
  R86() : Base(86) {}
  ValueTypes::NVMReadWrite nvmReadWrite;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    nvmReadWrite = (ValueTypes::NVMReadWrite)(bitRead(data[0], 0));
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (nvmReadWrite == ValueTypes::NVMReadWrite::write) {
      byte |= 0b1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("nvmReadWrite " +
                  String(nvmReadWrite == ValueTypes::NVMReadWrite::read
                             ? "read"
                             : "write"));
  }
};

class R87 : public Base {
 public:
  R87() : Base(87) {}
  bool nvmStart;

  void parse(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    nvmStart = bitRead(data[0], 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    uint8_t byte = 0;
    if (nvmStart) {
      byte |= 0b1;
    }
    data[0] = byte;
  }

  void print(Stream* serial) {
    serial->print("nvmStart " + string_asukiaaa::trueFalse(nvmStart));
  }
};

};  // namespace Registers
};  // namespace TC78B009FTG_asukiaaa
