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
    serial->print(F("chargePumpLow "));
    serial->print(string_asukiaaa::trueFalse(chargePumpLow));
    serial->print(F(", "));
    serial->print(F("temperature "));
    serial->print(string_asukiaaa::trueFalse(temperature));
    serial->print(F(", "));
    serial->print(F("overcurrent "));
    serial->print(string_asukiaaa::trueFalse(overcurrent));
    serial->print(F(", "));
    serial->print(F("maximumRotationNumber "));
    serial->print(string_asukiaaa::trueFalse(maximumRotationNumber));
    serial->print(F(", "));
    serial->print(F("minimumRotationNumber "));
    serial->print(string_asukiaaa::trueFalse(minimumRotationNumber));
    serial->print(F(", "));
    serial->print(F("startup "));
    serial->print(string_asukiaaa::trueFalse(startup));
  }
};

class R01 : public BaseOneValue {
 public:
  R01() : BaseOneValue(1, F("userId")) {}
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
    serial->print(F("noStop "));
    serial->print(string_asukiaaa::trueFalse(noStop));
    serial->print(F(", "));
    serial->print(F("stopDuty "));
    serial->print(String(stopDuty));
  }
};

class R03 : public BaseOneValue {
 public:
  R03() : BaseOneValue(3, F("startDuty")) {}
  uint8_t startDuty = 0;
  void setStartDutyByRate(float rate) {
    uint16_t val = rate * 0x200;
    startDuty = val > 0xff ? 0xff : val;
  }

  void print(Stream* serial) {
    serial->print(F("startDuty "));
    serial->print(startDuty);
    serial->print(F(" ("));
    serial->print((float)startDuty / 0x200);
    serial->print(F(")"));
  }

 private:
  uint8_t* valueP() { return &startDuty; }
};

class R04 : public BaseOneValue {
 public:
  R04() : BaseOneValue(4, F("changeDuty")) {}
  uint8_t changeDuty = 0;
  void setChangeDutyByRate(float rate) { changeDuty = rate * 0xff; }
  void print(Stream* serial) {
    String strChangeDuty = changeDuty == 0
                               ? "not used"
                               : "(" + String((float)changeDuty / 0x100) + ")";
    serial->print(F("changeDuty "));
    serial->print(changeDuty);
    serial->print(F(" "));
    serial->print(strChangeDuty);
  }

 private:
  uint8_t* valueP() { return &changeDuty; }
};

class R05 : public BaseOneValue {
 public:
  R05() : BaseOneValue(5, F("maxDuty")) {}
  uint8_t maxDuty = 0;
  void setMaxDutyByRate(float rate) {
    maxDuty = rate < 0.5 ? 0 : ((uint16_t)(rate * 0x200) - 0x101);
  }
  void print(Stream* serial) {
    serial->print(F("maxDuty "));
    serial->print(maxDuty);
    serial->print(F(" ("));
    serial->print(((float)maxDuty + 0x101) / 0x200);
    serial->print(F(")"));
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
    serial->print(F("startRpm "));
    serial->print(startRpm);
    serial->print(F(", "));
    serial->print(F("maxDutyHys "));
    serial->print(maxDutyHys);
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
    speedSlop = ((uint16_t)data[0] << 6) | ((data[1] & 0xfc) >> 2);
    maxOpen = bitRead(data[1], 1);
    maxOff = bitRead(data[1], 0);
  }

  void toBytes(uint8_t* data, uint8_t length) {
    if (length < this->length) {
      return;
    }
    data[0] = (speedSlop >> 6) & 0x00ff;
    data[1] = ((speedSlop << 2) & 0xfc) | (maxOpen ? bit(1) : 0) |
              (maxOff ? bit(0) : 0);
  }

  void print(Stream* serial) {
    serial->print(F("speedSlop "));
    serial->print(speedSlop);
    serial->print(F(", "));
    serial->print(F("maxOpen "));
    serial->print(string_asukiaaa::trueFalse(maxOpen));
    serial->print(F(", "));
    serial->print(F("maxOff "));
    serial->print(string_asukiaaa::trueFalse(maxOff));
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
    serial->print(F("speedSlop2 "));
    serial->print(speedSlop2);
    serial->print(F(", "));
    serial->print(F("vcpMask "));
    serial->print(string_asukiaaa::trueFalse(vcpMask));
    serial->print(F(", "));
    serial->print(F("openLoop "));
    serial->print(string_asukiaaa::trueFalse(openLoop));
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
    serial->print(F("kix "));
    serial->print(utils::getStrOfKX(kix));
    serial->print(F(", "));
    serial->print(F("ki "));
    serial->print(ki);
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
    serial->print(F("kpx "));
    serial->print(utils::getStrOfKX(kpx));
    serial->print(F(", "));
    serial->print(F("kp "));
    serial->print(kp);
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
    serial->print(F("stbyMode "));
    serial->print(string_asukiaaa::trueFalse(stbyMode));
    serial->print(F(", "));
    serial->print(F("dir "));
    serial->print(string_asukiaaa::trueFalse(dir));
    serial->print(F(", "));
    auto numPole = ((uint8_t)polePair + 1) * 2;
    serial->print(F("polePair "));
    serial->print(numPole);
    serial->print(F(", "));
    using ValueTypes::MaxSpeedRpm;
    auto strRpm = maxSpeedRpm == MaxSpeedRpm::rpm4096    ? F("4096")
                  : maxSpeedRpm == MaxSpeedRpm::rpm8192  ? F("8192")
                  : maxSpeedRpm == MaxSpeedRpm::rpm16834 ? F("16834")
                                                         : F("32768");
    serial->print(F("maxSpeedRpm "));
    serial->print(strRpm);
    serial->print(F("rpm, "));
    serial->print(F("fgOn "));
    serial->print(string_asukiaaa::trueFalse(fgOn));
    serial->print(F(", "));
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
    auto strFgSel = fgSel == FGSel::ppr1       ? F("2ppr")
                    : fgSel == FGSel::ppr1div2 ? F("1/2ppr")
                    : fgSel == FGSel::ppr1div3 ? F("1/3ppr")
                    : fgSel == FGSel::ppr2     ? F("2ppr")
                    : fgSel == FGSel::ppr2div3 ? F("2/3ppr")
                    : fgSel == FGSel::ppr2p4   ? F("2.4ppr")
                    : fgSel == FGSel::ppr3     ? F("3ppr")
                                               : F("alert");
    serial->print(F("fgSel "));
    serial->print(strFgSel);
    serial->print(F(", "));
    serial->print(F("speedInputSelect "));
    serial->print(speedInputSelect ==
                          ValueTypes::SpeedInputSelect::analogVoltage
                      ? F("analogVoltage")
                      : F("pwm"));
    serial->print(F(", "));
    serial->print(F("speedInv "));
    serial->print(speedInv == ValueTypes::SpeedInv::positive ? F("positive")
                                                             : F("negatime"));
    serial->print(F(", "));
    serial->print(F("latch "));
    serial->print(latch == ValueTypes::Latch::latch ? F("latch")
                                                    : F("auto resetart"));
    serial->print(F(", "));
    using ValueTypes::OCPFilterTime;
    auto strOcpFilterTime = ocpFilterTime == OCPFilterTime::ns500   ? F("500ns")
                            : ocpFilterTime == OCPFilterTime::ns666 ? F("666ns")
                            : ocpFilterTime == OCPFilterTime::ns750 ? F("750ns")
                                                                    : F("none");
    serial->print(F("OCP filter time "));
    serial->print(strOcpFilterTime);
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
    serial->print(F("disableLock "));
    serial->print(string_asukiaaa::trueFalse(disableLock));
    serial->print(F(", "));
    using ValueTypes::DutyChangeLimit;
    auto strDuty = dutyChangeLimit == DutyChangeLimit::x10div8   ? F("10/8")
                   : dutyChangeLimit == DutyChangeLimit::x20div8 ? F("30/8")
                   : dutyChangeLimit == DutyChangeLimit::x2div8  ? F("2/8")
                   : dutyChangeLimit == DutyChangeLimit::x3vid8  ? F("3/8")
                   : dutyChangeLimit == DutyChangeLimit::x4div8  ? F("4/8")
                   : dutyChangeLimit == DutyChangeLimit::x56div8 ? F("56/8")
                   : dutyChangeLimit == DutyChangeLimit::x6div8
                       ? F("6/8")
                       : F("64/8 for closed loop disable for open loop");
    serial->print(F("dutyChangeLimit "));
    serial->print(strDuty);
    serial->print(F(", "));
    using ValueTypes::StartCurrentLimit;
    auto strStart =
        startCurrentLimit == StartCurrentLimit::x0p125   ? F("0.125")
        : startCurrentLimit == StartCurrentLimit::x0p25  ? F("0.25")
        : startCurrentLimit == StartCurrentLimit::x0p375 ? F("0.375")
        : startCurrentLimit == StartCurrentLimit::x0p5   ? F("0.5")
        : startCurrentLimit == StartCurrentLimit::x0p625 ? F("0.625")
        : startCurrentLimit == StartCurrentLimit::x0p75  ? F("0.75")
        : startCurrentLimit == StartCurrentLimit::x0p875 ? F("0.875")
                                                         : F("1");
    serial->print(F("startCurrentLimit x"));
    serial->print(strStart);
    serial->print(F(", "));
    serial->print(F("disableOCP "));
    serial->print(string_asukiaaa::trueFalse(disableOCP));
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
    auto strAddSel = ssAddSel == SSAddSel::x0p0   ? F("0")
                     : ssAddSel == SSAddSel::x0p3 ? F("0.3")
                     : ssAddSel == SSAddSel::x0p4 ? F("0.4")
                                                  : F("0.5");
    serial->print(F("ssAddSel x"));
    serial->print(strAddSel);
    serial->print(F(", "));
    using ValueTypes::SSUpSel;
    auto strUpSel = ssUpSel == SSUpSel::x0p01   ? F("0.01")
                    : ssUpSel == SSUpSel::x0p02 ? F("0.02")
                    : ssUpSel == SSUpSel::x0p05 ? F("0.05")
                                                : F("0.10");
    serial->print(F("ssUpSel x"));
    serial->print(strUpSel);
    serial->print(F(", "));
    using ValueTypes::SSDutyChangeLimit;
    auto strDutyChangeLimit =
        ssDutyChangeLimit == SSDutyChangeLimit::percent0p17   ? F("0.17")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent0p20 ? F("0.20")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent0p55 ? F("0.55")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent1p11 ? F("1.11")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent1p84 ? F("1.84")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent2p76 ? F("2.76")
        : ssDutyChangeLimit == SSDutyChangeLimit::percent3p69 ? F("3.69")
                                                              : F("5.53");
    serial->print(F("ssDutyChangeLimit "));
    serial->print(strDutyChangeLimit);
    serial->print(F("%, "));
    auto strDutyUpTime =
        dutyUpTime == ValueTypes::DutyUpTime::ms2p7 ? F("2.7") : F("10.8");
    serial->print(F("dutyUpTime "));
    serial->print(strDutyUpTime);
    serial->print(F("ms"));
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
    auto strRpmChange =
        rpmChangeLimit == RPMChangeLimit::rpm10240  ? F("10240rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm2200 ? F("2200rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm3800 ? F("3800rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm512  ? F("512rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm5400 ? F("5400rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm7000 ? F("7000rpm")
        : rpmChangeLimit == RPMChangeLimit::rpm8600 ? F("8600rpm")
                                                    : F("no limit");
    serial->print(F("rpmChangeLimit "));
    serial->print(strRpmChange);
    serial->print(F(", "));
    serial->print(F("invBrake "));
    serial->print(string_asukiaaa::trueFalse(invBrake));
    serial->print(F(", "));
    serial->print(F("disableOverCurrentDetection "));
    serial->print(string_asukiaaa::trueFalse(disableOverCurrentDetection));
    serial->print(F(", "));
    using ValueTypes::AnalogFilter;
    auto strAnalogFilter = analogFilter == AnalogFilter::kHz100   ? F("kHz")
                           : analogFilter == AnalogFilter::kHz200 ? F("200kHz")
                           : analogFilter == AnalogFilter::kHz50  ? F("50kHz")
                                                                  : F("none");
    serial->print(F("analogFilter "));
    serial->print(strAnalogFilter);
    serial->print(F(", "));
    serial->print(F("disableAutoDeadTimecontrol "));
    serial->print(string_asukiaaa::trueFalse(disableAutoDeatTimeControl));
    serial->print(F(", "));
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
    serial->print(F("waitTime "));
    serial->print(waitTime);
    serial->print(F(", "));
    serial->print(F("waitMode "));
    serial->print(string_asukiaaa::trueFalse(waitMode));
    serial->print(F(", "));
    serial->print(F("waitCon "));
    serial->print(string_asukiaaa::trueFalse(waitCon));
    serial->print(F(", "));
    serial->print(F("lockBrk "));
    serial->print(string_asukiaaa::trueFalse(lockBrk));
    serial->print(F(", "));
    serial->print(F("alertPinConfig "));
    serial->print(alertPinConfig == ValueTypes::AlertPinConfig::highOnAlert
                      ? F("high")
                      : F("low"));
    serial->print(F(" on alert, "));
    serial->print(F("waitMode "));
    serial->print(string_asukiaaa::trueFalse(waitMode));
    serial->print(F(", "));
  }
};

class R20 : public Base {
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
    auto strRestartTime = restartTime == RestartTime::s0     ? F("0s")
                          : restartTime == RestartTime::s0p5 ? F("0.5s")
                          : restartTime == RestartTime::s10  ? F("10s")
                          : restartTime == RestartTime::s1   ? F("1s")
                          : restartTime == RestartTime::s1p5 ? F("1.5s")
                          : restartTime == RestartTime::s2   ? F("2s")
                          : restartTime == RestartTime::s4   ? F("4s")
                                                             : F("7s");
    serial->print(F("restartTime "));
    serial->print(strRestartTime);
    serial->print(F(", "));
    using ValueTypes::PeriodOf1stDCExcitation;
    auto str1st =
        periodOf1stDCExciation == PeriodOf1stDCExcitation::s0     ? F("0s")
        : periodOf1stDCExciation == PeriodOf1stDCExcitation::s0p2 ? F("0.2s")
        : periodOf1stDCExciation == PeriodOf1stDCExcitation::s0p5 ? F("0.5s")
                                                                  : F("1s");
    serial->print(F("periodOf1stDCExcitation "));
    serial->print(str1st);
    serial->print(F(", "));
    using ValueTypes::PeriodOf2ndDCExcitation;
    auto str2nd =
        periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p1   ? F("0.1s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p2 ? F("0.2s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p4 ? F("0.4s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p6 ? F("0.6s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s0p8 ? F("0.8s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s1   ? F("1s")
        : periodOf2ndDCExciation == PeriodOf2ndDCExcitation::s1p5 ? F("1.5s")
                                                                  : F("2s");
    serial->print(F("periodOf2ndDCExcitation "));
    serial->print(str2nd);
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
    auto strLeadAngle = leadAngle == LeadAngle::deg0       ? F("0deg")
                        : leadAngle == LeadAngle::deg11p25 ? F("11.25deg")
                        : leadAngle == LeadAngle::deg15    ? F("15deg")
                        : leadAngle == LeadAngle::deg18p75 ? F("18.75deg")
                        : leadAngle == LeadAngle::deg22p5  ? F("22.5deg")
                        : leadAngle == LeadAngle::deg26p25 ? F("26.25deg")
                        : leadAngle == LeadAngle::deg30    ? F("30deg")
                        : leadAngle == LeadAngle::deg3p75  ? F("3.75deg")
                        : leadAngle == LeadAngle::deg7p5   ? F("7.5deg")
                        : leadAngle == LeadAngle::case9    ? F("case0")
                        : leadAngle == LeadAngle::case10   ? F("case10")
                        : leadAngle == LeadAngle::case11   ? F("case11")
                        : leadAngle == LeadAngle::case12   ? F("case12")
                        : leadAngle == LeadAngle::case13   ? F("case13")
                        : leadAngle == LeadAngle::case14   ? F("case14")
                                                           : F("case15");
    serial->print(F("leadAngle "));
    serial->print(strLeadAngle);
    serial->print(F(", "));
    using ValueTypes::MaxRotationFrequency;
    auto strMax =
        maxRotationFrequency == MaxRotationFrequency::kHz0p75  ? F("0.75kHz")
        : maxRotationFrequency == MaxRotationFrequency::kHz1p5 ? F("1.5kHz")
        : maxRotationFrequency == MaxRotationFrequency::kHz3   ? F("3kHz")
                                                               : F("none");
    serial->print(F("maxRotationFrequency "));
    serial->print(strMax);
    serial->print(F(", "));
    using ValueTypes::MinRotationFrequency;
    auto strMin =
        minRotationFrequency == MinRotationFrequency::Hz1p6   ? F("1.6Hz")
        : minRotationFrequency == MinRotationFrequency::Hz3p2 ? F("3.2Hz")
        : minRotationFrequency == MinRotationFrequency::Hz6p4 ? F("6.4Hz")
                                                              : F("12.8Hz");
    serial->print(F("minRotationFrequency "));
    serial->print(strMin);
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
    serial->print(F("enableSoftSwitching "));
    serial->print(string_asukiaaa::trueFalse(enableSoftSwitching));
    serial->print(F(", "));
    using ValueTypes::SoftSwitchingAngle;
    auto strSoftSwitchAngle =
        softSwitchingAngle == SoftSwitchingAngle::deg120   ? F("120deg")
        : softSwitchingAngle == SoftSwitchingAngle::deg135 ? F("135deg")
        : softSwitchingAngle == SoftSwitchingAngle::deg150
            ? F("150deg")
            : F("150deg for soft 142.5 for no soft");
    serial->print(F("softSwitchingAndle "));
    serial->print(strSoftSwitchAngle);
    serial->print(F(", "));
    using ValueTypes::OutputPWM;
    auto strOutputPWM = outputPWM == OutputPWM::kHz187p5  ? F("187.5kHz")
                        : outputPWM == OutputPWM::kHz23p4 ? F("23.4kHz")
                        : outputPWM == OutputPWM::kHz46p9 ? F("46.9kHz")
                        : outputPWM == OutputPWM::kHz93p7 ? F("93.7kHz")
                        : outputPWM == OutputPWM::case4   ? F("case4")
                        : outputPWM == OutputPWM::case5   ? F("case5")
                        : outputPWM == OutputPWM::case6   ? F("case6")
                                                          : F("case7");
    serial->print(F("outputPWM "));
    serial->print(strOutputPWM);
    serial->print(F(", "));
    using ValueTypes::DeadTime;
    auto strDeadTime = deadTime == DeadTime::ns250    ? F("250ns")
                       : deadTime == DeadTime::ns500  ? F("500ns")
                       : deadTime == DeadTime::ns1000 ? F("1000ns")
                                                      : F("1500ns");
    serial->print(F("deadTime "));
    serial->print(strDeadTime);
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
    auto strIsdVolt =
        isdThreshold == ValueTypes::ISDThreshold::v1 ? F("1V") : F("0.5V");
    serial->print(F("ISD threshold "));
    serial->print(strIsdVolt);
    serial->print(F(", "));
    auto strCurrentLimitVolt =
        currentLimitVoltage == ValueTypes::CurrentLimitVoltage::v0p25
            ? F("0.25V")
            : F("0.125V");
    serial->print(F("currentLimitVoltage "));
    serial->print(strCurrentLimitVolt);
    serial->print(F(", "));
    using ValueTypes::SourceCurrent;
    auto strSourceCurrent =
        sourceCurrent == SourceCurrent::ma100    ? F("100mA")
        : sourceCurrent == SourceCurrent::ma10   ? F("10mA")
        : sourceCurrent == SourceCurrent::ma13p9 ? F("13.9mA")
        : sourceCurrent == SourceCurrent::ma19p3 ? F("19.3mA")
        : sourceCurrent == SourceCurrent::ma26p8 ? F("26.8mA")
        : sourceCurrent == SourceCurrent::ma37p3 ? F("37.3mA")
        : sourceCurrent == SourceCurrent::ma51p8 ? F("51.8mA")
                                                 : F("72.0mA");
    serial->print(F("sourceCurrent"));
    serial->print(strSourceCurrent);
    serial->print(F(", "));
    using ValueTypes::SincCurrent;
    auto strSincCurrent = sincCurrent == SincCurrent::ma103p6   ? F("103.6mA")
                          : sincCurrent == SincCurrent::ma143p9 ? F("143.9mA")
                          : sincCurrent == SincCurrent::ma20    ? F("20mA")
                          : sincCurrent == SincCurrent::ma200   ? F("200mA")
                          : sincCurrent == SincCurrent::ma27p8  ? F("27.8mA")
                          : sincCurrent == SincCurrent::ma38p6  ? F("38.6mA")
                          : sincCurrent == SincCurrent::ma53p7  ? F("53.7mA")
                                                                : F("74.6mA");
    serial->print(F("sincCurrent"));
    serial->print(strSincCurrent);
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
    auto strVolt = hysteresisVoltage == HysteresisVoltage::mv100   ? F("100mV")
                   : hysteresisVoltage == HysteresisVoltage::mv200 ? F("200mV")
                   : hysteresisVoltage == HysteresisVoltage::mv300 ? F("300mV")
                                                                   : F("none");
    serial->print(F("hysteresisVoltage "));
    serial->print(strVolt);
  }
};

class R25 : public BaseOneValue {
 public:
  R25() : BaseOneValue(25, F("slaveAddress")){};
  uint8_t slaveAddress;
  void print(Stream* serial) {
    serial->print(F("slaveAddress "));
    serial->print(slaveAddress);
    serial->print(F(" (0x"));
    serial->print(string_asukiaaa::padStart(String(slaveAddress, HEX), 2, '0'));
    serial->print(F(")"));
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
    serial->print(F("spd "));
    serial->print(spd);
    serial->print(F(" (rate "));
    serial->print(rate);
    serial->print(F(")"));
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
    serial->print(F("hzCnt "));
    serial->print(hzCnt);
    serial->print(F(" "));
    serial->print(strRotationFreq);
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
    serial->print(F("nvmReadWrite "));
    serial->print(nvmReadWrite == ValueTypes::NVMReadWrite::read ? F("read")
                                                                 : F("write"));
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
    serial->print(F("nvmStart "));
    serial->print(string_asukiaaa::trueFalse(nvmStart));
  }
};

};  // namespace Registers
};  // namespace TC78B009FTG_asukiaaa
