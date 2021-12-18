#pragma once

#include <Arduino.h>

namespace TC78B009FTG_asukiaaa {
namespace ValueTypes {

// kix R12[7:7], kpx R13[7:7]
enum class KX : uint8_t {
  x1 = 0,
  x8 = 1,
};

// R14[5:3]
enum class PolePair : uint8_t {
  n2 = 0,
  n4 = 1,
  n6 = 2,
  n8 = 3,
  n10 = 4,
  n12 = 5,
  n14 = 6,
  n16 = 7,
};

// R14[2:1]
enum class MaxSpeedRpm : uint8_t {
  rpm4096 = 0,
  rpm8192 = 1,
  rpm16834 = 2,
  rpm32768 = 3,
};

// R15[7:5]
enum class FGSel : uint8_t {
  ppr1 = 0,
  ppr2div3 = 1,
  ppr1div2 = 2,
  ppr2 = 3,
  ppr3 = 4,
  ppr2p4 = 5,
  ppr1div3 = 6,
  alert = 7,
};

// R15[4:4]
enum class SpeedInputSelect : uint8_t {
  analogVoltage = 0,
  pwm = 1,
};

// R15[3:3]
enum class SpeedInv : uint8_t {
  positive = 0,
  negative = 1,
};

// R15[2:2]
enum class Latch : uint8_t {
  autoRestart = 0,
  latch = 1,
};

// R15[1:0]
enum class OCPFilterTime : uint8_t {
  none = 0,
  ns500 = 1,
  ns666 = 2,
  ns750 = 3,
};

// R16[6:4]
enum class DutyChangeLimit : uint8_t {
  x64div8ForClosedLoopDisableForOpenLoop = 0,
  x2div8 = 1,
  x3vid8 = 2,
  x4div8 = 3,
  x6div8 = 4,
  x10div8 = 5,
  x20div8 = 6,
  x56div8 = 7,
};

// R16[3:1]
enum class StartCurrentLimit : uint8_t {
  x1 = 0,
  x0p875 = 1,
  x0p75 = 2,
  x0p625 = 3,
  x0p5 = 4,
  x0p375 = 5,
  x0p25 = 6,
  x0p125 = 7,
};

// R17[7:6]
enum class SSAddSel {
  x0p0 = 0,
  x0p3 = 1,
  x0p4 = 2,
  x0p5 = 3,
};

// R17[5:4]
enum class SSUpSel {
  x0p01 = 0,
  x0p02 = 1,
  x0p05 = 2,
  x0p10 = 3,
};

// R17[3:1]
enum class SSDutyChangeLimit {
  percent0p17 = 0,
  percent5p53 = 1,
  percent3p69 = 2,
  percent2p76 = 3,
  percent1p84 = 4,
  percent1p11 = 5,
  percent0p55 = 6,
  percent0p20 = 7,
};

// R17[0:0]
enum class DutyUpTime {
  ms2p7 = 0,
  ms10p8 = 1,
};

// R18[7:5]
enum class RPMChangeLimit {
  noLimitation = 0,
  rpm512 = 1,
  rpm2200 = 2,
  rpm3800 = 3,
  rpm5400 = 4,
  rpm7000 = 5,
  rpm8600 = 6,
  rpm10240 = 7,
};

// R18[2:1]
enum class AnalogFilter {
  none = 0,
  kHz200 = 1,
  kHz100 = 2,
  kHz50 = 3,
};

// R19[2:2]
enum class AlertPinConfig : uint8_t {
  highOnAlert = 0,
  lowOnAlert = 1,
};

// R20[7:5]
enum class RestartTime : uint8_t {
  s0 = 0,
  s0p5 = 1,
  s1 = 2,
  s1p5 = 3,
  s2 = 4,
  s4 = 5,
  s7 = 6,
  s10 = 7,
};

// R20[4:3]
enum class PeriodOf1stDCExcitation {
  s0 = 0,
  s0p2 = 1,
  s0p5 = 2,
  s1 = 3,
};

// R20[2:0]
enum class PeriodOf2ndDCExcitation {
  s0p1 = 0,
  s0p2 = 1,
  s0p4 = 2,
  s0p6 = 3,
  s0p8 = 4,
  s1 = 5,
  s1p5 = 6,
  s2 = 7,
};

// R21[7:4]
enum class LeadAngle : uint8_t {
  deg0 = 0,
  deg3p75 = 1,
  deg7p5 = 2,
  deg11p25 = 3,
  deg15 = 4,
  deg18p75 = 5,
  deg22p5 = 6,
  deg26p25 = 7,
  deg30 = 8,
  case9 = 9,
  case10 = 10,
  case11 = 11,
  case12 = 12,
  case13 = 13,
  case14 = 14,
  case15 = 15,
};

// R21[3:2]
enum class MaxRotationFrequency : uint8_t {
  kHz0p75 = 0,
  kHz1p5 = 1,
  kHz3 = 2,
  none = 3,
};

// R21[1:0]
enum class MinRotationFrequency : uint8_t {
  Hz1p6 = 0,
  Hz3p2 = 1,
  Hz6p4 = 2,
  Hz12p8 = 3,
};

// R22[6:5]
enum class SoftSwitchingAngle : uint8_t {
  deg120 = 0,
  deg135 = 1,
  deg150 = 2,
  deg150forSoft142p5forNotSoft = 3,
};

// R22[4:2]
enum class OutputPWM : uint8_t {
  kHz23p4 = 0,
  kHz46p9 = 1,
  kHz93p7 = 2,
  kHz187p5 = 3,
  case4 = 4,
  case5 = 5,
  case6 = 6,
  case7 = 7,
};

// R22[1:0]
enum class DeadTime : uint8_t {
  ns250 = 0,
  ns500 = 1,
  ns1000 = 2,
  ns1500 = 3,
};

// R23[7:7]
enum class ISDThreshold : uint8_t {
  v1 = 0,
  v0p5 = 1,
};

// R23[6:6]
enum class CurrentLimitVoltage : uint8_t {
  v0p25 = 0,
  v0p125 = 1,
};

// R23[5:3]
enum class SourceCurrent : uint8_t {
  ma10 = 0,
  ma13p9 = 1,
  ma19p3 = 2,
  ma26p8 = 3,
  ma37p3 = 4,
  ma51p8 = 5,
  ma72 = 6,
  ma100 = 7,
};

// R23[2:0]
enum class SincCurrent : uint8_t {
  ma20 = 0,
  ma27p8 = 1,
  ma38p6 = 2,
  ma53p7 = 3,
  ma74p6 = 4,
  ma103p6 = 5,
  ma143p9 = 6,
  ma200 = 7,
};

// R24[7:6]
enum class HysteresisVoltage : uint8_t {
  none = 0,
  mv100 = 1,
  mv200 = 2,
  mv300 = 3,
};

// R86[0:0]
enum class NVMReadWrite : uint8_t {
  read = 0,
  write = 1,
};

}  // namespace ValueTypes
}  // namespace TC78B009FTG_asukiaaa
