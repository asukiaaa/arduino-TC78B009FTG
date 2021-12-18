#include <TC78B009FTG_asukiaaa.hpp>

#define PIN_MOTOR_STDBY 5
#define PIN_MOTOR_SPD 15

namespace Registers = TC78B009FTG_asukiaaa::Registers;
namespace ValueTypes = TC78B009FTG_asukiaaa::ValueTypes;

TC78B009FTG_asukiaaa::CoreI2c motor(
    TC78B009FTG_asukiaaa::DeviceAddresses::InitialOnRegister, PIN_MOTOR_STDBY);

void setup() {
  Serial.begin(115200);
  motor.begin();

  Registers::R06to07 r06;
  motor.readRegister(&r06);
  r06.startRpm = 1;
  motor.writeRegister(r06);

  Registers::R15 r15;
  motor.readRegister(&r15);
  r15.speedInputSelect = ValueTypes::SpeedInputSelect::pwm;
  // r15.speedInv = ValueTypes::SpeedInv::positive;
  motor.writeRegister(r15);

  Registers::R19 r19;
  motor.readRegister(&r19);
  r19.alertPinConfig = ValueTypes::AlertPinConfig::lowOnAlert;
  motor.writeRegister(r19);

  pinMode(PIN_MOTOR_SPD, OUTPUT);
  digitalWrite(PIN_MOTOR_SPD, LOW);
}

void loop() {
  // Serial.println("motor state");
  // motor.printRegisters(&Serial);
  // Serial.println("at " + String(millis()));
  // delay(2000);

  digitalWrite(PIN_MOTOR_SPD, HIGH);
  Serial.println("run for 2 seconds");
  delay(2000);

  digitalWrite(PIN_MOTOR_SPD, LOW);
  Serial.println("stop");
  delay(2000);
}
