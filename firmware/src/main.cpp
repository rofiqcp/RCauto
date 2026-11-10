#include <Arduino.h>
#include <ESP32Servo.h>
#include "start_button.hpp"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "hal/gpio_ll.h"

namespace {
constexpr uint8_t kSteeringPin = 13;
constexpr uint8_t kForwardPin = 14;
constexpr uint8_t kReversePin = 27;
constexpr uint8_t kStartButtonPin = 12;
constexpr uint8_t kForwardChannel = 8;
constexpr uint8_t kReverseChannel = 9;
constexpr uint32_t kMotorPwmHz = 20000;
constexpr uint8_t kPwmResolution = 10;
constexpr int kMaxDuty = (1 << kPwmResolution) - 1;
constexpr int kSteeringLimit = 900;
constexpr int kSpeedLimit = 1000;
constexpr int kSteeringCenterUs = 1500;
constexpr int kSteeringSpanUs = 500;
constexpr uint32_t kWatchdogMs = 500;
constexpr uint32_t kDirectionPauseMs = 300;
constexpr uint32_t kRampPeriodMs = 20;
constexpr int kRampStep = 30;

Servo steeringServo;
bool armed = false;
bool estopLatched = false;
bool neutralReceived = false;
int steeringCommand = 0;
int speedTarget = 0;
int speedApplied = 0;
uint32_t lastDriveMs = 0;
uint32_t lastRampMs = 0;
uint32_t directionAllowedAt = 0;
StartButton startButton;
bool longStopEligible = false;
char rxLine[64] = {};
size_t rxLength = 0;
bool overflow = false;

void writeMotor(int signedSpeed) {
  const int duty = abs(signedSpeed) * kMaxDuty / kSpeedLimit;
  // Never command opposite half-bridges simultaneously.
  if (signedSpeed >= 0) {
    ledcWrite(kReverseChannel, 0);
    ledcWrite(kForwardChannel, duty);
  } else {
    ledcWrite(kForwardChannel, 0);
    ledcWrite(kReverseChannel, duty);
  }
}

void stopMotor() {
  speedTarget = 0;
  speedApplied = 0;
  writeMotor(0);
}

void setArmed(bool enable, const char* source) {
  if (enable && estopLatched) {
    Serial.println("ERR,ESTOP");
    return;
  }
  if (enable && startButton.longStopHeld()) {
    Serial.println("ERR,BUTTON_HELD");
    return;
  }
  if (armed == enable) {
    Serial.printf("ARMED,%d,%s\n", armed ? 1 : 0, source);
    return;
  }
  stopMotor();
  neutralReceived = false;
  armed = enable;
  Serial.printf("ARMED,%d,%s\n", armed ? 1 : 0, source);
}

void emergencyStop() {
  stopMotor();
  armed = false;
  neutralReceived = false;
  estopLatched = true;
  Serial.println("ESTOP,1");
}

void resetEstop() {
  if (armed) {
    Serial.println("ERR,ARMED");
    return;
  }
  estopLatched = false;
  Serial.println("ESTOP,0");
}

int steeringMicroseconds(int steering) {
  return kSteeringCenterUs +
         steering * kSteeringSpanUs / kSteeringLimit;
}

void applyDrive(int steering, int throttle) {
  steeringCommand = steering;
  steeringServo.writeMicroseconds(steeringMicroseconds(steering));
  lastDriveMs = millis();
  if (throttle == 0) {
    neutralReceived = true;
    stopMotor();
    return;
  }
  speedTarget = (armed && !estopLatched && neutralReceived) ? throttle : 0;
}

bool parseInt(const char* text, int minimum, int maximum, int& result) {
  if (!*text) return false;
  errno = 0;
  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (errno == ERANGE || *end || value < minimum || value > maximum) {
    return false;
  }
  result = static_cast<int>(value);
  return true;
}

int hexValue(char character) {
  if (character >= '0' && character <= '9') return character - '0';
  if (character >= 'A' && character <= 'F') return character - 'A' + 10;
  if (character >= 'a' && character <= 'f') return character - 'a' + 10;
  return -1;
}

void handleFrame(char* frame) {
  if (frame[0] != '@') {
    Serial.println("ERR,FRAME");
    return;
  }
  char* separator = strchr(frame, '*');
  if (!separator || separator[1] == '\0' || separator[2] == '\0' ||
      separator[3] != '\0') {
    Serial.println("ERR,FRAME");
    return;
  }
  const int hi = hexValue(separator[1]);
  const int lo = hexValue(separator[2]);
  if (hi < 0 || lo < 0) {
    Serial.println("ERR,CRC");
    return;
  }
  uint8_t crc = 0;
  for (char* p = frame + 1; p < separator; ++p) crc ^= uint8_t(*p);
  if (crc != uint8_t((hi << 4) | lo)) {
    Serial.println("ERR,CRC");
    return;
  }
  *separator = '\0';
  char* body = frame + 1;

  if (strncmp(body, "RC,", 3) == 0) {
    char* steeringText = body + 3;
    char* speedText = strchr(steeringText, ',');
    if (!speedText || strchr(speedText + 1, ',')) {
      Serial.println("ERR,FRAME");
      return;
    }
    *speedText++ = '\0';
    int steer = 0, speed = 0;
    if (!parseInt(steeringText, -kSteeringLimit, kSteeringLimit, steer) ||
        !parseInt(speedText, -kSpeedLimit, kSpeedLimit, speed)) {
      Serial.println("ERR,RANGE");
      return;
    }
    applyDrive(steer, speed);
    Serial.printf("ACK,%d,%d\n", steer, speed);
  } else if (strcmp(body, "ARM,1") == 0) {
    setArmed(true, "HOST");
  } else if (strcmp(body, "ARM,0") == 0) {
    startButton.cancelCurrentPress();
    setArmed(false, "HOST");
  } else if (strcmp(body, "ESTOP,1") == 0) {
    emergencyStop();
  } else if (strcmp(body, "RESET,1") == 0) {
    resetEstop();
  } else {
    Serial.println("ERR,FRAME");
  }
}

void sendStatus() {
  Serial.printf("STATUS,steer=%d,speed=%d,target=%d,forward_duty=%d,"
                "reverse_duty=%d,armed=%d,estop=%d,button=%d,watchdog_ms=%lu\n",
                steeringCommand, speedApplied, speedTarget,
                speedApplied > 0 ? speedApplied * kMaxDuty / kSpeedLimit : 0,
                speedApplied < 0 ? -speedApplied * kMaxDuty / kSpeedLimit : 0,
                armed ? 1 : 0, estopLatched ? 1 : 0,
                startButton.pressed() ? 1 : 0,
                static_cast<unsigned long>(kWatchdogMs));
}

void measurePwm() {
  gpio_ll_input_enable(&GPIO, static_cast<gpio_num_t>(kSteeringPin));
  const unsigned long high = pulseIn(kSteeringPin, HIGH, 50000);
  const unsigned long low = pulseIn(kSteeringPin, LOW, 50000);
  Serial.printf("PWM,pin=13,high_us=%lu,period_us=%lu\n", high, high + low);
  Serial.printf("PWM_CONFIG,pin=14,freq_hz=%lu,duty=%d\n",
                static_cast<unsigned long>(kMotorPwmHz),
                speedApplied > 0 ? speedApplied * kMaxDuty / kSpeedLimit : 0);
  Serial.printf("PWM_CONFIG,pin=27,freq_hz=%lu,duty=%d\n",
                static_cast<unsigned long>(kMotorPwmHz),
                speedApplied < 0 ? -speedApplied * kMaxDuty / kSpeedLimit : 0);
}

void handleLine() {
  rxLine[rxLength] = '\0';
  if (overflow) Serial.println("ERR,LENGTH");
  else if (rxLength && rxLine[0] == '@') handleFrame(rxLine);
  else if (strcmp(rxLine, "STATUS") == 0) sendStatus();
  else if (strcmp(rxLine, "PWM") == 0) measurePwm();
  else if (strcmp(rxLine, "STOP") == 0) {
    stopMotor();
    Serial.println("ACK,STOP");
  } else if (rxLength) Serial.println("ERR,FRAME");
  rxLength = 0;
  overflow = false;
}

void receiveSerial() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n' || character == '\r') {
      if (rxLength || overflow) handleLine();
    } else if (!overflow && rxLength < sizeof(rxLine) - 1) {
      rxLine[rxLength++] = character;
    } else {
      overflow = true;
    }
  }
}

void pollStartButton() {
  const ButtonEvent event = startButton.update(
      digitalRead(kStartButtonPin) == HIGH, millis());
  if (event == ButtonEvent::Pressed) {
    longStopEligible = armed;
    Serial.println("BUTTON,PRESSED");
    if (!armed) setArmed(true, "BUTTON");
  } else if (event == ButtonEvent::LongPress) {
    Serial.println("BUTTON,LONG");
    if (longStopEligible && armed) setArmed(false, "BUTTON");
    longStopEligible = false;
  }
  if (!startButton.pressed()) longStopEligible = false;
}

void updateMotor() {
  const uint32_t now = millis();
  if (speedTarget && now - lastDriveMs > kWatchdogMs) {
    stopMotor();
    Serial.println("SAFE_STOP,WATCHDOG");
  }
  if (!armed || estopLatched) {
    if (speedApplied) stopMotor();
    return;
  }
  if (now - lastRampMs < kRampPeriodMs) return;
  lastRampMs = now;

  if (speedApplied && speedTarget &&
      ((speedApplied > 0) != (speedTarget > 0))) {
    speedApplied = 0;
    writeMotor(0);
    directionAllowedAt = now + kDirectionPauseMs;
    return;
  }
  if (static_cast<int32_t>(now - directionAllowedAt) < 0) return;

  speedApplied += constrain(speedTarget - speedApplied,
                            -kRampStep, kRampStep);
  writeMotor(speedApplied);
}
}  // namespace

void setup() {
  // GPIO12: button is active HIGH; external 10k pulldown keeps boot strap LOW.
  pinMode(kStartButtonPin, INPUT_PULLDOWN);
  startButton.begin(digitalRead(kStartButtonPin) == HIGH, millis());
  pinMode(kForwardPin, OUTPUT);
  pinMode(kReversePin, OUTPUT);
  digitalWrite(kForwardPin, LOW);
  digitalWrite(kReversePin, LOW);

  Serial.begin(115200);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(kSteeringPin, 1000, 2000);
  steeringServo.writeMicroseconds(kSteeringCenterUs);
  if (!ledcSetup(kForwardChannel, kMotorPwmHz, kPwmResolution) ||
      !ledcSetup(kReverseChannel, kMotorPwmHz, kPwmResolution)) {
    Serial.println("ERR,PWM_INIT");
    while (true) delay(1000);
  }
  ledcAttachPin(kForwardPin, kForwardChannel);
  ledcAttachPin(kReversePin, kReverseChannel);
  stopMotor();
  Serial.println("READY,BTS7960,STEER=-900..900,SPEED=-1000..1000,ARM=0");
  sendStatus();
}

void loop() {
  receiveSerial();
  pollStartButton();
  updateMotor();
  delay(1);
}
