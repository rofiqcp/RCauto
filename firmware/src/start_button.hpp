#pragma once

#include <stdint.h>

enum class ButtonEvent { None, ShortPress, LongPress };

class StartButton {
 public:
  static constexpr uint32_t kDebounceMs = 45;
  static constexpr uint32_t kLongPressMs = 1200;

  void begin(bool pressed, uint32_t now) {
    initialized_ = true;
    rawPressed_ = pressed;
    stablePressed_ = pressed;
    readyForPress_ = !pressed;
    validPress_ = false;
    longHandled_ = false;
    changedAtMs_ = now;
    pressedAtMs_ = now;
  }

  ButtonEvent update(bool pressed, uint32_t now) {
    if (!initialized_) {
      begin(pressed, now);
      return ButtonEvent::None;
    }

    if (pressed != rawPressed_) {
      rawPressed_ = pressed;
      changedAtMs_ = now;
    }

    if (stablePressed_ != rawPressed_ &&
        now - changedAtMs_ >= kDebounceMs) {
      stablePressed_ = rawPressed_;
      if (stablePressed_) {
        pressedAtMs_ = now;
        longHandled_ = false;
        validPress_ = readyForPress_;
      } else {
        if (!readyForPress_) {
          readyForPress_ = true;
          return ButtonEvent::None;
        }
        if (validPress_ && !longHandled_) {
          validPress_ = false;
          if (now - pressedAtMs_ >= kLongPressMs) {
            longHandled_ = true;
            return ButtonEvent::LongPress;
          }
          return ButtonEvent::ShortPress;
        }
        validPress_ = false;
      }
    }

    if (stablePressed_ && rawPressed_ && validPress_ &&
        !longHandled_ && now - pressedAtMs_ >= kLongPressMs) {
      longHandled_ = true;
      return ButtonEvent::LongPress;
    }
    return ButtonEvent::None;
  }

  void cancelCurrentPress() { validPress_ = false; }

  bool pressed() const { return stablePressed_; }

  bool longStopHeld() const {
    return stablePressed_ && longHandled_;
  }

 private:
  bool initialized_ = false;
  bool rawPressed_ = false;
  bool stablePressed_ = false;
  bool readyForPress_ = false;
  bool validPress_ = false;
  bool longHandled_ = false;
  uint32_t changedAtMs_ = 0;
  uint32_t pressedAtMs_ = 0;
};
