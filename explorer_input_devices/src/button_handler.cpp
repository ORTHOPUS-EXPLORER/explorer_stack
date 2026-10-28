#include "explorer_input_devices/button_handler.h"

namespace input_device
{
void ButtonHandler::init(int threshold_ms) { threshold_ms_ = threshold_ms; }

void ButtonHandler::update(bool button_actual)
{
  auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock_click(mutex_click_);
  if (!button_prev_ && button_actual)
  {
    start_ = now;
  }
  else if (
    button_prev_ && button_actual &&
    (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count() > threshold_ms_))
  {
    long_click_ = true;
  }
  else if (button_prev_ && !button_actual)
  {
    if (
      (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count() <=
       threshold_ms_))
    {
      short_click_ = true;
    }
    long_click_ = false;
  }

  button_prev_ = button_actual;
}

bool ButtonHandler::is_short_click()
{
  std::lock_guard<std::mutex> lock_click(mutex_click_);
  bool val = short_click_;
  short_click_ = false;

  return val;
}

bool ButtonHandler::is_long_click()
{
  std::lock_guard<std::mutex> lock_click(mutex_click_);
  return long_click_;
}
}  // namespace input_device