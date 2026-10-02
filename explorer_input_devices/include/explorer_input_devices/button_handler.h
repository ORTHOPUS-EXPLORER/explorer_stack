#ifndef EXPLORER_INPUT_DEVICES_BUTTON_HANDLER_H
#define EXPLORER_INPUT_DEVICES_BUTTON_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono;

namespace input_device
{
    constexpr auto button_default_threshold_ms = 500;

    class ButtonHandler {
        public:
            ButtonHandler() = default;
    
            void init(int threshold_ms);

            void update(bool button_actual);
        
            bool is_short_click();

            bool is_long_click();

        private:
            int threshold_ms_ = button_default_threshold_ms;
            mutable std::mutex mutex_click_;
            bool short_click_ RCPPUTILS_TSA_GUARDED_BY(mutex_click_) = false;
            bool long_click_ RCPPUTILS_TSA_GUARDED_BY(mutex_click_) = false;
            bool button_prev_ = false;
            std::chrono::steady_clock::time_point start_;
    };

}

#endif