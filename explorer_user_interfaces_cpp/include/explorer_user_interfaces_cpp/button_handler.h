#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono;

namespace space_control
{
    class ButtonHandler {
        public:
            ButtonHandler();
    
            void init(int threshold_ms);

            void update(bool button_actual);
        
            bool isShortClick();

            bool isLongClick();

        private:
            int threshold_ms_;
            mutable std::mutex mutex_click_;
            bool short_click_ RCPPUTILS_TSA_GUARDED_BY(mutex_click_) = 0;
            bool long_click_ RCPPUTILS_TSA_GUARDED_BY(mutex_click_) = 0;
            bool button_prev_;
            std::chrono::steady_clock::time_point start_;
    };

}