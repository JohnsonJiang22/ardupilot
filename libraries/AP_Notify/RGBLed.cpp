/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include "RGBLed.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

// 此函数是用来初始化LED的不同状态（关闭、亮、适中和暗）
// 例如_led_off(led_off)的作用是将传入构造函数的参数led_off的值赋给类的成员变量_led_off
RGBLed::RGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim):
    _led_off(led_off), // 关闭
    _led_bright(led_bright), // 亮
    _led_medium(led_medium), // 适中
    _led_dim(led_dim) // 暗
{

}

// set_rgb - set color as a combination of red, green and blue values
// 检查输入的RGB值是否和当前设置的颜色不同，如果不同则调用hw_set_rgb（一个底层硬件更新函数）更新硬件，并更新当前颜色值。这样做避免了不必要的硬件调用
void RGBLed::_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red != _red_curr ||
        green != _green_curr ||
        blue != _blue_curr) {
        // call the hardware update routine
        if (hw_set_rgb(red, green, blue)) {
            _red_curr = red;
            _green_curr = green;
            _blue_curr = blue;
        }
    }
}

// 返回当前RGB LED的颜色来源类型，比如标准、MAVLink、交通灯模式等。pNotify是一个通知对象，包含了当前系统状态
RGBLed::rgb_source_t RGBLed::rgb_source() const
{
    return rgb_source_t(pNotify->_rgb_led_override.get());
}

// set_rgb - set color as a combination of red, green and blue values
// 如果RGB LED的来源是MAVLink（意味着外部命令覆盖），则不执行颜色设置；否则调用_set_rgb设置颜色
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (rgb_source() == mavlink) {
        // don't set if in override mode
        return;
    }
    _set_rgb(red, green, blue);
}

// 根据内部状态和外部条件（如是否通过USB连接），计算并返回当前应该设置的LED亮度值。它通过查阅pNotify对象中的亮度设置以及其他条件（如USB连接状态），来确定具体的亮度值。
// 定义了一个局部变量brightness，初始值设置为类成员变量_led_bright的值，这个变量代表最亮的亮度。
// pNotify是一个指向某个对象的指针，该对象包含有关于LED亮度的信息。_rgb_led_brightness是这个对象的成员变量，表示当前LED的亮度设置
uint8_t RGBLed::get_brightness(void) const
{
    uint8_t brightness = _led_bright;

    switch (pNotify->_rgb_led_brightness) {
    case RGB_LED_OFF:
        brightness = _led_off;
        break;
    case RGB_LED_LOW:
        brightness = _led_dim;
        break;
    case RGB_LED_MEDIUM:
        brightness = _led_medium;
        break;
    case RGB_LED_HIGH:
        brightness = _led_bright;
        break;
    }

    // use dim light when connected through USB
    if (hal.gpio->usb_connected() && brightness > _led_dim) {
        brightness = _led_dim;
    }
    return brightness;
}

// 返回OBC模式下的颜色序列，主要根据系统是否ARM（武装）状态设置为红色或绿色
uint32_t RGBLed::get_colour_sequence_obc(void) const
{
    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SOLID(RED);
    }
    return DEFINE_COLOUR_SEQUENCE_SOLID(GREEN);
}

// _scheduled_update - updates _red, _green, _blue according to notify flags
// 通过读取不同的系统状态标志（如初始化、校准、故障、武装状态等），决定LED的颜色显示模式。每种模式对应不同的颜色序列，通常用于向用户提供关于系统状态的视觉反馈
uint32_t RGBLed::get_colour_sequence(void) const
{
    // initialising pattern
    if (AP_Notify::flags.initialising) {
        return sequence_initialising;
    }

    // save trim or any calibration pattern
    if (AP_Notify::flags.save_trim ||
        AP_Notify::flags.esc_calibration ||
        AP_Notify::flags.compass_cal_running ||
        AP_Notify::flags.temp_cal_running) {
        return sequence_trim_or_esc;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio ||
        AP_Notify::flags.failsafe_gcs ||
        AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.ekf_bad ||
        AP_Notify::flags.gps_glitching ||
        AP_Notify::flags.leak_detected) {

        if (AP_Notify::flags.leak_detected) {
            // purple if leak detected
            return sequence_failsafe_leak;
        } else if (AP_Notify::flags.ekf_bad) {
            // red on if ekf bad
            return sequence_failsafe_ekf;
        } else if (AP_Notify::flags.gps_glitching) {
            // blue on gps glitch
            return sequence_failsafe_gps_glitching;
        }
        // all off for radio or battery failsafe
        return sequence_failsafe_radio_or_battery;
    }

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            return sequence_armed;
        }
        // solid blue if armed with no GPS lock
        return sequence_armed_nogps;
    }

    // double flash yellow if failing pre-arm checks
    if (!AP_Notify::flags.pre_arm_check) {
        return sequence_prearm_failing;
    }
    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_dgps;
    }

    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_gps;
    }

    return sequence_disarmed_bad_gps;
}

// 返回“交通灯”模式的颜色序列，用于指示初始状态、ARM状态和安全开关状态等
uint32_t RGBLed::get_colour_sequence_traffic_light(void) const
{
    if (AP_Notify::flags.initialising) {
        return DEFINE_COLOUR_SEQUENCE(RED,GREEN,BLUE,RED,GREEN,BLUE,RED,GREEN,BLUE,BLACK);
    }

    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SLOW(RED);
    }

    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        if (!AP_Notify::flags.pre_arm_check) {
            return DEFINE_COLOUR_SEQUENCE_ALTERNATE(YELLOW, BLACK);
        } else {
            return DEFINE_COLOUR_SEQUENCE_SLOW(YELLOW);
        }
    }

    if (!AP_Notify::flags.pre_arm_check) {
        return DEFINE_COLOUR_SEQUENCE_ALTERNATE(GREEN, BLACK);
    }
    return DEFINE_COLOUR_SEQUENCE_SLOW(GREEN);
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
// 根据当前的LED模式选择合适的颜色序列，然后根据时间步长和亮度设置LED的颜色。通过这种方式，可以实现多种不同的LED显示效果，例如根据系统状态显示不同的颜色，
// 或者在交通信号灯模式下显示不同的颜色序列
void RGBLed::update() // RGBLed::: 这是 RGBLed 类的成员函数
{
    uint32_t current_colour_sequence = 0; // 存储当前颜色序列的编码

    switch (rgb_source()) { // 根据 rgb_source() 函数返回的LED模式来选择不同的更新逻辑
    case mavlink: // 如果当前模式是 MAVLink，那么调用 update_override() 函数更新LED状态，然后返回（终止函数执行），不继续后面的代码
        update_override();
        return; // note this is a return not a break!
    case standard: // 如果当前模式是标准模式，调用 get_colour_sequence() 函数获取颜色序列，并将其赋值给 current_colour_sequence
        current_colour_sequence = get_colour_sequence();
        break;
    case obc: // 如果当前模式是 OBC（机载计算机），调用 get_colour_sequence_obc() 函数获取颜色序列，并将其赋值给 current_colour_sequence
        current_colour_sequence = get_colour_sequence_obc();
        break;
    case traffic_light: // 如果当前模式是交通信号灯模式，调用 get_colour_sequence_traffic_light() 函数获取颜色序列，并将其赋值给 current_colour_sequence
        current_colour_sequence = get_colour_sequence_traffic_light();
        break;
    }

    const uint8_t brightness = get_brightness(); // 获取亮度：返回当前的亮度值，根据系统状态（如是否通过USB连接）来调整亮度

    uint8_t step = (AP_HAL::millis()/100) % 10; // 计算时间步长：%10操作的含义是：对结果取模10，将步长限制在0到9之间，用于确定当前显示的颜色

    // ensure we can't skip a step even with awful timing
    if (step != last_step) { // 如果当前步长与上一个步长不同
        step = (last_step+1) % 10; // 计算下一个步长，并将其限制在0到9之间，防止跳过步骤
        last_step = step; // 更新上一个步长为当前步长
    }

    // 计算颜色：将颜色序列右移 (step*3) 位，以提取当前步长对应的颜色。每个颜色占3位（RGB），因此移动的位数为步长乘以3
    // & 7: 将结果与7（00000111）进行按位与操作，提取出当前颜色的值
    const uint8_t colour = (current_colour_sequence >> (step*3)) & 7; 

    // 更新LED颜色
    uint8_t red_des = (colour & RED) ? brightness : _led_off; // 目标红色分量。如果 colour & RED 为真（非零），则设置为当前亮度 brightness，否则设置为 _led_off（LED关闭）
    uint8_t green_des = (colour & GREEN) ? brightness : _led_off;
    uint8_t blue_des = (colour & BLUE) ? brightness : _led_off;

    set_rgb(red_des, green_des, blue_des);
}

/*
  handle LED control, only used when LED_OVERRIDE=1
*/
// 处理来自 MAVLink 消息的 LED 控制命令，如果当前模式为 MAVLink 则解码并设置 LED 的控制参数
void RGBLed::handle_led_control(const mavlink_message_t &msg)
{
    if (rgb_source() != mavlink) {
        // ignore LED_CONTROL commands if not in LED_OVERRIDE mode
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(&msg, &packet);

    _led_override.start_ms = AP_HAL::millis();

    switch (packet.custom_len) {
    case 3:
        _led_override.rate_hz = 0;
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    case 4:
        _led_override.rate_hz = packet.custom_bytes[3];
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    default:
        // not understood
        break;
    }
}

/*
  update LED when in override mode
 */
// 根据 rate_hz 和当前时间更新 LED 的状态，支持闪烁和固态显示
void RGBLed::update_override(void)
{
    if (_led_override.rate_hz == 0) {
        // solid colour
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
        return;
    }
    // blinking
    uint32_t ms_per_cycle = 1000 / _led_override.rate_hz;
    uint32_t cycle = (AP_HAL::millis() - _led_override.start_ms) % ms_per_cycle;
    if (cycle > ms_per_cycle / 2) {
        // on
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
    } else {
        _set_rgb(0, 0, 0);
    }
}

/*
  RGB control
  give RGB and flash rate, used with scripting
*/
// 设置 LED 的 RGB 颜色和闪烁频率，用于脚本控制或外部指令
void RGBLed::rgb_control(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz)
{
    _led_override.rate_hz = rate_hz;
    _led_override.r = r;
    _led_override.g = g;
    _led_override.b = b;
}
