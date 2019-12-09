/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_RPi_Follow
{
public:
    AP_RPi_Follow();

    /* Do not allow copies */
    AP_RPi_Follow(const AP_RPi_Follow &other) = delete;
    AP_RPi_Follow &operator=(const AP_RPi_Follow &) = delete;

    // init - perform required initialisation
    bool init(const AP_SerialManager &serial_manager);

    // update - perform regular data update
    bool update(void);

    //last time of receiving data
    uint32_t last_frame_ms;

private:
    AP_HAL::UARTDriver *_port; // UART used to send data to FrSky receiver

    uint16_t _crc;

    uint32_t check_sensor_status_timer;
    uint32_t check_ekf_status_timer;
    uint8_t _paramID;

    // the position of target. range from 0-1000
    uint16_t pos_x;
    uint16_t pos_y;
    // the instant step of the data fram
    uint16_t _step;

    //function for CRC check
    void calc_crc(uint8_t byte);
    void send_crc(void);
    void send_byte(uint8_t byte);
};
