/*
   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/RPi_Followetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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

/* 
   RPi optical follow library
*/

#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>
#include <math.h>
#include "AP_RPi_Follow.h"

#define AP_SERIALMANAGER_RPI_BAUD_RATE 115200
#define AP_SERIALMANAGER_RPI_RX_BUFSIZE 64
#define AP_SERIALMANAGER_RPI_TX_BUFSIZE 64

extern const AP_HAL::HAL &hal;

AP_RPi_Follow::AP_RPi_Follow(void)
{
    _port = nullptr;
    _step = 0;
}

/*
 * init - perform required initialisation
 */
bool AP_RPi_Follow::init(const AP_SerialManager &serial_manager)
{
    if (_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_RPi, 0)) //will assign nullptr if not find the port
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_RPI_BAUD_RATE, AP_SERIALMANAGER_RPI_RX_BUFSIZE, AP_SERIALMANAGER_RPI_TX_BUFSIZE);
        return true;
    }
    return false;
}

bool AP_RPi_Follow::update(void)
{
    // check the port had been initialized
    if (_port == nullptr)
        return false;

    uint16_t num_rx = _port->available(); // get the number of available char
    uint8_t data_now = 0;                 // instant data from the uart
    uint8_t pos_x_l = 0;
    uint8_t pos_x_h = 0; // temp storage of pos_x high 8 bits
    uint8_t pos_y_l = 0;
    uint8_t pos_y_h = 0; // temp storage of pos_y high 8 bits
    _step = 0;

    for (uint16_t i = 1; i <= num_rx; i++)
    {
        data_now = _port->read(); // not the frame head and not begin the frame

        switch (_step) // check the step of this frame
        {
        case 0:
            if (data_now == 0x7d)
                _step = 1;
            break;
        case 1: //pos_x_h
            pos_x_h = data_now;
            _step = 2;
            break;
        case 2:
            _step = 3;
            if (data_now == 0x7f) //transition char for 7d,7e,7f
            {
                data_now = _port->read();
                i++;
                switch (data_now)
                {
                case 0x00:
                    pos_x_l = 0x7d;
                    break;
                case 0x01:
                    pos_x_l = 0x7e;
                    break;
                case 0x02:
                    pos_x_l = 0x7f;
                    break;
                default:
                    _step = 0; // the frame meet sth wrong, end this frame
                    break;
                }
            }
            else
            {
                pos_x_l = data_now;
            }
            break;
        case 3:
            pos_y_h = data_now;
            _step = 4;
            break;
        case 4:
            _step = 5;
            if (data_now == 0x7f) //transition char for 7d,7e,7f
            {
                data_now = _port->read();
                i++;
                switch (data_now)
                {
                case 0x00:
                    pos_x_l = 0x7d;
                    break;
                case 0x01:
                    pos_x_l = 0x7e;
                    break;
                case 0x02:
                    pos_x_l = 0x7f;
                    break;
                default:
                    _step = 0; // the frame meet sth wrong, end this frame
                    break;
                }
            }
            else
            {
                pos_y_l = data_now;
            }
            break;
        case 5:
            data_now == (pos_x_h + pos_y_h) ? _step = 6 : _step = 0; // check the sum
            break;
        case 6:
            if (data_now == 0x7e)
            {
                pos_x = (pos_x_h << 8) + pos_x_l;
                pos_y = (pos_y_h << 8) + pos_y_l;
                last_frame_ms = AP_HAL::millis();
                return true;
            }
        default:
            _step = 0;
            break;
        }
    }
    return false;
}

/* 
 * build up the frame's crc
 * for FrSky SPort protocol (X-receivers)
 */
void AP_RPi_Follow::calc_crc(uint8_t byte)
{
    _crc += byte;      //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * for FrSky SPort protocol (X-receivers)
 */
void AP_RPi_Follow::send_crc(void)
{
    send_byte(0xFF - _crc);
    _crc = 0;
}
