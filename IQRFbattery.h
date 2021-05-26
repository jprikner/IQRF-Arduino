/* 
 * IQRFbattery library purposed for battery level check at the battery 
 * operated systems - version 1.0
 * Copyright (c) 2021 Jakub Prikner <jakub@prikner.net>
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  
 * @license GPL-3.0+ <https://spdx.org/licenses/GPL-3.0-or-later.html>
 */

#ifndef IQRFbattery_h
#define IQRFbattery_h

// include necessary libraries
#include "stdint.h"
#include "Arduino.h"

class IQRFbattery
{
private:
    // format measured voltage to the IQRF-specific LowVoltage type of sensor [2B]
    void formatLowVoltage  () ;

public:
    IQRFbattery  ();
    ~IQRFbattery ();

    // performs a battery voltage check and stores its value
    void checkBatteryLevel () ;

    // measured voltage [in mV]
    unsigned short batt ;
    
    // IQRF format battery voltage level value [2B]
    byte battLow ;  // first byte of battery voltage level (with 4 bits fractional part)
    byte battHigh ; // second byte of battery voltage level 
};

#endif