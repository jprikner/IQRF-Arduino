/* 
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

#include "IQRFbattery.h"

IQRFbattery::IQRFbattery ()
{
    batt = 0;
    battLow = 0;
    battHigh = 0;
}

IQRFbattery::~IQRFbattery ()
{
}

void IQRFbattery::checkBatteryLevel ()
{
  batt = 3375;

  formatLowVoltage () ;
}

void IQRFbattery::formatLowVoltage  ()
{
  // written using some info about bit operations from: https://www.codesdope.com/blog/article/set-toggle-and-clear-a-bit-in-c/
    unsigned short i;
    unsigned short low = (batt % 1000) / 62.5;
    unsigned short high = batt / 1000;

    // integral part
    int x = 7;

    for (i = 1 << 11; i > 0; i = i / 2)
    {
        if ( i > 8 )  // write all 8 bits of upper byte
          if ( high & i ) 
            battHigh |= 1 << x;

        if ( i <= 8 ) // write upper 4 bits of lower byte
          if ( i == 8 ) x = 7;  // re-set x
            if ( high & i ) battLow |= 1 << x;

        x--;
    }
    
    // fractal part
    for ( i = 1 << 3; i > 0; i = i / 2 )
    {
        if ( low & i ) 
          battLow |= 1 << x ;

        x--;
    }
}
