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

#include "IQRFservo.h"


IQRFservo::IQRFservo ()
{
}

IQRFservo::~IQRFservo ()
{
  this -> servo.detach () ;
}

void IQRFservo::init ( int pinNr )
{ 
  this -> servo.attach ( pinNr ) ;
}

void IQRFservo::setAngle ( byte angle, int delayTime )
{
  // determine current position of the servo
  int prevPos = getAngle () ;
  int i = 0 ;

  // if the requested position is higher than the current one, increase angle
  if (prevPos < angle)
  {
    // tell servo to go to position given by the variable 'angle'
    for (i = prevPos; i <= angle; i += 1)  // in steps of 1 degree
    {
      this -> servo.write (i);  
      delay ( delayTime );  // waits "delayTime" for the servo to reach the position
    }
    return ;
  }

  // if the requested position is lower than the current one, decrease angle
  if (prevPos > angle)
  {
    // tell servo to go to position given by the variable 'angle'
    for (i = prevPos; i >= angle; i -= 1)  // in steps of 1 degree
    {
      this -> servo.write (i);  
      delay ( delayTime );  // waits "delayTime" for the servo to reach the position
    }
    return ;
  }
}

byte IQRFservo::getAngle ()
{
  return this -> servo.read() ;
}
