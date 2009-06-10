/*
 * Copyright (c) 2009, Tully Foote
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <iostream>

#include <sstream>

#include <phidgetspp.hh>

int main(int argc, char* argv[])
{

  TextLCD myLCD;
  myLCD.init(-1);

  printf("connecting to textLCD");
  if (  myLCD.waitForAttachment(5000))
  {
    printf("Attachment failed, check permissions\n");
    exit(-1);
  }
  printf("Phidget type: %s\n", myLCD.getDeviceType().c_str());
  printf("Phidget label: %s\n", myLCD.getDeviceLabel().c_str());
  printf("Phidget name: %s\n", myLCD.getDeviceName().c_str());
  printf("Phidget Version: %d\n", myLCD.getDeviceVersion());
  printf("Phidget Serial number: %d\n", myLCD.getDeviceSerialNumber());
  printf("Library Version: %s\n", myLCD.getLibraryVersion().c_str());
 
  std::cout << "Row count is " << myLCD.getRowCount() << std::endl;
  std::cout << "Column count is " << myLCD.getColumnCount() << std::endl;

  myLCD.setBacklight(PTRUE);
  myLCD.setContrast(128);
  
  std::cout <<"Displaying all characters TOP and BOTTOM" << std::endl;
  for (int r = 0; r < myLCD.getRowCount(); r++)
    for (int i = 0; i < myLCD.getColumnCount(); i++)
    {
      std::stringstream ss;
      for (int c = 0; c < i; c++)
      {
        ss << " ";
      }
      ss << i%10;
      if (r == 0)
      {
        myLCD.setDisplayString(0,ss.str());
        myLCD.setDisplayString(1,"        Top");
      }
      else
      {
        myLCD.setDisplayString(1,ss.str());
        myLCD.setDisplayString(0,"       Bottom");
        myLCD.setBacklight(PTRUE);
      }
      usleep(200000);
    }

  std::cout << "Flashing backlight" << std::endl;
  for (int i = 0 ; i < 4; i++)
  {
    myLCD.setBacklight(PFALSE);
    usleep(200000);
    myLCD.setBacklight(PTRUE);
    myLCD.setDisplayString(i%2,"      Flashing");
    myLCD.setDisplayString(i%2,"");
    usleep(500000);
  }

  std::cout << "Adjusting contrast" << std::endl;

  for (int i = 0; i < 255; i++)
  {
    myLCD.setContrast(i);
    myLCD.setDisplayString(0,"Contrast: ");
    std::stringstream ss;
    ss << i;
    myLCD.setDisplayString(1,ss.str());
    usleep(10000);
  }

  myLCD.setContrast(128);

  myLCD.setDisplayString(0, "This concludes the");
  myLCD.setDisplayString(1, "LCD Phidget Demo");
  return 0;
}

