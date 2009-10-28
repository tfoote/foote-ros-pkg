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

#include <unistd.h>
#include <stdio.h>
#include <iostream>

#include <phidgetspp.hh>



int main(int argc, char** argv)
{
  RFIDPhidget myRFID;
                                    
  printf("Opening\n");
  myRFID.open(-1);

  printf("Attaching . . . timeout in 30 seconds\n");
  //wait 30 seconds for attachment
  if(myRFID.waitForAttachment(30000))
  {
    printf("ATTACH FAILED\n Make sure device is plugged in and this user has permission\n");
    exit(-1);
  }
  else
    printf("Success Ataching\n");

  printf("Phidget type: %s\n", myRFID.getDeviceType().c_str());
  printf("Phidget label: %s\n", myRFID.getDeviceLabel().c_str());
  printf("Phidget name: %s\n", myRFID.getDeviceName().c_str());
  printf("Phidget Version: %d\n", myRFID.getDeviceVersion());
  printf("Phidget Serial number: %d\n", myRFID.getDeviceSerialNumber());
  printf("Library Version: %s\n", myRFID.getLibraryVersion().c_str());

  printf("Turning ANtenna On\n");
  sleep(1);
  myRFID.setAntennaOn(true);


  printf("Number of outputs is: %u\n", myRFID.getOutputCount());
  myRFID.setOutputState(0, false);
  myRFID.setOutputState(1, false);
  sleep(1);
  printf("Output states 1: %d 2:%d\n", myRFID.getOutputState(0), myRFID.getOutputState(1));
  myRFID.setOutputState(0, true);
  myRFID.setOutputState(1, true);
  printf("Output states 1: %d 2:%d\n", myRFID.getOutputState(0), myRFID.getOutputState(1));

  std::cerr << "antenna is on: " << myRFID.isAntennaOn()<<std::endl;
  //  std::cerr <<myRFID.getType() <<std::endl;;
  //std::cout <<myRFID.getType() <<std::endl;; //FIXME Segfaults!!!

  std::cout << "Blinking LED and waiting until user termination" << std::endl;
  while(true)
    {
      //      printf("HI\n");
      myRFID.setLedOn(true);
      myRFID.setOutputState(0, true);
      myRFID.setOutputState(1, true);
      usleep(10000);
      myRFID.setLedOn(false);
      myRFID.setOutputState(0, false);
      myRFID.setOutputState(1, false);
      usleep(10000);
      std::cerr << "Last Tag seen was: " << myRFID.getLastTag() << std::endl;
      sleep(1);      
    }
  
  return 0;
};
