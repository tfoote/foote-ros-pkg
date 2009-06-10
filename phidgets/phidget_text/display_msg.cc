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

#include <sstream>

#include <ctime>
#include <sstream>

#include <phidgetspp.hh>
#include <ros/node.h>
#include <std_msgs/String.h>

class RecieveString 
{
public:
  RecieveString():node_("Phidget_text_display") {
    node_.subscribe("string", stringMsg_, &RecieveString::stringRecieved, this, 10);
  };
  void init();
  ros::Node node_;
private:
  void stringRecieved() {
    myLCD.setDisplayString(0, lastString_);
    myLCD.setDisplayString(1, stringMsg_.data);
    std::cout << "Recieved: " << stringMsg_.data << std::endl;
    lastString_ = stringMsg_.data;
  };
  TextLCD myLCD;
  std::string lastString_;
  std_msgs::String stringMsg_;
};


void RecieveString::init()
{
  myLCD.init(-1);
  myLCD.waitForAttachment(5000);
  myLCD.setDisplayString(0, "Welcome to Recieve-");
  myLCD.setDisplayString(1, "String.  Waiting...");  
  myLCD.setBacklight(PTRUE);  
  myLCD.setContrast(128);
  
  
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv);
  RecieveString myString;
  myString.init();  
  
  while (myString.node_.ok())
    {
      usleep(100000);
    }
    
  return 0;
}

