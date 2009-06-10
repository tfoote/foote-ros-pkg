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

class DemoText
{
public:
  std_msgs::String stringMsg_;
  DemoText() {
  };
  void init();
  void demo();
private:
  TextLCD myLCD;
};


void DemoText::init()
{
  myLCD.init(-1);
  myLCD.waitForAttachment(5000);
};

void DemoText::demo()
{
  myLCD.setContrast(128);
  myLCD.setBacklight(PTRUE);
  std::cout<<"Starting Demo"<<std::endl;
  sleep(1);
  
  myLCD.setDisplayString(0, "Welcome to Text LCD");
  myLCD.setDisplayString(1, "Demo");  
  sleep(1);
  myLCD.setBacklight(PTRUE);
  myLCD.setDisplayString(0, "Backlight ON");
  myLCD.setDisplayString(1, "");
  sleep(1);
  myLCD.setBacklight(PFALSE);
  myLCD.setDisplayString(0, "Backlight OFF");
  sleep(1);

  myLCD.setBacklight(PTRUE);

  for (unsigned int i = 0; i < 256; i++)
    {
      myLCD.setContrast(i);
      std::stringstream ss;
      ss << i << " contrast";
      myLCD.setDisplayString(0,ss.str() );
      usleep(10000);
    }
  
  myLCD.setContrast(128);
  
  
};
/*
int main(int argc, char* argv[])
{
  ros::init(argc, argv);
  DemoText myText;
  myText.init();  
  myText.demo();  
  
    
  return 0;
}
*/
#include <sstream>

#include <ctime>

#include <phidgetspp.hh>
#include <ros/node.h>
#include <std_msgs/String.h>

class SendString 
{
public:
  std_msgs::String stringMsg_;
  SendString():node_("send_string") { 
    stringMsg_.data = std::string("inited"); 
    node_.advertise<std_msgs::String>("string", 1);};
  
  
  ros::Node node_;

};


int main(int argc, char* argv[])
{
  ros::init(argc, argv);
  
  SendString mySender;

  if (argc == 2)
    {
      usleep(500000);
      
      mySender.stringMsg_.data= std::string(argv[1]);
      
      mySender.node_.publish("string", mySender.stringMsg_);
      std::cout << "Sent " << mySender.stringMsg_.data<<std::endl;
      usleep(500000);
    }
  
  else
    std::cout <<"Wrong usage";
  

  return 0;
}


