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

int main(int argc, char* argv[])
{
  ros::init(argc, argv);
  DemoText myText;
  myText.init();  
  myText.demo();  
  
    
  return 0;
}

