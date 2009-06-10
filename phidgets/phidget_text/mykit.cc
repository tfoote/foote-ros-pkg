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

int mval = 1000;

int IFK_DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	printf("Detach handler ran!\n");
	return 0;
}

int IFK_ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Attach handler ran!\n");
	return 0;
}

int IFK_OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr, int Index, int Value)
{
	printf("Output %d is %d\n", Index, Value);
	return 0;
}

int IFK_InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr, int Index, int Value)
{
	printf("Input %d is %d\n", Index, Value);
	return 0;
}

int IFK_SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr, int Index, int Value)
{
	printf("Sensor %d is %d\n", Index, Value);
	if (Index == 0)
	  mval = Value;
	return 0;
}


int IFK_AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	printf("Attach handler ran!\n");
	return 0;
}


class InterfaceKitPhidget : public Phidget
{
public:
  InterfaceKitPhidget();
  bool init(int serlai_number);  


  CPhidgetInterfaceKitHandle & getIFK(){return IFK;};

private:
  CPhidgetInterfaceKitHandle IFK;
  
  
};

InterfaceKitPhidget::InterfaceKitPhidget():
  Phidget((CPhidgetHandle*)&IFK),
  IFK(0)
{
  CPhidgetInterfaceKit_create(&IFK);
  
  CPhidgetInterfaceKit_set_OnInputChange_Handler(IFK, IFK_InputChangeHandler, NULL);
  CPhidgetInterfaceKit_set_OnOutputChange_Handler(IFK, IFK_OutputChangeHandler, NULL);
  CPhidgetInterfaceKit_set_OnSensorChange_Handler(IFK, IFK_SensorChangeHandler, NULL);
  CPhidget_set_OnAttach_Handler((CPhidgetHandle)IFK, IFK_AttachHandler, NULL);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle)IFK, IFK_DetachHandler, NULL);
  CPhidget_set_OnError_Handler((CPhidgetHandle)IFK, IFK_ErrorHandler, NULL);
  
  
};

bool InterfaceKitPhidget::init(int serial_number)
{
  if (open(serial_number))  //base class
    {
      std::cerr<< "Open Failed";
      return false;   ///\todo catch other info in return values
    }
  
  //wait 5 seconds for attachment
  if(CPhidget_waitForAttachment((CPhidgetHandle)IFK, 5000))
    {
      std::cerr<< "Attach Failed";
      return false; ///\todo catch other info in return values
    }

  return true;
  
};



int test_interfacekit()
{
	int numInputs, numOutputs, numSensors;

	extern int mval;
        InterfaceKitPhidget myPhidget;

        if (!myPhidget.init(-1))
          exit(-1);

          
        CPhidgetInterfaceKitHandle IFK =  myPhidget.getIFK();

        std::cout << "Device Type: " << myPhidget.getDeviceType() << std::endl;
        std::cout << "Serial Number: " << myPhidget.getDeviceSerialNumber() << std::endl;
        std::cout << "Device Version: " << myPhidget.getDeviceVersion() << std::endl;
        std::cout << "Device Label: " << myPhidget.getDeviceLabel() << std::endl;
        std::cout << "Device Name: " << myPhidget.getDeviceName() << std::endl;
        std::cout << "Library Version: " << myPhidget.getLibraryVersion() << std::endl;

        
	CPhidgetInterfaceKit_getNumOutputs((CPhidgetInterfaceKitHandle)IFK, &numOutputs);
	CPhidgetInterfaceKit_getNumInputs((CPhidgetInterfaceKitHandle)IFK, &numInputs);
	CPhidgetInterfaceKit_getNumSensors((CPhidgetInterfaceKitHandle)IFK, &numSensors);
	
	CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);

	printf("Sensors:%d Inputs:%d Outputs:%d\n", numSensors, numInputs, numOutputs);


        for (unsigned int ind = 0; ind < 3; ind++)
        {
          CPhidgetInterfaceKit_setOutputState(IFK, ind, 1);
          sleep(1);
          
          CPhidgetInterfaceKit_setOutputState(IFK, ind, 0);
          sleep(1);
          CPhidgetInterfaceKit_setOutputState(IFK, ind, 1);
          sleep(1);
          CPhidgetInterfaceKit_setOutputState(IFK, ind, 0);
        }


        //	CPhidgetTextLCDHandle textHandle = 0;
        //	CPhidgetTextLCD_create(&textHandle);
        //	CPhidget_open((CPhidgetHandle)textHandle, -1);

        TextLCD myLCD;
        myLCD.init(-1);

	printf("connecting to textLCD");
	sleep(1);
	int i;
	for (i = 0; i < 20; i++)
	  {
            /*
              CPhidgetTextLCD_setDisplayString(textHandle, 1, "Hello");
              CPhidgetTextLCD_setBacklight(textHandle, PTRUE);
              usleep(mval*1000);
              printf("mval: %d\n",mval);
              CPhidgetTextLCD_setDisplayString(textHandle, 0, "Hello");
              CPhidgetTextLCD_setBacklight(textHandle, PFALSE);
            */
            std::stringstream ss;
            ss <<"Hi " << i << myLCD.getColumnCount();
            myLCD.setDisplayString(0,ss.str());
            myLCD.setBacklight(PTRUE);
            usleep(mval*1000);
            if (myLCD.getBacklight())
              std::cout<<"Backlight ON"<<std::endl;
            else
              std::cout<<"Backlight OFF"<<std::endl;
            myLCD.setBacklight(PFALSE);
            ss << " Again. . . " << myLCD.getRowCount();
            myLCD.setDisplayString(1,ss.str());
            usleep(mval*1000);
            if (myLCD.getBacklight())
              std::cout<<"Backlight ON"<<std::endl;
            else
              std::cout<<"Backlight OFF"<<std::endl;
          }

        /*
	for (i = 0; i < 20; i++)
          {
            std::string mystring;
            std::cout << "Please Enter a string:";
            std::cin>>mystring;
            char astring[100];
            strcpy(astring, mystring.c_str());
            CPhidgetTextLCD_setDisplayString(textHandle, i%2, astring);
	    CPhidgetTextLCD_setBacklight(textHandle, PTRUE);
            sleep(3);
            CPhidgetTextLCD_setBacklight(textHandle, PFALSE);
          }
        */
        //exit:
	CPhidget_close((CPhidgetHandle)IFK);
	CPhidget_delete((CPhidgetHandle)IFK);

	return 0;
}

int main(int argc, char* argv[])
{
	test_interfacekit();
	return 0;
}

