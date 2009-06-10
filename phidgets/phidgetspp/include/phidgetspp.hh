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

#ifndef PHIDGETPP_HH
#define PHIDGETPP_HH

#include <phidget21.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstring>

class Phidget
{
public:
  Phidget(CPhidgetHandle * handle):mHandle(handle){;};

  int open(int serial_number){
    return CPhidget_open(*mHandle,serial_number);};
  
  int close(int serial_number){
    return CPhidget_close(*mHandle);};
  
  int deletePhidget(){
    return CPhidget_delete(*mHandle);};
  
  int waitForAttachment(int timeout)
  {return CPhidget_waitForAttachment(*mHandle, timeout);};


  std::string getDeviceType(){
    char a[1000];
    const char * deviceptr = a;
    CPhidget_getDeviceType( *mHandle, &deviceptr);
    return std::string(deviceptr);
  };

  std::string getDeviceName(){
    char a[1000];
    const char * deviceptr = a;
    CPhidget_getDeviceName( *mHandle, &deviceptr);
    return std::string(deviceptr);
  };

  std::string getDeviceLabel(){
    char a[1000];
    const char * deviceptr = a;
    CPhidget_getDeviceType( *mHandle, &deviceptr);
    return std::string(deviceptr);
  };

  std::string getServerID(){
    const char * deviceptr;
    CPhidget_getServerID( *mHandle, &deviceptr);
    return std::string(deviceptr);
  };

  std::string getServerAddress(){
    const char * deviceptr;
    int port;
    CPhidget_getServerAddress( *mHandle, &deviceptr, &port);
    return std::string(deviceptr);
  };

  int getServerPort(){
    const char * deviceptr;
    int port;
    CPhidget_getServerAddress( *mHandle, &deviceptr, &port);
    return port;
  };

  int getServerStatus(){
    int status;
    CPhidget_getServerStatus( *mHandle, &status);
    return status;
  };
  
  std::string getLibraryVersion(){
    const char * deviceptr;
    CPhidget_getLibraryVersion(&deviceptr);
    return std::string(deviceptr);
  };

  int getSerialNumber(){
    int sernum;
    CPhidget_getSerialNumber(*mHandle, &sernum);
    return sernum;
  };
  
  int getDeviceVersion(){
    int version;
    CPhidget_getDeviceVersion(*mHandle, &version);
    return version;
  };
  
  static std::string getErrorDescription(int errorCode){
    const char * errorPtr;
    CPhidget_getErrorDescription(errorCode, &errorPtr);
    return std::string(errorPtr);
  };
  

protected:
  CPhidgetHandle* mHandle;
  
  int attachHandler()
  {
    printf("Attach handler ran!\n");
    return 0;
  };

private:
  
};

class TextLCD: public Phidget
{
public:
  /** @brief The constructor for the TextLCD Phidget
      Create a new CPhidget TextLCD interface class */
  TextLCD(): Phidget((CPhidgetHandle*)&textHandle), textHandle(0),
             column_count_(0), column_count_initialized_(false){
    CPhidgetTextLCD_create(&textHandle);  };
  /** @brief Initialize and connect to a device
      This will connect to any or a specific TextLCD
      @param serial_number The serial number to which to connect (-1 for any) */
  int init(int serial_number){
    return open(serial_number);};
  /** @brief Get the number of rows in the attached display. */
  int getRowCount(){int count; CPhidgetTextLCD_getRowCount(textHandle, &count); return count;};
  /** @brief Get the number of columns in the attached display. */
  int getColumnCount(){if (!column_count_initialized_) return privateGetColumnCount(); else return column_count_;}
    /** @brief Get the baclight level 
        PTRUE or PFALSE */
  bool getBacklight(){int status; 
    CPhidgetTextLCD_getBacklight(textHandle, &status); if (status == PTRUE) return true; else return false;};
  /** @brief Set the baclight level
      @param status PTRUE or PFALSE */
  int setBacklight(int status){return CPhidgetTextLCD_setBacklight(textHandle, status);};
  /** Get unknownval error code
      int getContrast(){int contrast; 
      CPhidgetTextLCD_getContrast(textHandle, &contrast); return contrast;};
  */
  /** @brief Set the contrast
      Set the contrast ratio of the display.  
      @param contrast 0 to 255 contrast 128 is default
  */
  int setContrast(int contrast){return CPhidgetTextLCD_setContrast(textHandle, contrast);};
  //int getCursorOn();
  //int setCursorOn();
  //int getCursorBlink();
  //int setCursorBlink();
  //int setCustomCharacter();
  /** @brief Set a single character on the display
      Set a single character on the display by row and column number
      @param row The row number 
      @param column The column 
      @param character The character to display*/
  int setDisplayCharacter(int row, int column, unsigned char character){
    return CPhidgetTextLCD_setDisplayCharacter(textHandle, row, column, character);};
  /** @brief Set the display by string
      Set a row of characters by string
      @param row The row of characters to set
      @param astring The string to fill in*/
  int setDisplayString(int row, std::string astring){
    char cstring[100] = "";//fixme
    strncpy(cstring, astring.c_str(), std::min(astring.length(), (size_t)privateGetColumnCount()));
    return CPhidgetTextLCD_setDisplayString(textHandle, row, cstring);};
    
private:
  ///Storage for the CPhidget handle
  CPhidgetTextLCDHandle textHandle;
  //A way to query the device only once since this value will be used for every write 
  //This could be also just queried at startup.  WHich would clean up the flag etc.  
  int privateGetColumnCount(){int count; CPhidgetTextLCD_getColumnCount(textHandle, &count); column_count_ = count; column_count_initialized_ = true; return count;};

  int column_count_;
  bool column_count_initialized_;
};



class Servo: public Phidget
{
public:
  /** @brief The constructor for the Servo Phidget
      Create a new CPhidget Servo interface class */
  Servo(): Phidget((CPhidgetHandle*)&servoHandle), servoHandle(0){
    CPhidgetServo_create(&servoHandle);  };
  /** @brief Initialize and connect to a device
      This will connect to any or a specific Servo
      @param serial_number The serial number to which to connect (-1 for any) */
  int init(int serial_number){
    int retval = open(serial_number);
    waitForAttachment(5000); /** @todo add some checks on this */
    return retval;
  };

  /** @brief Return how many servo motor channels are available */  
  int getMotorCount(){int count; CPhidgetServo_getMotorCount(servoHandle, &count); return count;};
  
  /** @brief Get the Position of the servo
   * @todo broken returns bad values */
  double getPosition(int index) 
  { 
    double position; 
    CPhidgetServo_getPosition(servoHandle, index, &position); 
    return position;
  };
  /** @brief Set the position of a servo
   * @param index Which motor
   * @param position The position to go to
   */
  int setPosition(int index, double position){return CPhidgetServo_setPosition(servoHandle, index, position);};
  /** @brief Get the max position value
   * @param index Which motor to poll*/
  double getPositionMax(int index){double maxPos; CPhidgetServo_getPositionMax(servoHandle, index, &maxPos); return maxPos;};
  /** @brief Get the minimum position value
   * @param index Which motor to poll*/
  double getPositionMin(int index){double minPos; CPhidgetServo_getPositionMin(servoHandle, index, &minPos); return minPos;};
  /** @brief Get whether the motor is engaged
   * @param index Which motor */
  int getEngaged(int index){int state; CPhidgetServo_getEngaged(servoHandle, index, &state); return state;};
  /** @brief Set whether the motor is engaged
   * @param index Which motor
   * @param state PTRUE or PFALSE*/
  bool setEngaged(int index, int state){ if (!CPhidgetServo_setEngaged(servoHandle, index, state)) return true; else return false;};
  
  //Unimplemented set_OnPositionChange

private:
  ///Storage for the CPhidget handle
  CPhidgetServoHandle servoHandle;


};

/**@brief The basic class for interfacing with the RFIDPhidget 
 * This class is designed to either standalone for basic usage, 
 * or to provide a baseclass for further customization. */
class RFIDPhidget : public Phidget
{
public:
/**@brief Default Constructor */
  RFIDPhidget(): Phidget((CPhidgetHandle*)&RFID),
                 RFID(0)
  {
    //Setup the phidget
    CPhidgetRFID_create(&RFID);
    //Setup the handler for the new tag event
    CPhidgetRFID_set_OnTag_Handler(RFID, RFIDPhidget::tag_function, this);
    //Setup the handler for the lost tag event
    CPhidgetRFID_set_OnTagLost_Handler(RFID, RFIDPhidget::tagLost_function, this);
    //Setup the attachment handler from the base class
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)RFID, RFIDPhidget::AttachHandler, this);  ///@TODO find how to move to base class?? It can't go in constructor due ot order
  };
  

  /**@brief An example function for handling new tag events 
   * This is expected to be overriden in a subclass */
  virtual int gotTag(unsigned int tag){
    std::cout << "Default gotTag response! " << tag <<std::endl;
    return 0;
  }

  /**@brief An example function for handling lost tag events 
   * This is expected to be overriden in a subclass */
  virtual int lostTag(unsigned int tag){
      printf( "Default lostTag response! %u\n", tag);
      return 0;
  };

  /**@brief Set the antenna on(true) or off (false) */
  int setAntennaOn(bool state)
  {
    if (state == true)
      return CPhidgetRFID_setAntennaOn(RFID, PTRUE);
    else
      return CPhidgetRFID_setAntennaOn(RFID, PFALSE);
  };
  
  /**@brief Return whether the RFID antenna is currently on */
  bool isAntennaOn()
  {
    int status;
    ///\todo ignoring return code
    CPhidgetRFID_getAntennaOn(RFID, &status);
    if (status == PTRUE) return true;
    else return false;
  };

  /** @brief Turn the LED on(true) or off(false) */
  int setLedOn(bool state)
  {
    if (state == true)
      return CPhidgetRFID_setLEDOn(RFID, PTRUE);
    else
      return CPhidgetRFID_setLEDOn(RFID, PFALSE);
  };

  /** @brief Return whether the LED is on at the moment */
  bool isLedOn()
  {
    int status;
    ///\todo ignoring return code
    CPhidgetRFID_getLEDOn(RFID, &status);
    if (status == PTRUE) return true;
    else return false;
  };
  
  /** @brief Return whether a tag is present at the moment */
  bool isTagPresent()
  {
    int status;
    ///\todo ignoring return code
    CPhidgetRFID_getTagStatus(RFID, &status);
    if (status == PTRUE) return true;
    else return false;
  }    
  
  /** @brief Get the number of the last tag seen */
  unsigned int getLastTag()
  {
    unsigned char mytag[100];
    ///\todo ignoring return code
    CPhidgetRFID_getLastTag(RFID, mytag);
    return *(unsigned int*)(mytag);
  };
  
  /** @brief Get the number of outputs available */
  unsigned int getOutputCount()
  {
    int status;
    ///\todo ignoring return code
    CPhidgetRFID_getOutputCount(RFID, &status);
    return (unsigned int)status;
  }; 

  /** @brief Get the state of the output numbered index */
  bool getOutputState(int index)
  {
    int status;
    ///\todo ignoring return code
    CPhidgetRFID_getOutputState(RFID, index, &status);
    if (status == PTRUE) return true;
    else return false;    
  };

  /**@brief Set the state of the output number index */
  void setOutputState(int index, bool state)
  {
    if (state == true)
      CPhidgetRFID_setOutputState(RFID, index, PTRUE);
    else
      CPhidgetRFID_setOutputState(RFID, index, PFALSE);
  };

private:
  CPhidgetRFIDHandle RFID; ///<! The handle for the C API

  //Static functions to callback to from the C library
  static int tag_function(CPhidgetRFIDHandle RFID, void* userptr, unsigned char * tag)
  { return ((RFIDPhidget*)userptr)->gotTag(*(unsigned int*)(tag));};
  static int tagLost_function(CPhidgetRFIDHandle RFID, void* userptr, unsigned char * tag)
  { return ((RFIDPhidget*)userptr)->lostTag(*(unsigned int*)(tag));};
  static int AttachHandler(CPhidgetHandle RFID, void *userptr)
  { return ((RFIDPhidget*)userptr)->attachHandler();};

};


#endif //PHIDGETPP_HH
