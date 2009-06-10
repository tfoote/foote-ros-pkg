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

#include <gtest/gtest.h>
#include <sstream>

#include <ctime>
#include <iostream>
#include <sstream>

#include <phidgetspp.hh>


TEST(Servo, Basic){
  Servo myServo;
  
  ASSERT_FALSE(myServo.init(-1));
  
  int count = myServo.getMotorCount();
  std::cout<< count<< " Motors" << std::endl;
  ASSERT_EQ(count, 1);
  
  int engaged = myServo.getEngaged(0);
  if (engaged != PTRUE)
    {
      myServo.setEngaged(0, PTRUE);
      engaged = myServo.getEngaged(0);
    }
  std::cout << engaged << " should be " << PTRUE << std::endl;
  ASSERT_EQ(engaged, PTRUE);

  double max = myServo.getPositionMax(0);
  
  double min = myServo.getPositionMin(0);
  double pos = myServo.getPosition(0);

  std::cout <<"Min: " << min << " Max: " << max <<" Now: " << pos << std::endl;

  for (unsigned int i = 0; i < 3; i++)
    {
      myServo.setPosition(0, min);
      std::cout << myServo.getPosition(0)<<std::endl;
      usleep(100000);
      std::cout << myServo.getPosition(0)<<std::endl;
      usleep(100000);
      std::cout << myServo.getPosition(0)<<std::endl;
      usleep(100000);
      std::cout << myServo.getPosition(0)<<std::endl;
      usleep(100000);
      std::cout << myServo.getPosition(0)<<std::endl;
      usleep(100000);

      sleep(1);
      std::cout << myServo.getPosition(0)<<std::endl;

      myServo.setPosition(0, max);
      sleep(1);
    }
  myServo.setEngaged(0, PFALSE);
  std::cout << "Monitering position for 10 seconds"<<std::endl;
  for (unsigned int i = 0; i < 100; i++)
    {
      usleep(100000);
      std::cout << myServo.getPosition(0)<<std::endl;
    }

  
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
