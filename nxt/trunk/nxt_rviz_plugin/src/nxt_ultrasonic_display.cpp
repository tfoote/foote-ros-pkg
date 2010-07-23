/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "nxt_ultrasonic_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <ogre_tools/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
NXTUltrasonciDisplay::NXTUltrasonciDisplay( const std::string& name, rviz::VisualizationManager* manager )
: Display( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, messages_received_(0)
, tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  cone_ = new ogre_tools::Shape(ogre_tools::Shape::Cone, vis_manager_->getSceneManager(), scene_node_);

  scene_node_->setVisible( false );

  setAlpha( 0.5f );
  Ogre::Vector3 scale( 0, 0, 0);
  rviz::scaleRobotToOgre( scale );
  cone_->setScale(scale);
  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&NXTUltrasonciDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

NXTUltrasonciDisplay::~NXTUltrasonciDisplay()
{
  unsubscribe();
  clear();
  delete cone_;
}

void NXTUltrasonciDisplay::clear()
{

  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received");
}

void NXTUltrasonciDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void NXTUltrasonciDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void NXTUltrasonciDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void NXTUltrasonciDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void NXTUltrasonciDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void NXTUltrasonciDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void NXTUltrasonciDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void NXTUltrasonciDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void NXTUltrasonciDisplay::update(float wall_dt, float ros_dt)
{
}


void NXTUltrasonciDisplay::processMessage(const nxt_msgs::Range::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range/2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!vis_manager_->getFrameManager()->transform(msg->header.frame_id,msg->header.stamp,pose, position, orientation, true))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cone_->setPosition(position);
  cone_->setOrientation(orientation); 
  Ogre::Vector3 scale( sin(msg->spread_angle) * msg->range, sin(msg->spread_angle) * msg->range , msg->range);
  rviz::scaleRobotToOgre( scale );
  cone_->setScale(scale);
  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

}

void NXTUltrasonciDisplay::incomingMessage(const nxt_msgs::Range::ConstPtr& msg)
{
  processMessage(msg);
}

void NXTUltrasonciDisplay::reset()
{
  Display::reset();
  clear();
}

void NXTUltrasonciDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &NXTUltrasonciDisplay::getTopic, this ),
                                                                                boost::bind( &NXTUltrasonciDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "nxt_msgs::Range topic to subscribe to.");
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<nxt_msgs::Range>());
  color_property_ = property_manager_->createProperty<rviz::ColorProperty>( "Color", property_prefix_, boost::bind( &NXTUltrasonciDisplay::getColor, this ),
                                                                      boost::bind( &NXTUltrasonciDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color to draw the range.");
  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &NXTUltrasonciDisplay::getAlpha, this ),
                                                                       boost::bind( &NXTUltrasonciDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the range.");
}

const char* NXTUltrasonciDisplay::getDescription()
{
  return "Displays data from a nxt_msgs::Range message as a cone.";
}
} // namespace nxt_rviz_plugin
