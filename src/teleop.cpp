/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
 \file camera.cpp
 \brief 
 */


#include "teleop.h"
#include "names.h"
#include <geometry_msgs/Twist.h>
#include <sstream>


namespace pioneer_tools
{
Teleop::Teleop(int argc, char**argv) :
            n_("~"),
            spinner(0),
            linear_(1),
            angular_(2),
            cancel_(1),
            goal_set_(false),
            queue_size_(1000)
{
  pioneer_tools::remap();

  n_.param(axis_linear_param, linear_, linear_);
  n_.param(axis_angular_param, angular_, angular_);
  n_.param(scale_linear_param, l_scale_, l_scale_);
  n_.param(scale_angular_param, a_scale_, a_scale_);
  n_.param(cancel_param, cancel_, cancel_);

  ros::Duration(1).sleep();
  ROS_INFO("Joy topic set to: %s", pioneer_tools::joy_topic.c_str());
  ROS_INFO("Velocity topic set to: %s", pioneer_tools::velocity_topic.c_str());
  ROS_INFO("Goal topic set to: %s", pioneer_tools::goal_topic.c_str());
  ROS_INFO("Goal cancel topic set to: %s", pioneer_tools::goal_cancel_topic.c_str());
  ROS_INFO("axis angular set to: %d", angular_);
  ROS_INFO("axis linear set to: %d", linear_);
  ROS_INFO("cancel button set to: %d", cancel_);

  ROS_INFO("scale angular set to: %f", a_scale_);
  ROS_INFO("scale linear set to: %f", l_scale_);


  joy_subscriber_ = n_.subscribe<sensor_msgs::Joy>(pioneer_tools::joy_topic, queue_size_, &Teleop::joyCallback, this);
  goal_subscriber_ = n_.subscribe<move_base_msgs::MoveBaseActionGoal>(pioneer_tools::goal_topic, queue_size_, &Teleop::goalCallback, this);
  velocity_publisher_ = n_.advertise<geometry_msgs::Twist>(velocity_topic, queue_size_);
  goal_cancel_publisher_ = n_.advertise<move_base_msgs::MoveBaseActionGoal>(goal_cancel_topic, queue_size_);

  ros::spin();

}

void Teleop::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){
  goal_ = goal->goal_id;
  goal_set_ = true;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  geometry_msgs::Twist vel;
  double v_angular = a_scale_*joy->axes[angular_];
  double v_linear = l_scale_*joy->axes[linear_];

  int cancel_button_ = joy->buttons[cancel_];

  vel.linear.x = v_linear;
  vel.linear.y = 0.;
  vel.angular.z = v_angular;
  ROS_INFO("New speed: linear= %f, angular=%f", v_linear,v_angular);

  if(cancel_button_ && goal_set_){
    goal_cancel_publisher_.publish(goal_);
    goal_set_ = false;
  }

  velocity_publisher_.publish(vel);

}


Teleop::~Teleop()
{
  // TODO Auto-generated destructor stub
}
}
