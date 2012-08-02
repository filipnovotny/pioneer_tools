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
#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <sstream>


namespace pioneer
{
Teleop::Teleop(int argc, char**argv) :
            n_("~"),
            spinner(0),
            linear_(1),
            angular_(2),
            queue_size_(1000)

{
  pioneer::remap();

  n_.param(axis_linear_param, linear_, linear_);
  n_.param(axis_angular_param, angular_, angular_);
  n_.param(scale_linear_param, l_scale_, l_scale_);
  n_.param(scale_angular_param, a_scale_, a_scale_);
  spinner.start();
  ros::Duration(1).sleep();
  ROS_INFO("Joy topic set to: %s", pioneer::joy_topic.c_str());
  ROS_INFO("axis angular set to: %d", angular_);
  ROS_INFO("axis linear set to: %d", linear_);
  ROS_INFO("scale linear set to: %f", l_scale_);
  ROS_INFO("scale angular set to: %f", a_scale_);

  joy_subscriber_ = n_.subscribe<sensor_msgs::Joy>(pioneer::joy_topic, queue_size_, &Teleop::joyCallback, this);

#ifdef VISP_HAVE_PIONEER
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot_);
  if(!robotConnector.connectRobot())
  {
    ROS_ERROR("Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
      ros::shutdown();
    }
  }
  if (!Aria::parseArgs())
  {
    ROS_ERROR("Could not parse command line arguments.");
    ros::shutdown();
  }

  ROS_INFO("Robot connected");
  robot_.useSonar(false); // disable the sonar device usage
#endif


}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  double v_angular = a_scale_*joy->axes[angular_];
  double v_linear = l_scale_*joy->axes[linear_];


  vpColVector v(2);
  v[0] = v_linear;
  v[1] = v_angular;
  ROS_INFO("New speed: linear= %f, angular=%f", v_linear,v_angular);

#ifdef VISP_HAVE_PIONEER
  robot_.setVelocity(vpRobot::REFERENCE_FRAME, v);
#endif
}


Teleop::~Teleop()
{
  // TODO Auto-generated destructor stub
}
}
