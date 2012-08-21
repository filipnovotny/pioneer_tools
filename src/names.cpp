/****************************************************************************
 *
 * $Id: file.h 3496 2011-11-22 15:14:32Z fnovotny $
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
 * File containing names of topics or services used all accross the package
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file names.cpp
  \brief File containing names of topics or services used all accross the package
*/

#include "names.h"
#include "ros/ros.h"

namespace pioneer_tools
{
  std::string joy_topic("joy");
  std::string odometry_topic("odometry");
  std::string velocity_topic("velocity");
  std::string goal_topic("goal");
  std::string goal_cancel_topic("cancel");

  std::string axis_linear_param("axis_linear");
  std::string axis_angular_param("axis_angular");
  std::string scale_linear_param("scale_linear");
  std::string scale_angular_param("scale_angular");
  std::string cancel_param("cancel");

  std::string child_frame_param("child_frame");
  std::string offset_x_param("offset_x");
  std::string offset_y_param("offset_y");
  std::string offset_z_param("offset_z");


  void remap(){
    if (ros::names::remap("joy") != "joy")
      joy_topic = ros::names::remap("joy");
    if (ros::names::remap("velocity") != "velocity")
      velocity_topic = ros::names::remap("velocity");
    if (ros::names::remap("odometry") != "odometry")
      odometry_topic = ros::names::remap("odometry");
    if (ros::names::remap("cancel") != "cancel")
      goal_cancel_topic = ros::names::remap("cancel");
    if (ros::names::remap("goal") != "goal")
      goal_topic = ros::names::remap("goal");
  }

}


