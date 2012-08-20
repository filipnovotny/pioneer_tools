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
 \file OdometryFramePublisher.cpp
 \brief 
 */

#include "odometry_frame_publisher.h"
#include "names.h"

namespace pioneer{
  OdometryFramePublisher::OdometryFramePublisher() :
    n_("~"),
    queue_size_(1000),
    child_frame_("/robot_pos"),
    offset_x_(0.),
    offset_y_(0.),
    offset_z_(0.)
  {
      pioneer::remap();

      n_.param(child_frame_param, child_frame_, child_frame_);
      n_.param(offset_x_param, offset_x_, offset_x_);
      n_.param(offset_y_param, offset_y_, offset_y_);
      n_.param(offset_z_param, offset_z_, offset_z_);

      ros::Duration(1).sleep();

      ROS_INFO("Odometry topic set to: %s", pioneer::odometry_topic.c_str());
      ROS_INFO("Param offset_x set to: %f", offset_x_);
      ROS_INFO("Param offset_y set to: %f", offset_y_);
      ROS_INFO("Param offset_z set to: %f", offset_z_);
      odometry_subscriber_ = n_.subscribe<nav_msgs::Odometry>(pioneer::odometry_topic, queue_size_, &OdometryFramePublisher::odometryCallback, this);

      ros::spin();

  }

  void OdometryFramePublisher::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry){
    tf::Transform transform;

    transform.setOrigin(
        tf::Vector3(
            odometry->pose.pose.position.x+offset_x_,
            odometry->pose.pose.position.y+offset_y_,
            odometry->pose.pose.position.z+offset_z_
            )
    );
    transform.setRotation(
        tf::Quaternion(
            odometry->pose.pose.orientation.x,
            odometry->pose.pose.orientation.z,
            odometry->pose.pose.orientation.y,
            odometry->pose.pose.orientation.w
        )
    );

    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odometry->header.frame_id, child_frame_));

  }

  OdometryFramePublisher::~OdometryFramePublisher()
  {
    // TODO Auto-generated destructor stub
  }
}
