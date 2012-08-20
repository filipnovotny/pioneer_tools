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
 \file OdometryFramePublisher.h
 \brief 
 */

#ifndef ODOMETRYFRAMEPUBLISHER_H_
#define ODOMETRYFRAMEPUBLISHER_H_
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

namespace pioneer{
  class OdometryFramePublisher
  {
  private:
    ros::NodeHandle n_;

    ros::Subscriber odometry_subscriber_;
    tf::TransformBroadcaster br_;

    unsigned int queue_size_;

    std::string child_frame_;
    double offset_x_;
    double offset_y_;
    double offset_z_;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);

  public:
    OdometryFramePublisher();
    virtual ~OdometryFramePublisher();
  };
}
#endif /* ODOMETRYFRAMEPUBLISHER_H_ */
