/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 *
 * This file is part of DICH.
 *
 * DICH is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DICH is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DICH. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DICH_STEERER_H
#define DICH_STEERER_H

#include <dich/Match.h>

#include <yamabros/spur.h>

#include <clarus/clarus.hpp>

#include <ros/ros.h>

namespace dich
{

struct Steerer
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node;

  /** \brief Subscriber object used to receive commands. */
  ros::Subscriber subscriber;

  /** \brief Mobile robot control interface. */
  yamabros::spur::Spur spur;

  /** \brief List of robot steerings. */
  clarus::List<cv::Point2d> steerings;

  /** \brief Robot linear speed. */
  double v;

  /** \brief Robot angular speed. */
  double w;

  Steerer();

  void callback(const MatchConstPtr &message);
};

} // namespace dich

#endif
