/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of DICH.

DICH is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

DICH is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with DICH. If not, see <http://www.gnu.org/licenses/>.
*/

#include <dich/steerer.h>
#include <dich/io.h>
#include <dich/settings.h>
#include <dich/Match.h>

#include <cv_video/video.h>
using cv_video::Video;

#include <yamabros/robot.h>
using yamabros::Robot;

#include <std_msgs/Int32.h>

namespace dich
{

Steerer::Steerer():
  node(),
  subscriber(node.subscribe<Match>(name::match(), 1, &Steerer::callback, this)),
  steerings(load<cv::Point2d>(param::path_steerings())),
  v(param::lin_vel()),
  w(0.1 * param::ang_vel())
{
  spur.coast();
}

void Steerer::callback(const MatchConstPtr &message)
{
  int index = message->index;
  int shift = message->shift;

  if (index >= steerings.size())
  {
    spur.coast();
    ros::shutdown();
    return;
  }

  cv::Point2d speeds = steerings[index];
  double vt = speeds.x;
  double wt = speeds.y;

  if (wt == 0)
    wt = -w * copysign(1.0, shift);

  spur.steer(vt, wt);
}

} // namespace dich
