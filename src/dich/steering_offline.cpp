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

#include <dich/settings.h>

#include <cv_video/video.h>
using cv_video::Video;

#include <yamabros/robot.h>
using yamabros::Robot;

#include <std_msgs/Int32.h>

namespace dich
{

struct SteeringMock
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node;

  /** \brief Subscriber object used to receive commands. */
  ros::Subscriber subscriber;

  Video video;

  bool playing;

  SteeringMock()
  {
    subscriber = node.subscribe<std_msgs::Int32>(name::index(), 1, &SteeringMock::callback, this);
    playing = false;
  }

  void callback(const std_msgs::Int32ConstPtr &message)
  {
    if (playing)
      return;

    playing = true;
    video.replay(param::path(), name::image(), ros::shutdown);
  }
};

} // namespace dich

int main(int argc, char *argv[])
{
  Robot robot;
  robot.init("steering_offline", argc, argv);
  robot.set("steering", new dich::SteeringMock());
  robot.spin();

  return 0;
}
