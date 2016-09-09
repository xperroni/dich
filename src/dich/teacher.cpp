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

#include <dich/teacher.h>
using cv_video::Video;
using cv_video::Frame;
using terminal_io::Terminal;

#include <dich/settings.h>

#include <ros/console.h>

#include <boost/bind.hpp>

namespace dich
{

static const char *STEER_COMMANDS =
  "\nCommands:\n"
  "w - Move straight\n"
  "s - Stops all movement\n"
  "q, e - Steer left / right while moving forward\n"
  "a, d - Stops then spins left / right\n"
  "z, c - Spins 1 degree left / right\n"
  "x - Exit application\n";

Teacher::Teacher():
  spur(false),
  terminal(),
  video(),
  recorder(),
  v(0),
  w(0),
  V(param::lin_vel()),
  W(param::ang_vel())
{
  video.subscribe("teach", &Teacher::log, this);

  terminal.get(STEER_COMMANDS, boost::bind(&Teacher::input, this, _1, _2), '\0', false);
}

void Teacher::log(Video &video, Frame &frame)
{
  if (v == 0 && w == 0)
    return;

  recorder(frame);

  ROS_INFO_STREAM("Speeds: (" << v << ", " << w << ")");
}

void Teacher::input(Terminal &terminal, char key)
{
  static double DEG = M_PI / 180.0;

  switch (key)
  {
    case 's': steer(0,  0); return;
    case 'w': steer(V,  0); return;
    case 'q': steer(V,  0.1 * W); return;
    case 'e': steer(V, -0.1 * W); return;
    case 'a': steer(0,  W); return;
    case 'd': steer(0, -W); return;
    case 'z': spur.spin(+DEG); return;
    case 'c': spur.spin(-DEG); return;
    case 'x':
    {
      spur.coast();
      ros::shutdown();
      return;
    }
  }
}

void Teacher::steer(double vt, double wt)
{
  v = vt;
  w = wt;
  spur.steer(v, w);
}

} // namespace dich
