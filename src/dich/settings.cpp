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

#include <dich/settings.h>

#include <boost/filesystem.hpp>
using boost::filesystem::current_path;

namespace dich
{

namespace name
{

std::string image()
{
  return ros::names::resolve("image");
}

std::string match()
{
  return ros::names::resolve("match");
}

} // namespace name

namespace param
{

double ang_vel()
{
  return param<double>("~ang_vel", 0.3);
}

double lin_vel()
{
  return param<double>("~lin_vel", 0.3);
}

std::string path()
{
  return param<std::string>("~path", "");
}

std::string path_replay_offline()
{
  return param<std::string>("~path_replay_offline", "");
}

std::string path_replay_online()
{
  return param<std::string>("~path_replay_online", (current_path() / "replay.mpg").native());
}

std::string path_steerings()
{
  return param<std::string>("~path_steerings", "");
}

std::string path_ground_truth()
{
  return param<std::string>("~path_ground_truth", (current_path() / "ground_truth.txt").native());
}

bool headless()
{
  return param<bool>("~headless", true);
}

namespace difference
{

int exposure()
{
  return param<int>("~exposure", 5);
}

double threshold()
{
  return param<double>("~threshold", 0.1);
}

cv::Size size()
{
  return cv::Size(width(), height());
}

int width()
{
  return param<int>("~width", 256);
}

int height()
{
  return param<int>("~height", 192);
}

} // namespace difference

namespace interest
{

int padding()
{
  return param<int>("~padding", 12);
}


int padding2()
{
  return param<int>("~padding2", 12);
}

int points()
{
  return param<int>("~interest_points", 30);
}

double cutoff()
{
  return param<double>("~cutoff", 0.02);
}

} // namespace interest

namespace similarities
{

double error()
{
  return param<double>("~similarity_error", 5.0);
}

int samples()
{
  return param<int>("~similarity_samples", 5);
}

int height()
{
  return param<int>("~similarity_height", 50);
}

double leak()
{
  return param<double>("~similarity_leak", 0.98);
}

int width()
{
  return param<int>("~similarity_width", 20);
}

} // namespace similarities

namespace shift
{

int column()
{
  return param<int>("~column", 16);
}

int width()
{
  return param<int>("~shift_width", 32);
}

} // namespace shift

namespace replay
{

int queue()
{
  return param<int>("~queue", 0);
}

} // namespace replay


} // namespace param

} // namespace dich
