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

#include <dich/io.h>
using clarus::List;

#include <dich/difference_filter.h>
#include <dich/settings.h>

#include <cv_video/replayer.h>
using cv_video::Replayer;

namespace dich
{

template<>
List<cv::Mat> load(const std::string &path)
{
  ROS_INFO_STREAM("Load path: \"" << path << '"');

  List<cv::Mat> images;
  Replayer replayer(path);
  for (;;)
  {
    cv::Mat image = replayer();
    if (image.empty())
      break;

    images.append(image);
  }

  return images;
}

template<>
List<DifferenceImage> load(const std::string &path)
{
  ROS_INFO_STREAM("Load path: \"" << path << '"');

  List<DifferenceImage> differences;
  Replayer replayer(path);
  DifferenceFilter filter;
  for (;;)
  {
    cv::Mat image = replayer();
    if (image.empty())
      break;

    DifferenceImage J = filter(image);
    if (J.empty())
      continue;

    differences.append(J);
  }

  return differences;
}

} // namespace dich
