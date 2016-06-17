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

#include <dich/difference_filter.h>
using clarus::ZERO;

#include <dich/settings.h>

namespace dich
{

DifferenceFilter::DifferenceFilter():
  exposure_(param::difference::exposure()),
  threshold_(param::difference::threshold()),
  width_(param::difference::width()),
  count_(0),
  i_(0),
  j_(0)
{
  // Nothing to do.
}

DifferenceFilter::DifferenceFilter(int exposure, double threshold, int width):
  exposure_(exposure),
  threshold_(threshold),
  width_(width),
  count_(0),
  i_(0),
  j_(0)
{
  // Nothing to do.
}

DifferenceImage DifferenceFilter::operator () (const cv::Mat &image)
{
  cv::Mat current = clarus::log(1.0 + images::convert(colors::grayscale(image), CV_32F));
  cv::Mat previous = previous_;
  previous_ = current;

  if (previous.empty())
  {
    accumulated_ = cv::Mat::zeros(current.size(), CV_64F);
    return DifferenceImage();
  }

  i_++;
  count_++;

  cv::Mat differences = cv::abs(current - previous);
  cv::threshold(differences, differences, threshold_, 1.0, cv::THRESH_BINARY);
  cv::add(differences, accumulated_, accumulated_, cv::noArray(), CV_64F);
  if (count_ < exposure_)
    return DifferenceImage();

  j_++;

  cv::Mat scaled = images::scale(accumulated_, width_, cv::INTER_NEAREST);
  DifferenceImage J = DifferenceImage(i_, j_, scaled / clarus::max(scaled));
  accumulated_ = ZERO;
  count_ = 0;

  ROS_INFO_STREAM("Difference Image: (" << i_ << ", " << j_ << ")");

  return J;
}

} // namespace dich
