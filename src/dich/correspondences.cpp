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

#include <dich/correspondences.h>
using clarus::Integral;
using clarus::ONE;
using clarus::ZERO;

#include <dich/io.h>
#include <dich/settings.h>

#include <ros/console.h>

namespace dich
{

Correspondences::Correspondences():
  limit(param::interest::points()),
  padding(param::interest::padding()),
  padding2(param::interest::padding2()),
  side(padding * 2 + 1),
  side2(side + 2 * padding2),
  buckets(param::difference::exposure() * side * side),
  cutoff(buckets.size() * param::interest::cutoff()),
  aROIs(limit, cv::Size(side2, side2), fftw::FORWARD_R2C),
  aNOIs(limit, cv::Size(side2, side2), fftw::FORWARD_R2C),
  matches(limit, cv::Size(side2, side2), fftw::BACKWARD_C2R)
{
  loadTaught();
}

cv::Mat Correspondences::operator () (const DifferenceImage &J)
{
  return (*this)(0, J, 0, taught.size());
}

cv::Mat Correspondences::operator () (const DifferenceImage &J, int y0, int yn)
{
  return (*this)(0, J, y0, yn);
}

cv::Mat Correspondences::operator () (int shift, const DifferenceImage &J, int y0, int yn)
{
  loadReplay(shift, J);

  cv::Mat correspondences(taught.size(), 1, CV_64F, ZERO);
  int images = std::min(yn + 1, (int) taught.size());
  int points = neighborhoods.size();

  int side3 = side2 - side + 1;
  double s = 1.0 / (side2 * side2);
  for (int i = y0; i < images; i++)
  {
    const DifferenceImage &teach = taught[i];
    const cv::Mat &D = coefficients[i];

    loadTeach(teach);

    for (int j = 0; j < points; j++)
      matches[j].C.mul(aROIs[j], aNOIs[j], -1, s);

    matches.transform();

    double value = 0;
    for (int j = 0; j < points; j++)
    {
      cv::Rect roi = neighborhoods[j];
      cv::Mat responses(matches[j].toMat(), cv::Rect(0, 0, side3, side3));
      cv::Mat corrections(D, cv::Rect(roi.x, roi.y, side3, side3));
      value += clarus::max(responses.mul(corrections));
    }

    correspondences.at<double>(i, 0) = value;
  }

  ROS_INFO_STREAM("Similarities: (" << J.j << ", " << correspondences.t() << ")");

  return correspondences;
}

void Correspondences::loadTaught()
{
  taught = load<DifferenceImage>(param::path());
  for (int i = 0, n = taught.size(); i < n; i++)
    coefficients.append(taught[i].stdDevInv(side, side));
}

void Correspondences::loadTeach(const DifferenceImage &J)
{
  for (int i = 0, n = neighborhoods.size(); i < n; i++)
    aNOIs[i].set(J(neighborhoods[i]));

  aNOIs.transform();
}

void Correspondences::loadBuckets(const DifferenceImage &J)
{
  const Integral &integral = J.integral;
  int rows = J.rows - side + 1;
  int cols = J.cols - side + 1;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      int k = integral.sum(i, j, side, side);
      if (k < cutoff)
        continue;

      buckets.add(k, cv::Point(j, i));
    }
  }
}

void Correspondences::loadReplay(int shift, const DifferenceImage &Jr)
{
  loadBuckets(Jr);

  neighborhoods.clear();
  cv::Mat active(Jr.rows - side + 1, Jr.cols - side + 1, CV_8U, ZERO);
  for (sorter::Sink<cv::Point> sink(buckets); sink.drain();)
  {
    const cv::Point &point = sink();
    int i = point.y;
    int j = point.x;

    if (active.at<uint8_t>(i, j) == 1)
      continue;

    int k = neighborhoods.size();
    cv::Rect &neighborhood = neighborhoods.append();
    neighborhood.x = clarus::truncate(j - padding2 - shift, 0, Jr.cols - side2);
    neighborhood.y = clarus::truncate(i - padding2, 0, Jr.rows - side2);
    neighborhood.width = side2;
    neighborhood.height = side2;

    aROIs[k].set(Jr.centered(i, j, side, side));

    if (neighborhoods.size() >= limit)
      break;

    cv::Rect roi;
    roi.x = std::max(j - side + 1, 0);
    roi.y = std::max(i - side + 1, 0);
    roi.width = std::min(side, active.cols - roi.x);
    roi.height = std::min(side, active.rows - roi.y);
    cv::rectangle(active, roi, ONE, CV_FILLED);
  }

  aROIs.transform();
}

} // namespace dich
