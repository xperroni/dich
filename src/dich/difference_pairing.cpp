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

#include <dich/difference_pairing.h>
using clarus::List;

#include <dich/settings.h>

#include <limits>

namespace dich
{

DifferencePairing::DifferencePairing(bool kidnapped):
  samples(param::similarities::samples()),
  points(samples * param::similarities::width()),
  y0(0), yn(param::similarities::height()),
  line(0, 1.0),
  width(param::similarities::width() - 2),
  height(param::similarities::height()),
  error(param::similarities::error()),
  leak(param::similarities::leak())
{
  if (kidnapped)
  {
    yn = std::numeric_limits<int>::max() - 1;
    line.y = NAN;
  }
}

DifferenceImage DifferencePairing::operator() (const DifferenceImage &Jr)
{
  return (*this)(0, Jr);
}

DifferenceImage DifferencePairing::operator() (int shift, const DifferenceImage &Jr)
{
  cv::Mat column = correspondences(shift, Jr, y0, yn);
  update(Jr.j, column);

  int i = estimate(Jr.j);
  if (i < 0)
    return DifferenceImage();

  ROS_INFO_STREAM("Pairing: (" << i << ", " << Jr.j << ")");

  ROS_INFO_STREAM("Trend: " << line);

  return correspondences.taught[i];
}

cv::Point2f DifferencePairing::interpolate()
{
  cv::Point2f line(0, 0);
  int best = 0;

  int n = points.size();
  for (int i = 0; i < n; i++)
  {
    const cv::Point2f &p1 = points[i];
    for (int j = i + 1; j < n; j++)
    {
      const cv::Point2f &p2 = points[j];
      bool order = (p1.x <= p2.x);

      double x1 = (order ? p1.x : p2.x);
      double x2 = (order ? p2.x : p1.x);
      double y1 = (order ? p1.y : p2.y);
      double y2 = (order ? p2.y : p1.y);

      double dx = x2 - x1;
      double dy = y2 - y1;
      double m = dy / dx;
      if (dx < width || m < 0.1 || m > 10)
        continue;

      int count = 0;
      for (int k = 0; k < n; k++)
      {
        const cv::Point2f &p0 = points[k];
        double x0 = p0.x;
        double y0 = p0.y;

        double w = dy * x0 - dx * y0 + x2 * y1 - y2 * x1;
        double z = dy * dy + dx * dx;
        double d2 = w * w / z;
        if (d2 < error)
          count++;
      }

      if (count > best)
      {
        double b = y1 - m * x1;

        best = count;
        line.x = b;
        line.y = m;
      }
    }
  }

  return line;
}

List<int> DifferencePairing::select(const cv::Mat &column)
{
  List<double> values;
  List<int> indexes;

  int ys = y0 + samples;
  int rows = std::min(yn, column.rows);
  for (int i = y0; i < ys && i < rows; i++)
  {
    values.append(column.at<double>(i, 0));
    indexes.append(i);
  }

  for (int i = ys; i < rows; i++)
  {
    int index = 0;
    double lowest = values[0];
    for (int j = 1; j < samples; j++)
    {
      double value = values[j];
      if (value < lowest)
      {
        index = j;
        lowest = value;
      }
    }

    double value = column.at<double>(i, 0);
    if (lowest < value)
    {
      values[index] = value;
      indexes[index] = i;
    }
  }

  return indexes;
}

void DifferencePairing::update(int x, const cv::Mat &column)
{
  List<int> y = select(column);
  for (int i = 0, n = y.size(); i < n; i++)
    points.append(cv::Point2f(x, y[i]));

  if (points.idle())
    return;

  cv::Point2f fitted = interpolate();
  if (isnan(line.y))
    line = fitted;
  else
  {
    line.x = leak * line.x + (1.0 - leak) * fitted.x;
    line.y = leak * line.y + (1.0 - leak) * fitted.y;
  }

  y0 = std::max(estimate(x) - height / 2, 0);
  yn = y0 + height;
}

int DifferencePairing::estimate(int index)
{
  if (isnan(line.y))
    return -1;

  return clarus::truncate((int) (line.x + index * line.y), 0, (int) (correspondences.taught.size() - 1));
}

} // namespace dich
