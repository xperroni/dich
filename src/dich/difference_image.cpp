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

#include <dich/difference_image.h>
using clarus::Integral;

namespace dich
{

DifferenceImage::DifferenceImage():
  cv::Mat(),
  i(-1),
  j(-1)
{
  // Nothing to do.
}

DifferenceImage::DifferenceImage(int i, int j, const cv::Mat &differences):
  cv::Mat(differences),
  integral(differences)
{
  this->i = i;
  this->j = j;
}

inline double valid(double value, double fallback)
{
  return (std::isnormal(value) && 0.000001 <= value && value < 1000000 ? value : fallback);
}

cv::Mat DifferenceImage::centered(const cv::Rect &roi) const
{
  cv::Mat patch = (*this)(roi) - integral.mean(roi);
  double v = 1.0 / clarus::magnitude(patch);
  return patch * (std::isnormal(v) ? v : 0.0);
}

cv::Mat DifferenceImage::centered(int i0, int j0, int rows, int cols) const
{
  return centered(cv::Rect(j0, i0, cols, rows));
}

cv::Mat DifferenceImage::normalized(const cv::Rect &roi) const
{
  double v = 1.0 / integral.magnitude(roi);
  return (*this)(roi) * (std::isnormal(v) ? v : 0.0);
}

cv::Mat DifferenceImage::normalized(int i0, int j0, int rows, int cols) const
{
  return normalized(cv::Rect(j0, i0, cols, rows));
}

cv::Mat DifferenceImage::magnitudeInv(int sr, int sc) const
{
  int rows = this->rows - sr + 1;
  int cols = this->cols - sc + 1;

  cv::Mat D(rows, cols, CV_64F);
  double *data = (double*) D.data;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++, data++)
    {
      *data = valid(1.0 / integral.magnitude(i, j, sr, sc), 0.0);
    }
  }

  return D;
}

cv::Mat DifferenceImage::stdDevInv(int sr, int sc) const
{
  int rows = this->rows - sr + 1;
  int cols = this->cols - sc + 1;

  cv::Mat D(rows, cols, CV_64F);
  double *data = (double*) D.data;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++, data++)
    {
      double s = 1.0 / integral.standardDeviation(i, j, sr, sc);
      *data = (std::isnormal(s) ? s : 0.0);
    }
  }

  return D;
}

cv::Mat bgr(const cv::Mat &J)
{
    cv::Mat grays = clarus::log(1 + clarus::max(J) - J);
    cv::normalize(grays, grays, 0, 255, CV_MINMAX, CV_8U);
    return colors::colormap(grays, cv::COLORMAP_HOT);
}

} // namespace dich
