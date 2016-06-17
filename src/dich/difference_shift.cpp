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

#include <dich/difference_shift.h>
using clarus::ZERO;

#include <dich/settings.h>

namespace dich
{

DifferenceShift::DifferenceShift():
  rows(param::difference::height()),
  cols(param::shift::column()),
  width(param::shift::width()),
  columns(1 + param::difference::width() / cols, param::difference::size(), fftw::FORWARD_R2C),
  matches(0 + param::difference::width() / cols, param::difference::size(), fftw::BACKWARD_C2R),
  correlations(param::similarities::width()),
  shifts(1, param::difference::width(), CV_64F, ZERO),
  estimate(0)
{
  // Nothing to do.
}

int DifferenceShift::operator () (const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  cv::Mat correlation = shiftVector(Jr, Jt);
  if (correlations.full())
    shifts -= correlations[0];

  correlations.append(correlation);
  shifts += correlation;

  int j0 = 0;
  int jn = shifts.cols;
  if (correlations.full())
  {
    j0 = clarus::truncate((int) (estimate - 0.5 * width), 0, shifts.cols - width);
    jn = j0 + width;
  }

  double u = 0;
  double v = 0;
  for (int j = j0; j < jn; j++)
  {
    double s = shifts.at<double>(0, j);
    double w = s * s;
    u += j * w;
    v += w;
  }

  estimate = u / v;

  ROS_INFO_STREAM("Shift: (" << Jr.j << ", " << estimate << ", " << shifts << ")");

  return estimate - 0.5 * shifts.cols;
}

void DifferenceShift::correlate(const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  fftw::Signal &replay = columns[0];
  replay.set(Jr);

  for (int i = 1, n = columns.size(); i < n; i++)
  {
    int j = (i - 1) * cols;
    columns[i].set(Jt.normalized(0, j, rows, cols));
  }

  columns.transform();

  double s = 1.0 / (Jr.rows * Jr.cols);
  for (int j = 0, n = matches.size(); j < n; j++)
    matches[j].C.mul(columns[j + 1], replay, -1, s);

  matches.transform();
}

cv::Mat DifferenceShift::shiftVector(const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  correlate(Jr, Jt);

  cv::Mat D = Jr.magnitudeInv(rows, cols);
  cv::Mat S(1, shifts.cols, CV_64F, ZERO);
  int c2 = S.cols / 2;

  cv::Rect roi(0, 0, D.cols, D.rows);
  for (int j = 0, n = matches.size(); j < n; j++)
  {
    cv::Mat R = cv::Mat(matches[j].toMat(), roi).mul(D);

    int xS = std::max(0, c2 - j * cols);
    int xR = std::max(0, j * cols - c2);
    int wR = std::min(R.cols - xS, R.cols - xR);

    cv::Rect roiS(xS, 0, wR, 1);
    cv::Rect roiR(xR, 0, wR, 1);

    S(roiS) += R(roiR);
  }

  ROS_INFO_STREAM("Shift correlations: (" << Jr.j << ", " << S << ")");

  return S;
}

} // namespace dich
