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

#include <tbb/tbb.h>

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

int DifferenceShift::operator () (int fallback, const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  int estimated = (*this)(Jr, Jt);
  return (correlations.full() ? estimated : fallback);
}

struct match
{
  fftw::Signal &replay;

  fftw::Signals &columns;

  fftw::Signals &matches;

  double s;

  match(fftw::Signals &columns_, double s_, fftw::Signals &matches_):
    replay(columns_[0]),
    columns(columns_),
    matches(matches_),
    s(s_)
  {
    tbb::parallel_for(tbb::blocked_range<int>(0, matches.size()), *this);

    matches.transform();
  }

  void operator() (const tbb::blocked_range<int> &r) const
  {
    int k0 = r.begin();
    int kn = r.end();
    for (int k = k0; k < kn; k++)
      matches[k].C.mul(columns[k + 1], replay, -1, s);
  }
};

void DifferenceShift::correlate(const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  fftw::Signal &replay = columns[0];
  replay.set(fourier::normalize(Jr));

  for (int i = 1, n = columns.size(); i < n; i++)
  {
    cv::Rect roi((i - 1) * cols, 0, cols, rows);
    columns[i].set(fourier::normalize(Jt(roi)));
  }

  columns.transform();

  double s = 1.0 / (Jr.rows * Jr.cols);
  for (int j = 0, n = matches.size(); j < n; j++)
    matches[j].C.mul(columns[j + 1], replay, -1, s);

  matches.transform();

//   match(columns, s, matches);
}

cv::Mat DifferenceShift::shiftVector(const DifferenceImage &Jr, const DifferenceImage &Jt)
{
  correlate(Jr, Jt);

  int reach = Jr.cols - cols;
  cv::Mat likelihoods(1, reach * 2, CV_64F, cv::Scalar::all(0));

  for (int j = 0, n = Jr.cols / cols; j < n; j++)
  {
    cv::Mat correlated(matches[j].toMat(), cv::Rect(0, 0, reach, 1));
    cv::Rect roi(reach - j * cols, 0, reach, 1);
    likelihoods(roi) += correlated;
  }

  cv::Mat vector(likelihoods, cv::Rect(reach - shifts.cols / 2, 0, shifts.cols, 1));

  ROS_INFO_STREAM("Shift correlations: (" << Jr.j << ", " << vector << ")");

  return vector;
}

} // namespace dich
