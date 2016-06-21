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

#ifndef DICH_DIFFERENCE_PAIRING_H
#define DICH_DIFFERENCE_PAIRING_H

#include <dich/correspondences.h>
#include <dich/working_memory.h>

namespace dich
{

struct DifferencePairing
{
  Correspondences correspondences;

  int samples;

  WorkingMemory<cv::Point2f> points;

  int y0;

  int yn;

  cv::Point2f line;

  int width;

  int height;

  double error;

  double leak;

  DifferencePairing(bool kidnapped = true);

  DifferenceImage operator() (const DifferenceImage &J);

  DifferenceImage operator() (int shift, const DifferenceImage &J);

  int estimate(int index);

  cv::Point2f interpolate();

  clarus::List<int> select(const cv::Mat &column);

  void update(int x, const cv::Mat &column);
};

} // namespace dich

#endif
