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

#ifndef DICH_DIFFERENCE_SHIFT_H
#define DICH_DIFFERENCE_SHIFT_H

#include <dich/difference_image.h>
#include <dich/working_memory.h>

namespace dich
{

namespace fftw = clarus::fftw;

struct DifferenceShift
{
  /** \brief Height of a shift estimation column. */
  int rows;

  /** \brief Width of a shift estimation column. */
  int cols;

  /** \brief Width of the shift estimation window */
  int width;

  /** \brief Shift estimation forward transform. */
  fftw::Signals columns;

  /** \brief Shift estimation backward transform. */
  fftw::Signals matches;

  /** \brief Recent teach / replay correlation vectors. */
  WorkingMemory<cv::Mat> correlations;

  /** \brief Shift likelihood vector. */
  cv::Mat shifts;

  /** \brief Current shift estimate. */
  int estimate;

  /**
   * \brief Default constructor.
   */
  DifferenceShift();

  /**
   * \brief Compute shift between the given images.
   */
  int operator () (const DifferenceImage &Jr, const DifferenceImage &Jt);

  void correlate(const DifferenceImage &Jr, const DifferenceImage &Jt);

  cv::Mat shiftVector(const DifferenceImage &Jr, const DifferenceImage &Jt);
};

} // namespace dich

#endif
