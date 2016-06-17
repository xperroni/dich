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

#ifndef DICH_DIFFERENCE_FILTER_H
#define DICH_DIFFERENCE_FILTER_H

#include <dich/difference_image.h>

namespace dich
{

class DifferenceFilter
{
  /** \brief Number of frames accumulated in a single difference image. */
  int exposure_;

  /** \brief Log difference threshold. */
  double threshold_;

  /** \brief Width of scaled difference images. */
  int width_;

  /** \brief Last received frame that was used to create a difference image. */
  cv::Mat previous_;

  /** \brief The accumulated difference image. */
  cv::Mat accumulated_;

  /** \brief Number of frames currently accumulated. */
  int count_;

  /** \brief Index of the latest raw image used to generate a Difference Image. */
  int i_;

  /** \brief Index of the latest Difference Image returned. */
  int j_;

public:
  /**
   * \brief Default constructor.
   */
  DifferenceFilter();

  /**
   * \brief Create a new difference image filter of given parameters.
   */
  DifferenceFilter(int exposure, double threshold, int width);

  /**
   * \brief Return a valid difference image if possible, or an empty image otherwise..
   */
  DifferenceImage operator () (const cv::Mat &frame);
};

} // namespace dich

#endif
