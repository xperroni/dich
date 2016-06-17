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

#ifndef DICH_DIFFERENCE_IMAGE_H
#define DICH_DIFFERENCE_IMAGE_H

#include <clarus/clarus.hpp>

namespace dich
{

struct DifferenceImage: public cv::Mat
{
  /** \brief Index of the later image used to create this difference image. */
  int i;

  /** \brief Difference image index. */
  int j;

  /** \brief Integral image of the difference image. */
  clarus::Integral integral;

  /**
   * \brief Creates an empty difference image.
   *
   * Both indexes are set to \c -1 to emphasize the image's invalid status.
   */
  DifferenceImage();

  /**
   * \brief Creates a new Difference Image object with given attributes.
   */
  DifferenceImage(int i, int j, const cv::Mat &differences);

  /**
   * \brief Compute the normalization factors for a sliding cosine to a template of given dimensions.
   */
  cv::Mat magnitudeInv(int sr, int sc) const;

  /**
   * \brief Compute the normalization factors for a cross-correlation to a template of given dimensions.
   */
  cv::Mat stdDevInv(int sr, int sc) const;

  /**
   * \brief Return the contents of the given area, centered and normalized.
   */
  cv::Mat centered(const cv::Rect &roi) const;

  /**
   * \brief Return the contents of the given area, centered and normalized.
   */
  cv::Mat centered(int i0, int j0, int rows, int cols) const;

  /**
   * \brief Return the contents of the given area, normalized.
   */
  cv::Mat normalized(const cv::Rect &roi) const;

  /**
   * \brief Return the contents of the given area, normalized.
   */
  cv::Mat normalized(int i0, int j0, int rows, int cols) const;
};

/**
 * \brief Convert the difference image to a color map.
 */
cv::Mat bgr(const cv::Mat &J);

} // namespace dich

#endif
