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

#ifndef DICH_CORRESPONDENCES_H
#define DICH_CORRESPONDENCES_H

#include <dich/difference_image.h>

namespace dich
{

namespace sorter = clarus::bucket_sort;

namespace fftw = clarus::fftw;

struct Correspondences
{
  /** \brief Maximum number of interest points considered. */
  int limit;

  /** \brief Padding around interest points. */
  int padding;

  /** \brief Padding around Regions-of-Interest. */
  int padding2;

  /** \brief Side length of Regions-of-Interest. */
  int side;

  /** \brief Side length of neighborhoods. */
  int side2;

  /** \brief Bucket sorter used to select interest points. */
  sorter::Buckets<cv::Point> buckets;

  /** \brief Interest point cutoff floor. */
  int cutoff;

  /** \brief Contents of selected Regions of Interest. */
  fftw::Signals aROIs;

  /** \brief Contents of selected Neighborhoods of Interest in a compared image. */
  fftw::Signals aNOIs;

  /** \brief Results of matching aROIs to aNOIs. */
  fftw::Signals matches;

  /** \brief Neighborhood boundaries for each aROI. */
  clarus::List<cv::Rect> neighborhoods;

  /** \brief Teach step record. */
  clarus::List<DifferenceImage> taught;

  /** \brief Teach step image normalization coefficients. */
  clarus::List<cv::Mat> coefficients;

  Correspondences();

  cv::Mat operator () (const DifferenceImage &J);

  cv::Mat operator () (const DifferenceImage &J, int y0, int yn);

  cv::Mat operator () (int shift, const DifferenceImage &J, int y0, int yn);

  void loadTaught();

  void loadBuckets(const DifferenceImage &J);

  void loadTeach(const DifferenceImage &J);

  void loadReplay(int shift, const DifferenceImage &J);
};

} // namespace dich

#endif
