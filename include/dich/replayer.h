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

#ifndef DICH_REPLAYER_H
#define DICH_REPLAYER_H

#include <dich/difference_filter.h>
#include <dich/difference_pairing.h>
#include <dich/difference_shift.h>

#include <cv_video/video.h>

namespace dich
{

struct Replayer
{
  ros::NodeHandle node;

  ros::Publisher publisher;

  cv_video::Video video;

  DifferenceFilter filter;

  DifferencePairing pairing;

  DifferenceShift shift;

  cv::Mat similarity_map;

  clarus::List<cv::Point> correspondences;

  cv::Mat shift_map;

  clarus::List<cv::Point> shifts;

  int shift_estimate;

  Replayer();

  void handle(cv_video::Video &video, cv_video::Frame &frame);

  void process(const DifferenceImage &Jr);

  void display(cv_video::Video &video, cv_video::Frame &frame);

  void plotSimilarities(const cv::Mat &column);

  void plotShifts(const cv::Mat &shift_vector);
};

} // namespace dich

#endif
