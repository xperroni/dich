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

#ifndef DICH_SETTINGS_H
#define DICH_SETTINGS_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace dich
{

namespace name
{

/**
 * \brief Topic for image data.
 */
std::string image();

/**
 * \brief Topic for image match data.
 */
std::string match();

} // namespace name

namespace param
{

template<class T>
T param(const std::string& name, const T& fallback)
{
  T value;
  return (ros::param::get(name, value) ? value : fallback);
}

/**
 * \brief Robot angular velocity.
 */
double ang_vel();

/**
 * \brief Robot linear velocity.
 */
double lin_vel();

/**
 * \brief Path to a video record.
 */
std::string path();

/**
 * \brief Path to an input replay video record.
 */
std::string path_replay_offline();

/**
 * \brief Path to an output replay video record.
 */
std::string path_replay_online();

/**
 * \brief Path to a steerings record.
 */
std::string path_steerings();

/**
 * \brief Path to output ground truth file.
 */
std::string path_ground_truth();

/**
 * \brief Whether nodes should be executed in headless mode (i.e., without GUI).
 */
bool headless();

namespace difference
{

/**
 * \brief Number of frames collected for each difference image.
 */
int exposure();

/**
 * \brief Log difference threshold for difference image pixels.
 */
double threshold();

/**
 * \brief Size of the scaled difference image.
 *
 * Equivalent to `cv::Size(param::difference::width(), param::difference::height())`.
 */
cv::Size size();

/**
 * \brief Width of the scaled difference image.
 */
int width();

/**
 * \brief Height of the scaled difference image.
 */
int height();

} // namespace difference

namespace interest
{

/**
 * \brief Padding around interest points.
 */
int padding();

/**
 * \brief Padding around aROIs.
 */
int padding2();

/**
 * \brief Number of interest points sought in difference images.
 */
int points();

/**
 * \brief Rate at which interest points are preemptively discarded.
 */
double cutoff();

} // namespace interest

namespace similarities
{

/**
 * \brief Maximum acceptable interpolation error for image pairing estimation.
 */
double error();

/**
 * \brief Number of samples extracted from the similarity matrix.
 */
int samples();

/**
 * \brief Height of the similarity matrix.
 */
int height();

/**
 * \brief Update ratio of the pairing estimation.
 */
double leak();

/**
 * \brief Width of the similarity matrix.
 */
int width();

} // namespace similarities

namespace shift
{

/**
 * \brief Width of a shift estimation column.
 */
int column();

/**
 * \brief Width of the shift estimation window.
 */
int width();

} // namespace shift

namespace replay
{

int queue();

} // namespace replay

} // namespace param

} // namespace dich

#endif
