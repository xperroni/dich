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

#include <dich/replayer.h>
using clarus::List;
using cv_video::Video;
using cv_video::Frame;

#include <dich/io.h>
#include <dich/settings.h>

#include <ros/console.h>

#include <std_msgs/Int32.h>

namespace dich
{

static const cv::Scalar BLUE(255, 0, 0);

Replayer::Replayer():
  pairing(false),
  shift_estimate(0)
{
  std::string path_replay = param::path_replay();
  if (path_replay != "")
  {
    List<DifferenceImage> replay = load<DifferenceImage>(path_replay);
    for (int i = 0, n = replay.size(); i < n; i++)
      process(replay[i]);

    ros::shutdown();

    return;
  }

  correspondences.append(cv::Point(0, 0));

  if (param::headless())
    video.subscribe("replay", param::replay::queue(), &Replayer::handle, this);
  else
    video.subscribe("replay", param::replay::queue(), &Replayer::display, this);

  publisher = node.advertise<std_msgs::Int32>(name::index(), 1, true);

  std_msgs::Int32 message;
  message.data = -1;
  publisher.publish(message);
}

void Replayer::handle(Video &video, Frame &frame)
{
  DifferenceImage Jr = filter(frame.share());
  if (Jr.empty())
    return;

  process(Jr);
}

void Replayer::process(const DifferenceImage &Jr)
{
  DifferenceImage Jt = pairing(shift_estimate, Jr);
  if (Jt.empty())
    return;

  shift_estimate = shift(shift_estimate, Jr, Jt);

  std::cerr << cv::Point3i(Jr.j, Jt.j, shift_estimate) << std::endl;
}

void Replayer::display(Video &video, Frame &frame)
{
  DifferenceImage Jr = filter(frame.share());
  if (Jr.empty())
    return;

  cv::Mat column = pairing.correspondences(shift_estimate, Jr, pairing.y0, pairing.yn);
  //cv::Mat column = pairing.correspondences(Jr, pairing.y0, pairing.yn);

  pairing.update(Jr.j, column);
  if (pairing.points.full())
  {
    int i = pairing.estimate(Jr.j);
    const DifferenceImage &Jt = pairing.correspondences.taught[i];
    shift_estimate = shift(Jr, Jt);
    int k = shift.estimate;

    correspondences.append(cv::Point(Jr.j, i));
    shifts.append(cv::Point(shifts.size(), shift.estimate));

    viewer::show("Teach", bgr(Jt));
    plotShifts(shift.shifts);
  }

  std::cerr << frame.index() << std::endl;

  viewer::show("Replay", bgr(Jr));
  plotSimilarities(column);
}

static cv::Mat normalize(const cv::Mat &data)
{
  cv::Mat normalized = data - clarus::min(data);
  double w = clarus::max(normalized);
  if (w != 0)
    normalized /= w;

  return normalized;
}

void Replayer::plotSimilarities(const cv::Mat &column)
{
  cv::Mat normalized = normalize(column);
  if (similarity_map.empty())
    similarity_map = normalized;
  else
    cv::hconcat(similarity_map, normalized, similarity_map);

  cv::Mat image = bgr(similarity_map);
  for (int i = 0, n = pairing.points.size(); i < n; i++)
  {
    cv::Point2f p = pairing.points[i];
    int x = p.x;
    int y = p.y;

    cv::Vec3b &pixel = image.at<cv::Vec3b>(y, x);
    pixel[0] = 255;
    pixel[1] = 0;
    pixel[2] = 0;
  }

  if (correspondences.size() >= 2)
  {
    cv::Point p1 = correspondences[0];
    for (int i = 1, n = correspondences.size(); i < n; i++)
    {
      cv::Point p2 = correspondences[i];
      cv::line(image, p1, p2, BLUE, 3);
      p1 = p2;
    }
  }

  viewer::show("Similarities", image);
}

void Replayer::plotShifts(const cv::Mat &shift_vector)
{
  cv::Mat normalized = normalize(shift_vector).t();
  if (shift_map.empty())
    shift_map = normalized;
  else
    cv::hconcat(shift_map, normalized, shift_map);

  cv::Mat image = bgr(shift_map);
  if (shifts.size() >= 2)
  {
    cv::Point p1 = shifts[0];
    for (int i = 1, n = shifts.size(); i < n; i++)
    {
      cv::Point p2 = shifts[i];
      cv::line(image, p1, p2, BLUE, 3);
      p1 = p2;
    }
  }

  viewer::show("Shifts", image);
}

} // namespace dich
