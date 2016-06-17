/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Dejavu.

Dejavu is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Dejavu is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
*/

#include <dich/settings.h>

#include <cv_video/replayer.h>
using cv_video::Replayer;

#include <clarus/clarus.hpp>
using clarus::List;

#include <fstream>
#include <iostream>
#include <string>

namespace dich
{

static const int SCALE = 20;

static const int KEY_0     = 48;    //1048624;
static const int KEY_LEFT  = 65361; //1113937;
static const int KEY_RIGHT = 65363; //1113939;
static const int KEY_UP    = 65362; //1113938;
static const int KEY_DOWN  = 65364; //1113940;
static const int KEY_ENTER = 10;    //1048586;
static const int KEY_ESC   = 27;    //1048603;

static cv::Mat roll(const cv::Mat &image, int j)
{
  int rows = image.rows;
  int cols = image.cols;

  int k = (cols - j) % cols;
  cv::Mat image_l(image, cv::Rect(0, 0, k, rows));
  cv::Mat image_r(image, cv::Rect(k, 0, cols - k, rows));

  cv::Mat rolled(image.size(), image.type());
  cv::Mat rolled_l(rolled, cv::Rect(0, 0, image_r.cols, rows));
  cv::Mat rolled_r(rolled, cv::Rect(image_r.cols, 0, k, rows));

  image_l.copyTo(rolled_r);
  image_r.copyTo(rolled_l);

  return rolled;
}

static cv::Mat merge(const cv::Mat &teach, const cv::Mat &replay, int shift)
{
  cv::Mat shifted = teach;
  if (shift != 0)
  {
    shifted = roll(teach, shift);

    int cols = shifted.cols;
    int x = (shift < 0 ? cols + shift : 0);
    int w = abs(shift);

    cv::Rect roi(x, 0, w, shifted.rows);
    cv::Mat blank(shifted, roi);
    cv::Mat paste(replay, roi);
    paste.copyTo(blank);
  }

  cv::Mat merged;
  cv::addWeighted(shifted, 0.5, replay, 0.5, 0.0, merged);
  return merged;
}

struct TeachStream
{
  List<cv::Mat> images;

  TeachStream()
  {
    int gap = param::difference::exposure();
    Replayer replayer(param::path());
    for (;;)
    {
      cv::Mat image;
      for (int i = 0; i < gap; i++)
        image = replayer();

      if (image.empty())
        break;

      images.append(image);
    }
  }

  cv::Mat operator () (int index)
  {
    return images[index];
  }
};

struct ReplayStream
{
  Replayer replayer;

  int gap;

  ReplayStream():
    replayer(param::path_replay()),
    gap(param::difference::exposure())
  {
    // Nothing to do.
  }

  cv::Mat operator () ()
  {
    cv::Mat image;
    for (int i = 0; i < gap; i++)
      image = replayer();

    return image;
  }
};

struct GroundTruth
{
  TeachStream teachStream;

  ReplayStream replayStream;

  int last;

  std::ofstream out;

  int index;

  int shift;

  int gap;

  GroundTruth():
    last(teachStream.images.size() - 1),
    out(param::path_ground_truth().c_str()),
    index(0),
    shift(0),
    gap(replayStream.gap)
  {
    // Nothing to do.
  }

  void adjust(int i)
  {
    cv::Mat replay = replayStream();
    if (replay.empty())
    {
      ros::shutdown();
      return;
    }

    while (ros::ok())
    {
      cv::Mat teach = teachStream(index);
      cv::Mat merged = merge(teach, replay, shift);
      viewer::show("Ground Truth", merged);

      int key = cv::waitKey();
      switch (key)
      {
        case KEY_ESC: ros::shutdown(); break;

        case KEY_LEFT  : shift--;   break;
        case KEY_RIGHT : shift++;   break;
        case KEY_0     : shift = 0; break;

        case KEY_UP   : index = (index == last ? last : index + 1); break;
        case KEY_DOWN : index = (index == 0 ? 0 : index - 1); break;

        case KEY_ENTER:
        {
          List<int> match = (List<int>(), gap * i, gap * index, shift);
          std::cerr << "Point: " << match << std::endl;
          out << match << std::endl;
          index++;
          return;
        }

        default: std::cerr << "Unknown key: " << key << std::endl;
      }
    }
  }

  void spin()
  {
    for (int i = 0; ros::ok(); i++)
      adjust(i);
  }
};

} // namespace dejavu

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ground_truth");

  dich::GroundTruth ground_truth;

  ground_truth.spin();

  return 0;
}
