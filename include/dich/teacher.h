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

#ifndef DICH_TEACHER_H
#define DICH_TEACHER_H

#include <cv_video/recorder.h>
#include <cv_video/video.h>
#include <terminal_io/terminal.h>
#include <yamabros/spur.h>

namespace dich
{

struct Teacher
{
  yamabros::spur::Spur spur;

  terminal_io::Terminal terminal;

  cv_video::Video video;

  cv_video::Recorder recorder;

  double v;

  double w;

  double V;

  double W;

  Teacher();

  void log(cv_video::Video &video, cv_video::Frame &frame);

  void input(terminal_io::Terminal &terminal, char key);

  void steer(double vt, double wt);
};

} // namespace dich

#endif
