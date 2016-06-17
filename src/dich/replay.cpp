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
using dich::Replayer;

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <yamabros/robot.h>
using yamabros::Robot;

#include <fftw3.h>

int main(int argc, char *argv[])
{
  fftw_init_threads();
  fftw_plan_with_nthreads(8);

  Robot robot;
  robot.init("replay", argc, argv);
  robot.set("replayer", new Replayer());
  robot.spin();

  return 0;
}
