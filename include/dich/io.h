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

#ifndef DICH_IO_H
#define DICH_IO_H

#include <dich/difference_image.h>

#include <clarus/clarus.hpp>

namespace dich
{

template<class T>
clarus::List<T> load(const std::string &path);

template<>
clarus::List<cv::Mat> load(const std::string &path);

template<>
clarus::List<DifferenceImage> load(const std::string &path);

} // namespace dich

#endif
