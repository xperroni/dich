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

#ifndef DICH_WORKING_MEMORY_H
#define DICH_WORKING_MEMORY_H

#include <clarus/clarus.hpp>

namespace dich
{

template<class T>
class WorkingMemory
{
  clarus::List<T> buffer_;

  size_t size_;

  size_t offset_;

  inline int roll(int index) const
  {
    int size = buffer_.size();
    return ((index < 0 ? size + index : index) + offset_) % size;
  }

public:
  WorkingMemory(size_t size);

  clarus::List<T>& operator * ();

  const clarus::List<T>& operator * () const;

  T &operator [] (int index);

  const T &operator [] (int index) const;

  T &append(const T &data);

  bool full() const;

  bool idle() const;

  size_t size() const;
};

template<class T>
WorkingMemory<T>::WorkingMemory(size_t size):
  size_(size),
  offset_(0)
{
  // Nothing to do.
}

template<class T>
clarus::List<T>& WorkingMemory<T>::operator * ()
{
    return buffer_;
}

template<class T>
const clarus::List<T>& WorkingMemory<T>::operator * () const
{
    return buffer_;
}

template<class T>
T &WorkingMemory<T>::operator [] (int index)
{
  return buffer_[roll(index)];
}

template<class T>
const T &WorkingMemory<T>::operator [] (int index) const
{
  return buffer_[roll(index)];
}

template<class T>
T &WorkingMemory<T>::append(const T &data)
{
  if (buffer_.size() < size_)
    return buffer_.append(data);

  int index = offset_;
  offset_ = (index + 1) % size_;
  return (buffer_[index] = data);
}

template<class T>
bool WorkingMemory<T>::full() const
{
  return (buffer_.size() >= size_);
}

template<class T>
bool WorkingMemory<T>::idle() const
{
  return (buffer_.size() < size_);
}

template<class T>
size_t WorkingMemory<T>::size() const
{
  return size_;
}

} // namespace dich

#endif
