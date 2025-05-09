/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CIRCULAR_BUFFER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CIRCULAR_BUFFER_H_

#include <vector>

namespace imp::editor {

// A circular buffer implementation where the start of the buffer is denoted by
// a marker position, and is written specifically for use with the performance
// panel classes due to how ImPlot does circular processing of containers in its
// own way.
//
// Note: This mirrors gtl::CircularBuffer as close as reasonably possible,
// so follow their API if any new ones are getting added here.
template <typename T>
class CircularBuffer {
 public:
  // Creates a new CircularBuffer of the specified size. The size cannot be
  // modified after creation.
  explicit CircularBuffer(int size);

  // Adds a new element to the back of the buffer. If the buffer is full, it
  // overwrites the oldest value.
  void push_back(T val);

  // Returns the total number of elements in the buffer.
  int size();

  bool empty();

  // Returns the capacity of the buffer.
  int capacity() { return size_; }

  // Returns the position of the latest value in the buffer.
  int marker() { return pos_; }

  // Returns the latest value in the buffer.
  const T& back();

  // Returns a vector containing the buffered data.
  const std::vector<T>& data() { return data_; }

 private:
  std::vector<T> data_;
  int size_;
  int pos_;
};

template <typename T>
CircularBuffer<T>::CircularBuffer(int size) {
  size_ = size;
  pos_ = 0;
  data_.reserve(size);
}

template <typename T>
void CircularBuffer<T>::push_back(T val) {
  if (data_.size() < size_) {
    data_.push_back(val);
  } else {
    data_[pos_] = val;
    pos_ = (pos_ + 1) % size_;
  }
}

template <typename T>
int CircularBuffer<T>::size() {
  return data_.size();
}

template <typename T>
bool CircularBuffer<T>::empty() {
  return data_.empty();
}

template <typename T>
const T& CircularBuffer<T>::back() {
  if (data_.size() < size_) {
    return data_.back();
  } else {
    return data_[(pos_ - 1 + size_) % size_];
  }
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CIRCULAR_BUFFER_H_
