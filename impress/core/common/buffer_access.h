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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_BUFFER_ACCESS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_BUFFER_ACCESS_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace imp {

// BufferAccess abstracts the source of a buffer from access to its data.  It
// contains a view and an optional unique pointer.  The latter will be null e.g.
// when the BufferAccess is pointing into static data from our executable, or
// into java memory.
class BufferAccess {
 public:
  BufferAccess() : BufferAccess(nullptr, 0) {}
  BufferAccess(std::unique_ptr<uint8_t[]>&& storage, size_t size);
  BufferAccess(std::unique_ptr<uint8_t[]>&& storage,
               absl::Span<const uint8_t> view);

  // Copies an existing buffer into a BufferAccess.
  static BufferAccess Clone(const uint8_t* data, size_t size);

  // Wrap an existing buffer, does not take ownership.  Pointer must remain
  // valid for the lifetime of BufferAccess.
  static BufferAccess Wrap(const uint8_t* data, size_t size);

  // Create a BufferAccess with the given size.
  // Returns a mutable pointer to the buffer.
  static uint8_t* Create(size_t size, BufferAccess* access);

  absl::Span<const uint8_t> View() const { return view_; }
  absl::string_view StringView() const {
    return absl::string_view{reinterpret_cast<const char*>(Data()), Size()};
  }
  const uint8_t* Data() const { return view_.data(); }
  size_t Size() const { return view_.size(); }
  bool Empty() const { return !Size(); }
  explicit operator bool() const noexcept { return !Empty(); }

  // Helper method for javascript binding - a js-wrapped BufferAccess will
  // be a ref-counted shared pointer; this allows for a js-wrapped
  // BufferAccess to release ownership of the contained storage (e.g. to grant
  // it to a LoadedBundle).
  BufferAccess ReleaseOwnership();

  // Helper method for mediapipe::ImageFrame binding to the buffer data without
  // a copy. Usage example:
  //   ImageFrame image;
  //   image.AdoptRawImage(ImageFormat::SRGBA, width, height, 4 * width,
  //                       buffer_access.ReleaseDataOwnership().release());
  std::unique_ptr<uint8_t[]> ReleaseDataOwnership();

  // Makes a copy of unowned data and owns it.
  void GainOwnershipByCopying();

 private:
  BufferAccess(const uint8_t* data, size_t size);
  std::unique_ptr<uint8_t[]> optional_storage_;
  absl::Span<const uint8_t> view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BUFFER_ACCESS_H_
