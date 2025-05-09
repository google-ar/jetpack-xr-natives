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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_FRAME_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_FRAME_H_

#include <functional>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "absl/time/time.h"
#include "core/ar/ar_trackable_helpers.h"
#include "core/render/texture.h"

namespace imp {
namespace ar {

// A cross-platform representation of the state that can change in an AR frame.
// This is the primary structure for ArSessionNative implementations to
// express their tracking information.
class ArFrame {
 public:
  // Struct to support YUV420 format CPU image.
  struct YUV420Image {
    const uint8_t* y_ptr = nullptr;
    const uint8_t* u_ptr = nullptr;
    const uint8_t* v_ptr = nullptr;
    int32_t width;
    int32_t height;
    int32_t y_buffer_size = 0;
    int32_t u_buffer_size = 0;
    int32_t v_buffer_size = 0;
    // Y is not interleaved. U and V are guaranteed to have the same stride.
    int32_t uv_stride = 1;
    std::unique_ptr<std::function<void()>> on_delete = nullptr;
    ~YUV420Image() {
      if (on_delete != nullptr) {
        (*on_delete)();
      }
    }
  };

  ArFrame(absl::Time timestamp, const imp::Texture* camera_texture,
          intptr_t camera_texture_id, mat4 projection_matrix,
          mat4f model_matrix, TrackableTuple&& updated_trackables,
          std::unique_ptr<ArFrame::YUV420Image> yuv_image = nullptr,
          absl::optional<mat4f> camera_node_matrix = absl::nullopt)
      : timestamp_(timestamp),
        camera_texture_(camera_texture),
        camera_texture_id_(camera_texture_id),
        projection_matrix_(projection_matrix),
        model_matrix_(model_matrix),
        camera_node_matrix_(camera_node_matrix),
        updated_trackables_(std::move(updated_trackables)),
        yuv_image_(nullptr) {}

  // The time in nanoseconds that the frame represents.
  absl::Time timestamp() const { return timestamp_; }
  // The struct containing cpu image information.
  const YUV420Image* camera_image() const { return yuv_image_.get(); }
  // Hands over the ownership of the struct containing cpu image information.
  std::unique_ptr<YUV420Image> move_camera_image() {
    return std::move(yuv_image_);
  }
  // The camera texture that the frame represents.
  const imp::Texture* camera_texture() const { return camera_texture_; }
  // A handle to the camera texture. In GL, this value should be static_cast to
  // a GLuint.
  const intptr_t camera_texture_id() const { return camera_texture_id_; }
  // The projection matrix to use for the virtual camera.
  const mat4& projection_matrix() const { return projection_matrix_; }
  // The model matrix to use for placing the virtual camera.
  const mat4f& model_matrix() const { return model_matrix_; }
  // An optional transform for the camera node distinct from the virtual camera.
  const absl::optional<mat4f>& camera_node_matrix() const {
    return camera_node_matrix_;
  }
  // The list of trackables updated this frame.
  template <typename T>
  const std::vector<T>& GetUpdatedTrackables() {
    return std::get<std::vector<T>>(updated_trackables_);
  }

  // Gets the tuple of trackable lists.
  const TrackableTuple& GetUpdatedTrackablesTuple() {
    return updated_trackables_;
  }

 private:
  absl::Time timestamp_;
  const imp::Texture* camera_texture_;
  intptr_t camera_texture_id_;
  mat4 projection_matrix_;
  mat4f model_matrix_;
  absl::optional<mat4f> camera_node_matrix_;
  TrackableTuple updated_trackables_;
  std::unique_ptr<YUV420Image> yuv_image_;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_FRAME_H_
