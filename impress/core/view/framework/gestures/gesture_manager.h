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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_MANAGER_H_

#include <vector>

#include "absl/algorithm/container.h"
#include "absl/memory/memory.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

class View;

/**
 * The GestureManager detects various gestures based on the type of gestures
 * called with AddGestureRecognizer. It also contains default gestures such as
 * drag.
 */
class GestureManager {
 public:
  // Detects and sends gestures to and from this `Dispatcher`.
  explicit GestureManager(Dispatcher* dispatcher);

  template <typename T>
  void AddGestureRecognizer();

  template <typename T>
  void RemoveGestureRecognizer();

  void OnPointerHitEvent(const PointerHitEvent& hit_event);

  template <typename T>
  absl::Span<const T> GetGestures() const;

 private:
  Dispatcher* dispatcher_;
  GesturePointerUtils pointer_utils_;

  /** Base class for the templated GestureRecognizer class. */
  class BaseGestureRecognizer {
   public:
    BaseGestureRecognizer() : last_geture_id_(Gesture::kEmptyId) {}
    virtual ~BaseGestureRecognizer() {}
    // Handler for touch input data from a PointerHitEvent.
    virtual void OnPointerHitEvent(
        const PointerHitEvent& pointer_hit_event) = 0;

   protected:
    Gesture::Id NextId() {
      // Always expected to be called from the main thread.
      return ++last_geture_id_;
    }

   private:
    Gesture::Id last_geture_id_;
  };

  /**
   * GestureRecognizer creates potential Gesture objects for pointers from
   * pointer input and propagates PointerHitEvents to start and/or update them.
   */
  template <typename T>
  class GestureRecognizer final : public BaseGestureRecognizer {
   public:
    using TryCreateGestureFn = std::function<absl::optional<T>(
        const PointerHitEvent& pointer_hit, absl::Span<const T> gestures)>;
    explicit GestureRecognizer(TryCreateGestureFn create_gesture);
    void OnPointerHitEvent(const PointerHitEvent& pointer_hit_event) override;
    absl::Span<const T> GetGestures() const {
      return absl::MakeSpan(gestures_);
    }

   private:
    TryCreateGestureFn try_create_;
    std::vector<T> gestures_;
    // Remove cancelled and/or finished gestures from the gesture vector.
    void RemoveCompletedGestures();
  };

  std::vector<std::unique_ptr<BaseGestureRecognizer>> recognizers_;
  std::vector<HashValue> recognizer_hashes_;
};

template <typename T>
void GestureManager::AddGestureRecognizer() {
  recognizers_.push_back(
      std::make_unique<GestureRecognizer<T>>(GestureRecognizer<T>(
          T::GetCreateFunction(dispatcher_, &pointer_utils_))));
  recognizer_hashes_.push_back(type_traits::kTypeHash<T>);
}

template <typename T>
void GestureManager::RemoveGestureRecognizer() {
  auto iter = absl::c_find(recognizer_hashes_, type_traits::kTypeHash<T>);
  if (iter == recognizer_hashes_.end()) return;
  size_t index = std::distance(recognizer_hashes_.begin(), iter);
  recognizers_.erase(recognizers_.begin() + index);
  recognizer_hashes_.erase(recognizer_hashes_.begin() + index);
}

template <typename T>
absl::Span<const T> GestureManager::GetGestures() const {
  auto iter = absl::c_find(recognizer_hashes_, type_traits::kTypeHash<T>);
  if (iter == recognizer_hashes_.end()) return {};
  size_t index = std::distance(recognizer_hashes_.begin(), iter);
  GestureRecognizer<T>* recognizer =
      static_cast<GestureRecognizer<T>*>(recognizers_[index].get());
  return recognizer->GetGestures();
}

template <typename T>
GestureManager::GestureRecognizer<T>::GestureRecognizer(
    TryCreateGestureFn create_gesture)
    : try_create_(std::move(create_gesture)), gestures_() {}

template <typename T>
void GestureManager::GestureRecognizer<T>::OnPointerHitEvent(
    const PointerHitEvent& pointer_hit_event) {
  // Instantiate gestures based on touch input.
  // Just because a gesture was created, doesn't mean that it is started.
  // For example, a DragGesture is created when the user touch's down,
  // but doesn't actually start until the touch has moved beyond a threshold.
  absl::optional<T> opt_gesture =
      try_create_(pointer_hit_event, absl::MakeSpan(gestures_));
  if (opt_gesture.has_value()) {
    // Use an InitializeId method instead of passing the id through the
    // try_create function to avoid the extra complexity caused by the case
    // where try_create does not instantiate a geture and therefore the id
    // shouldn't be incremented.
    opt_gesture.value().InitializeId(NextId());
    gestures_.push_back(std::move(*opt_gesture));
  }

  for (T& gesture : gestures_) {
    gesture.OnPointerHitEvent(pointer_hit_event);
  }

  RemoveCompletedGestures();
}

template <typename T>
void GestureManager::GestureRecognizer<T>::RemoveCompletedGestures() {
  gestures_.erase(
      std::remove_if(
          gestures_.begin(), gestures_.end(),
          [](T& gesture) { return gesture.Finished() || gesture.Cancelled(); }),
      gestures_.end());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_GESTURE_MANAGER_H_
