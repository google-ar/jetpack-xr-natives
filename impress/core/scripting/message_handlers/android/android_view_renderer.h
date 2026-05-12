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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_RENDERER_H_

#include <jni.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "core/actions/controller_events.h"
#include "core/async/future.h"
#include "core/common/jni_helpers.h"
#include "core/geometry/shapes/box.h"
#include "core/input/pointer_event_processor.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/primitive_shape_renderer.h"
#include "core/render/texture.h"
#include "core/scripting/message_handlers/android/android_view_renderer.proto.imp.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/view.h"
#include "core/view/platforms/android/wrappers/motion_event.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/materials/texture_external_material.h"

namespace imp::android {

// JNI wrapper for the RenderViewToSurfaceTexture class.
class RenderViewToSurfaceTextureWrapper : public JavaWrapper {
 public:
  explicit RenderViewToSurfaceTextureWrapper(
      NodeHandle node, jobject android_view, ViewSize view_size,
      float corner_radius, AndroidExternalTextureSurface& surface);
  ~RenderViewToSurfaceTextureWrapper() override;

  BorrowedTexturePtr BorrowTexture();

  // Returns the size of the surface in pixels.
  int2 GetSize() const;
  // Sets the surface buffer size and scale of the node to match aspect ratio.
  void SetSize(int2 size);

  // Internally, this calls view.dispatchGenericMotionEvent(motion_event) on the
  // Java side.
  void DispatchGenericMotionEvent(MotionEvent& motion_event);
  // Internally, this calls view.dispatchTouchEvent(motion_event) on the Java
  // side.
  void DispatchTouchEvent(MotionEvent& motion_event);

 private:
  NodeHandle node_;
  AndroidExternalTextureSurface& surface_;
  JniHandle release_;
  JniHandle dispatch_generic_motion_event_;
  JniHandle dispatch_touch_event_;
  ViewSize view_size_;
};

// Component to manage rendering and lifetime.
class AndroidViewRenderer : public Component {
 public:
  // Ensure that the PrimitiveShapeRenderer is cleaned up before this wrapper.
  using CleanupDependencies = CleanupIds<PrimitiveShapeRenderer>;

  Future<absl::Status> Setup(
      jobject android_view, ViewSize view_size,
      InputForwardingMode input_forwarding_mode =
          InputForwardingMode::INPUT_FORWARDING_MODE_DEFAULT,
      absl::optional<imp::MaterialDefinition> material_definition =
          std::nullopt,
      absl::optional<uint32_t> blend_priority = std::nullopt,
      absl::optional<float> corner_radius = std::nullopt);
  void Cleanup();

  // Returns the material used to render the Android View.
  // Returns nullptr if the material is not yet loaded.
  BorrowedMaterialPtr GetMaterial() const;

  // Updates the collider of the Android View.
  void UpdateCollider(const imp::Box& collider);

  // Dispatch motion_event as generic motion events to the Android View.
  void DispatchGenericMotionEventToView(MotionEvent& motion_event);
  // Dispatch motion_event as touch events to the Android View.
  void DispatchTouchEventToView(MotionEvent& motion_event);

  // Converts a 3d point on the surface in world space to a 2d point in surface
  // coordinates.
  float2 GetSurfaceCoordinatesFromWorldPoint(float3 world_hit_point);

  // Returns the jobject Android View associated with this component.
  jobject GetAndroidView();

 private:
  // A variant of the default unlit material or a custom material.
  using TextureMaterialVariant =
      std::variant<std::unique_ptr<android_xr::TextureExternalMaterial>,
                   OwnedMaterialPtr>;

  void ForwardHoverInputs(const PointerHitEvent& event);
  void ForwardTouchInputs(const PointerHitEvent& event);
  void ForwardSplitEngineInputs(const android_xr::SplitEngineInputEvent& event);
  Future<TextureMaterialVariant> LoadMaterial(
      absl::optional<imp::MaterialDefinition> material_definition);
  void ForwardControllerInputs(const ControllerHitEvent& event);

  NodeHandle renderer_node_;
  jobject android_view_ = nullptr;
  std::unique_ptr<AndroidExternalTextureSurface> surface_;
  bool was_hovering_ = false;
  absl::flat_hash_map<ControllerHitEvent::Hand, bool> controller_was_hovering_;
  std::unique_ptr<RenderViewToSurfaceTextureWrapper> renderer_wrapper_;
  TextureMaterialVariant material_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_RENDERER_H_
