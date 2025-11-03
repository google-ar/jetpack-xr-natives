// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/scripting/message_handlers/android/android_view_renderer.h"

#include <jni.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <variant>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_events.h"
#include "core/async/future.h"
#include "core/common/jni_helpers.h"
#include "core/geometry/shapes/box.h"
#include "core/input/pointer_event.h"
#include "core/input/pointer_event_processor.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
#include "core/scripting/message_handlers/android/android_view_renderer.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/primitive_shape_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/view/platforms/android/wrappers/motion_event.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/materials/texture_external_material.h"

namespace imp::android {

namespace {

static constexpr absl::string_view kBaseColorMaterialParam = "baseColor";
static constexpr imp::Box kBaseBoxBounds = {imp::float3(0, 0, 0),
                                            imp::float3(0.5, 0.5, 0.001)};

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_scripting_viewtexture_RenderViewToSurfaceTexture_##method_name  // NOLINT

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, RenderViewToSurfaceTextureWrapper>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, RenderViewToSurfaceTextureWrapper>::FromJava(n);
}

}  // namespace

// LINT.IfChange(android_view_renderer)

extern "C" {

JNI_METHOD(void, nSetRenderViewSurfaceDimensions)
(JNIEnv* env, jclass /*clazz*/, jlong renderer_handle, jint width,
 jint height) {
  int2 size(width, height);
  imp::android::RenderViewToSurfaceTextureWrapper* renderer =
      FromJava<imp::android::RenderViewToSurfaceTextureWrapper>(
          renderer_handle);
  if (!renderer) {
    env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                  "Invalid renderer handle.");
    return;
  }
  renderer->SetSize(size);
}

}  // extern "C"

RenderViewToSurfaceTextureWrapper::RenderViewToSurfaceTextureWrapper(
    NodeHandle node, jobject android_view, ViewSize view_size,
    AndroidExternalTextureSurface& surface)
    : JavaWrapper(node->GetView().GetContext().GetJniEnv(),
                  "com/google/ar/imp/core/scripting/viewtexture/"
                  "RenderViewToSurfaceTexture",
                  "(Landroid/content/Context;Landroid/view/View;Landroid/view/"
                  "Surface;IIJ)V",
                  node->GetView().GetContext().GetActivityContext(),
                  android_view, surface.GetSurface()->Reference(),
                  view_size.width, view_size.height, ToJava(this)),
      node_(node),
      surface_(surface),
      view_size_(view_size) {
  CallVoidMethod(GetMethodHandle("initialize", "()V"));
  release_ = GetMethodHandle("release", "()V");
  dispatch_generic_motion_event_ = GetMethodHandle(
      "dispatchGenericMotionEventToView", "(Landroid/view/MotionEvent;)V");
  dispatch_touch_event_ = GetMethodHandle("dispatchTouchEventToView",
                                          "(Landroid/view/MotionEvent;)V");
}

RenderViewToSurfaceTextureWrapper::~RenderViewToSurfaceTextureWrapper() {
  CallVoidMethod(release_);
}

BorrowedTexturePtr RenderViewToSurfaceTextureWrapper::BorrowTexture() {
  return surface_.BorrowTexture();
}

int2 RenderViewToSurfaceTextureWrapper::GetSize() const {
  return {view_size_.width, view_size_.height};
}

void RenderViewToSurfaceTextureWrapper::SetSize(int2 size) {
  view_size_.width = size.x;
  view_size_.height = size.y;

  if (absl::Status status = surface_.SetDefaultBufferSize(size); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to set default buffer size: " << status.ToString();
  }

  // Set the scale to correct for aspect ratio.
  float aspect = static_cast<float>(size.x) / static_cast<float>(size.y);
  float3 scale;
  if (view_size_.meters_per_pixel.has_value()) {
    float meters_per_pixel = *view_size_.meters_per_pixel;
    scale = {size.x * meters_per_pixel, size.y * meters_per_pixel, 1};
  } else if (aspect > 1) {
    scale = {1, 1.f / aspect, 1};
  } else {
    scale = {aspect, 1, 1};
  }
  node_->SetLocalScale(scale);
}

void RenderViewToSurfaceTextureWrapper::DispatchGenericMotionEvent(
    MotionEvent& motion_event) {
  CallVoidMethod(dispatch_generic_motion_event_, motion_event.WeakReference());
}

void RenderViewToSurfaceTextureWrapper::DispatchTouchEvent(
    MotionEvent& motion_event) {
  CallVoidMethod(dispatch_touch_event_, motion_event.WeakReference());
}

Future<absl::Status> AndroidViewRenderer::Setup(
    jobject android_view, ViewSize view_size,
    InputForwardingMode input_forwarding_mode,
    absl::optional<imp::MaterialDefinition> material_definition,
    absl::optional<uint32_t> blend_priority) {
  // Adding a child node so that generated Components are not attached to the
  // root node and so that we can set the scale of the renderer node to reflect
  // the aspect ratio of the Android View.
  renderer_node_ = GetView().CreateNode();
  renderer_node_->SetParent(GetNode());

  android_view_ =
      GetView().GetContext().GetJniEnv()->NewGlobalRef(android_view);

  return AndroidExternalTextureSurface::CreateAsync(GetView()).Then(
      [this, view_size, input_forwarding_mode, material_definition,
       blend_priority](std::unique_ptr<AndroidExternalTextureSurface> surface) {
        surface_ = std::move(surface);
        renderer_wrapper_ = std::make_unique<RenderViewToSurfaceTextureWrapper>(
            renderer_node_, android_view_, view_size, *surface_);

        // Add a collider so we get hit events.
        renderer_node_->AddComponent<BoxCollider>(kBaseBoxBounds);

        if (input_forwarding_mode ==
            InputForwardingMode::INPUT_FORWARDING_MODE_DEFAULT) {
          // TODO: GetNode()->Connect(...) and return kAccept.
          GetView().GetDispatcher().Connect(
              [this](const PointerHitEvent& event) {
                if (event.GetPointerCount() == 0) {
                  return;
                }
                if (event.event.Type() == PointerEventType::kHover) {
                  ForwardHoverInputs(event);
                } else {
                  ForwardTouchInputs(event);
                }
              },
              this);

          GetView().GetDispatcher().Connect(
              [this](const ControllerHitEvent& event) {
                ForwardControllerInputs(event);
              },
              this);

          GetView().GetDispatcher().Connect(
              [this](const android_xr::SplitEngineInputEvent& event) {
                bool is_mouse =
                    event.device_type ==
                    android_xr::SplitEngineInputEvent::DeviceType::MOUSE;
                bool is_left_pointer =
                    event.pointer_type ==
                    android_xr::SplitEngineInputEvent::PointerType::LEFT;
                bool is_right_pointer =
                    event.pointer_type ==
                    android_xr::SplitEngineInputEvent::PointerType::RIGHT;

                if (!is_mouse && !is_left_pointer && !is_right_pointer) {
                  return;
                }
                ForwardSplitEngineInputs(event);
              },
              this);
        }

        if (!GetView().AreSplitEngineMaterialsInLocalMode() &&
            material_definition) {
          return Future<absl::Status>(absl::InvalidArgumentError(
              "Custom materials are not supported in non local mode."));
        }

        return LoadMaterial(material_definition)
            .Then([this](TextureMaterialVariant material) {
              if (std::holds_alternative<OwnedMaterialPtr>(material)) {
                // Custom material
                std::get<OwnedMaterialPtr>(material)->SetParameter(
                    kBaseColorMaterialParam,
                    renderer_wrapper_->BorrowTexture());
              } else {
                // Default material
                std::get<std::unique_ptr<android_xr::TextureExternalMaterial>>(
                    material)
                    ->SetTexture(renderer_wrapper_->BorrowTexture());
              }
              material_ = std::move(material);
            })
            .Then([this, blend_priority]() {
              PrimitiveShapeRendererState primitive_shape_state;
              primitive_shape_state.primitive = {
                  .mesh = PrimitiveShapeRendererState::QuadMesh{
                      .size = kOne2, .flip_uv = true}};
              primitive_shape_state.priority = blend_priority;
              return renderer_node_
                  ->AddComponentWithState<PrimitiveShapeRenderer>(
                      primitive_shape_state);
            })
            .Then([this](ComponentHandle<PrimitiveShapeRenderer> quad_renderer)
                      -> Future<absl::Status> {
              if (std::holds_alternative<OwnedMaterialPtr>(material_)) {
                return quad_renderer->SetMaterial(
                    std::get<OwnedMaterialPtr>(material_).Borrow());
              } else {
                return quad_renderer->SetMaterial(
                    std::get<
                        std::unique_ptr<android_xr::TextureExternalMaterial>>(
                        material_)
                        ->GetMaterial());
              }
            });
      });
}

void AndroidViewRenderer::UpdateCollider(const imp::Box& collider) {
  renderer_node_->GetComponent<BoxCollider>()->SetBox(collider);
}

BorrowedMaterialPtr AndroidViewRenderer::GetMaterial() const {
  return std::holds_alternative<OwnedMaterialPtr>(material_)
             ? std::get<OwnedMaterialPtr>(material_).Borrow()
             : std::get<std::unique_ptr<android_xr::TextureExternalMaterial>>(
                   material_)
                   ->GetMaterial();
}

Future<AndroidViewRenderer::TextureMaterialVariant>
AndroidViewRenderer::LoadMaterial(
    absl::optional<imp::MaterialDefinition> material_definition) {
  if (material_definition) {
    return GetView()
        .GetMaterialFactory()
        .LoadMaterial(*material_definition)
        .Then([](OwnedMaterialPtr material) {
          return TextureMaterialVariant(std::move(material));
        });
  } else {
    return android_xr::TextureExternalMaterial::Create(GetView()).Then(
        [](std::unique_ptr<android_xr::TextureExternalMaterial> material) {
          return TextureMaterialVariant(std::move(material));
        });
  }
}

void AndroidViewRenderer::Cleanup() {
  // Remove the PrimitiveShapeRenderer before the AndroidViewRenderer is
  // destroyed, so that the borrowed material can be released first.
  GetNode()->RemoveComponent<PrimitiveShapeRenderer>();
  DeleteRef(GetView().GetContext().GetJniEnv(), android_view_);
}

void AndroidViewRenderer::DispatchGenericMotionEventToView(
    MotionEvent& motion_event) {
  // TODO: Handle multitouch / cases where the input starts
  // inside and then leaves the view.
  renderer_wrapper_->DispatchGenericMotionEvent(motion_event);
}

void AndroidViewRenderer::DispatchTouchEventToView(MotionEvent& motion_event) {
  // TODO: Handle multitouch / cases where the input starts
  // inside and then leaves the view.
  renderer_wrapper_->DispatchTouchEvent(motion_event);
}

float2 AndroidViewRenderer::GetSurfaceCoordinatesFromWorldPoint(
    float3 world_hit_point) {
  float3 local_hit = renderer_node_->LocalFromWorldPoint(world_hit_point);
  // Convert from quad-space [-0.5, 0.5] to normalized [0, 1] space and
  // flip y because java views go from top-left to bottom-right.
  float2 normalized_hit = {local_hit.x + 0.5f, 1 - (local_hit.y + 0.5f)};
  int2 surface_size = renderer_wrapper_->GetSize();
  return {normalized_hit.x * surface_size.x, normalized_hit.y * surface_size.y};
}

jobject AndroidViewRenderer::GetAndroidView() { return android_view_; }

void AndroidViewRenderer::ForwardHoverInputs(const PointerHitEvent& event) {
  // Only consider this event if we are the front-most node hit.
  std::optional<RayHit> hit = event.GetTruncatedRayHit();
  if (!hit.has_value() || hit->node != renderer_node_) {
    if (was_hovering_) {
      was_hovering_ = false;
      MotionEvent motion_event(GetView().GetContext().GetJniEnv(), {0, 0},
                               MotionEvent::Action::kHoverExit);
      DispatchGenericMotionEventToView(motion_event);
    }
  } else {
    MotionEvent::Action action = MotionEvent::Action::kHoverMove;
    if (!was_hovering_) {
      was_hovering_ = true;
      action = MotionEvent::Action::kHoverEnter;
    }
    float2 surface_coordinates =
        GetSurfaceCoordinatesFromWorldPoint(hit->world_point);
    MotionEvent motion_event(GetView().GetContext().GetJniEnv(),
                             surface_coordinates, action);
    DispatchGenericMotionEventToView(motion_event);
  }
}

void AndroidViewRenderer::ForwardTouchInputs(const PointerHitEvent& event) {
  // Only consider this event if we are the front-most node hit.
  std::optional<RayHit> hit = event.GetTruncatedRayHit();
  if (!hit.has_value() || hit->node != renderer_node_) {
    return;
  }
  float2 surface_coordinates =
      GetSurfaceCoordinatesFromWorldPoint(hit->world_point);
  MotionEvent motion_event(GetView().GetContext().GetJniEnv(),
                           surface_coordinates, event.event.Type());
  DispatchTouchEventToView(motion_event);
}

// LINT.ThenChange(
//
// //depot/google3/third_party/impress/java/com/google/ar/imp/core/scripting/viewtexture/RenderViewToSurfaceTexture.java
// )

void AndroidViewRenderer::ForwardSplitEngineInputs(
    const android_xr::SplitEngineInputEvent& event) {
  if (!event.hit_node || !(event.hit_node->target == renderer_node_) ||
      !event.hit_node->world_hit_position) {
    return;
  }
  float2 surface_coordinates =
      GetSurfaceCoordinatesFromWorldPoint(*event.hit_node->world_hit_position);
  MotionEvent::Action action;
  bool is_touch = false;
  switch (event.action) {
    case android_xr::SplitEngineInputEvent::Action::ACTION_DOWN:
      action = MotionEvent::Action::kDown;
      is_touch = true;
      break;
    case android_xr::SplitEngineInputEvent::Action::ACTION_MOVE:
      action = MotionEvent::Action::kMove;
      is_touch = true;
      break;
    case android_xr::SplitEngineInputEvent::Action::ACTION_UP:
      action = MotionEvent::Action::kUp;
      is_touch = true;
      break;
    case android_xr::SplitEngineInputEvent::Action::ACTION_CANCEL:
      // Action cancel is not supported by Android MotionEvent.
      return;
    case android_xr::SplitEngineInputEvent::Action::ACTION_HOVER_MOVE:
      action = MotionEvent::Action::kHoverMove;
      break;
    case android_xr::SplitEngineInputEvent::Action::ACTION_HOVER_ENTER:
      action = MotionEvent::Action::kHoverEnter;
      break;
    case android_xr::SplitEngineInputEvent::Action::ACTION_HOVER_EXIT:
      action = MotionEvent::Action::kHoverExit;
      break;
    default:
      return;
  }

  MotionEvent motion_event(GetView().GetContext().GetJniEnv(),
                           surface_coordinates, action);
  if (is_touch) {
    DispatchTouchEventToView(motion_event);
  } else {
    DispatchGenericMotionEventToView(motion_event);
  }
}

void AndroidViewRenderer::ForwardControllerInputs(
    const ControllerHitEvent& event) {
  // Make sure the controller hit event has a kDefaultSelectActionName action.
  if (!event.HasInputAction(kDefaultSelectActionName)) {
    return;
  }

  // Get previous hover state for this hand.
  bool& was_hovering = controller_was_hovering_[event.GetHand()];

  if (event.GetHitNode() != renderer_node_) {
    // If the hit node is not the renderer node and we were previously hovering,
    // send a hover exit event.
    if (was_hovering) {
      was_hovering = false;
      MotionEvent motion_event(GetView().GetContext().GetJniEnv(), {0, 0},
                               MotionEvent::Action::kHoverExit);
      DispatchGenericMotionEventToView(motion_event);
    }
    // Always return if the hit node is not the renderer node.
    return;
  }

  // Check for hover enter event.
  if (!was_hovering) {
    was_hovering = true;
    MotionEvent motion_event(GetView().GetContext().GetJniEnv(), {0, 0},
                             MotionEvent::Action::kHoverEnter);
    DispatchGenericMotionEventToView(motion_event);
    return;
  }

  float2 surface_coordinates =
      GetSurfaceCoordinatesFromWorldPoint(event.GetHit()->world_point);
  auto select_button_state =
      event.GetInputActionStateVariant(kDefaultSelectActionName);

  // Using this visitor to allow "select" to be defined as either a bool action
  // (ie for a button) or a float action (ie for a trigger).
  bool is_select = std::visit(
      [](auto&& state) {
        if constexpr (std::is_same_v<decltype(state.current_state), bool>) {
          return state.current_state;
        } else if constexpr (std::is_same_v<decltype(state.current_state),
                                            float>) {
          return state.current_state >= .95f;
        } else {
          return false;
        }
      },
      select_button_state);

  bool select_changed =
      std::visit([](auto&& state) { return state.has_changed_since_last_sync; },
                 select_button_state);

  // If we are tap down, tap up, or moving, send a touch event.
  if (is_select || select_changed) {
    MotionEvent::Action action;
    if (select_changed) {
      action =
          is_select ? MotionEvent::Action::kDown : MotionEvent::Action::kUp;
    } else {
      action = MotionEvent::Action::kMove;
    }
    MotionEvent motion_event(GetView().GetContext().GetJniEnv(),
                             surface_coordinates, action);
    DispatchTouchEventToView(motion_event);
  } else {
    // Otherwise send a generic motion event with kHoverMove action.
    MotionEvent motion_event(GetView().GetContext().GetJniEnv(),
                             surface_coordinates,
                             MotionEvent::Action::kHoverMove);
    DispatchGenericMotionEventToView(motion_event);
  }
}

}  // namespace imp::android
