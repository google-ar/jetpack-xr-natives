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

#ifndef THIRD_PARTY_IMPRESS_IMP_H_
#define THIRD_PARTY_IMPRESS_IMP_H_
// TODO(b/350956276): This is inserted because Soong makes it impossible to
// enable this warning in blueprint files.
#pragma clang diagnostic error "-Wunused-result"


// imp.h is a forwarding convenience library that aims to reduce the need to
// manage dependencies for the user as well as include the minimum files
// required to utilize the impress library. This forwarding header also makes it
// easier for developers of impress to make changes under the hood without fear
// or need to fix and update applications relying on impress.
// (broken link) tracks the ongoing discussions related to the state of the
// forwarding library as well as recommendations on how to add dependencies on
// impress.

// IWYU pragma: begin_exports
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_status_utils.h"
#include "core/common/context.h"
#include "core/common/filament_helpers.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/common/rememberer.h"
#include "core/common/resource_helpers.h"
#include "core/common/smooth.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/lighting/environment_light_factory.h"
#include "core/materials/material.h"
#include "core/math/aabb_helpers.h"
#include "core/math/almost_equal.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/vertex_format.h"
#include "core/model/model_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/groups_manager.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/node_handle_message.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/system.h"
#include "core/ncsb/update_system.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/render/texture_registry.h"
#include "core/resources/resource_definition.h"
#include "core/scene_handles/scene_handle_status_utils.h"
#include "core/scene_handles/scene_handles.h"
#include "core/scene_handles/scene_handles.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_asset_loader.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/assets/material_asset.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/assets/proto_asset.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/client_api.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/framework/display_layer/display_layer_manager.h"
#include "core/view/framework/gestures/double_tap_gesture.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_manager.h"
#include "core/view/framework/gestures/multi_drag_gesture.h"
#include "core/view/framework/gestures/pinch_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/framework/gestures/twist_gesture.h"
#if IMP_PLATFORM(DESKTOP)
#include "core/view/framework/input/desktop_input_handler.h"
#endif
#include "filament/libs/utils/include/utils/Systrace.h"
#include "core/model/mesh/mesh.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/mesh_factory.h"
// TODO: When render_component is safe to deprecate, change this.
#include "core/view/framework/render/render_component.h"
#include "core/view/framework/scene/load_scene_visitor.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/device.h"
#include "core/view/utils/frame_time.h"
#include "core/view/view_events.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"
// IWYU pragma: end_exports

#endif  // THIRD_PARTY_IMPRESS_IMP_H_
