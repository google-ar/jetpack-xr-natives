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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_BASIC_API_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_BASIC_API_H_

#include <memory>

#include "core/config.h"
#include "core/scripting/message_handlers/add_or_update_component_handler.h"
#include "core/scripting/message_handlers/animate_node_handler.h"
#include "core/scripting/message_handlers/create_node_handler.h"
#include "core/scripting/message_handlers/destroy_node_handler.h"
#include "core/scripting/message_handlers/find_node_handler.h"
#include "core/scripting/message_handlers/forward_input_handler.h"
#include "core/scripting/message_handlers/get_animation_names_handler.h"
#include "core/scripting/message_handlers/get_camera_handler.h"
#include "core/scripting/message_handlers/get_node_active_handler.h"
#include "core/scripting/message_handlers/get_node_name_handler.h"
#if IMP_RUNTIME(DEV)
#include "core/scripting/message_handlers/load_file_handler.h"
#endif
#include "core/scripting/message_handlers/get_children_handler.h"
#include "core/scripting/message_handlers/load_model_handler.h"
#if IMP_PLATFORM(ANDROID)
#include "core/scripting/message_handlers/load_and_apply_environment_light_handler.h"
#endif
#include "core/scripting/message_handlers/play_animation_handler.h"
#include "core/scripting/message_handlers/set_camera_projection_handler.h"
#include "core/scripting/message_handlers/set_node_enabled_handler.h"
#include "core/scripting/message_handlers/set_node_name_handler.h"
#include "core/scripting/message_handlers/set_parent_handler.h"
#include "core/scripting/message_handlers/transform_handler.h"
#include "core/scripting/scripting_system.h"
#include "core/view/base_view.h"

namespace imp::scripting {

// Add all message handlers that are part of the main Impress scripting API.
static void AddBasicApiMessageHandlers(ScriptingSystem& scripting_system,
                                       BaseView& view) {
  scripting_system.AddHandler(
      std::make_unique<AddOrUpdateComponentHandler>(view));
  scripting_system.AddHandler(std::make_unique<AnimateNodeHandler>(view));
  scripting_system.AddHandler(std::make_unique<CreateNodeHandler>(view));
  scripting_system.AddHandler(std::make_unique<DestroyNodeHandler>(view));
  scripting_system.AddHandler(std::make_unique<FindNodeHandler>(view));
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS) || IMP_PLATFORM(DESKTOP)
  scripting_system.AddHandler(std::make_unique<ForwardInputHandler>(view));
#endif
  scripting_system.AddHandler(std::make_unique<GetAnimationNamesHandler>());
  scripting_system.AddHandler(std::make_unique<GetCameraHandler>(view));
  scripting_system.AddHandler(std::make_unique<LoadModelHandler>(view));
#if IMP_PLATFORM(ANDROID)
  scripting_system.AddHandler(
      std::make_unique<LoadModelFromInputStreamHandler>(view));
  scripting_system.AddHandler(
      std::make_unique<LoadAndApplyEnvironmentLightFromInputStreamHandler>(
          view));
#endif
#if IMP_RUNTIME(DEV)
  scripting_system.AddHandler(std::make_unique<LoadFileHandler>(view));
#endif
  scripting_system.AddHandler(std::make_unique<PlayAnimationHandler>(view));
  scripting_system.AddHandler(
      std::make_unique<SetCameraProjectionHandler>(view));
  scripting_system.AddHandler(std::make_unique<SetNodeEnabledHandler>());
  scripting_system.AddHandler(std::make_unique<GetNodeActiveHandler>());
  scripting_system.AddHandler(std::make_unique<GetChildrenHandler>());
  scripting_system.AddHandler(std::make_unique<SetParentHandler>());
  scripting_system.AddHandler(std::make_unique<TransformHandler>());
  scripting_system.AddHandler(std::make_unique<SetNodeNameHandler>());
  scripting_system.AddHandler(std::make_unique<GetNodeNameHandler>());
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_BASIC_API_H_
