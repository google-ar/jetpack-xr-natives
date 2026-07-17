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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SHARED_SPLIT_ENGINE_DEFINES_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SHARED_SPLIT_ENGINE_DEFINES_H_

#include <cstdint>

#include "core/common/enum_flags.h"

namespace imp::split_engine {
// Identifies an unset user_id property.
constexpr uint32_t kUndefinedUserId = 0;

// An entity ID set to this represents an invalid Node, ie. NodeHandle().
constexpr uint32_t kInvalidEntityId = 0;

// Uniquely identifies a SplitEngineBridge on the client side. This is used in
// the SplitEngineBridgeSender message group tracking and release callbacks to
// ensure that the correct SplitEngineBridge is used for the callback.
using ClientId = uint64_t;

// Uniquely identifies the connection of a Split Engine application on the
// system (SplitEngineRenderer) side. This is used to track app content context
// and cleanup on the system side.
using BridgeId = uint64_t;

// Uniquely identifies a shared buffer in a Split Engine application.
using BufferId = uint64_t;

// Uniquely identifies a texture in a Split Engine application.
using TextureId = uint64_t;

// Uniquely identifies a message group in a Split Engine application.
using MessageGroupId = uint64_t;

// Flags defining the different permissions for apps.
enum class AppPermissionTypes : uint8_t {
  // The app has unrestricted access to the system.
  // The app is allowed to render 3D content, without any
  // system-side restrictions.
  kHasUnrestrictedSystemAccess = (1 << 0),
  // The app is allowed to control transforms on nodes with user IDs.
  kAllowCustomTransformsOnNodesWithUserIds = (1 << 1),
  // The app is allowed to use unstable_api attributes in Flatbuffers schemas.
  kAllowUnstableApiAttributes = (1 << 2),
};
// enum_flag type that encapsulates the application permissions.
using AppPermission = imp::Flags<AppPermissionTypes>;

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SHARED_SPLIT_ENGINE_DEFINES_H_
