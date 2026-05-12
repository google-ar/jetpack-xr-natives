/*
 * Copyright 2025 Google LLC
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

#include "core/split_engine/materials/builtin/builtin_material_registry.h"

#include <array>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "core/common/small_source_location.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

using BuiltinMaterialCreator = BuiltinMaterialRegistry::BuiltinMaterialCreator;
using BuiltinMaterialCreators = std::array<
    std::optional<BuiltinMaterialRegistry::BuiltinMaterialCreator>,
    static_cast<size_t>(android_xr::schemas::BuiltInMaterialSpec::MAX) + 1>;
using BuiltInMaterialSpecValue =
    std::underlying_type_t<android_xr::schemas::BuiltInMaterialSpec>;

namespace {
inline BuiltInMaterialSpecValue GetBuiltInMaterialSpecValue(
    android_xr::schemas::BuiltInMaterialSpec type) {
  return static_cast<BuiltInMaterialSpecValue>(type);
}

BuiltinMaterialCreators& GetBuiltinMaterialCreators() {
  static absl::NoDestructor<BuiltinMaterialCreators> creators;
  return *creators;
}
}  // namespace

absl::StatusOr<BuiltinMaterialCreator&> BuiltinMaterialRegistry::Get(
    android_xr::schemas::BuiltInMaterialSpec type) {
  BuiltinMaterialCreators& creators = GetBuiltinMaterialCreators();
  if (type == android_xr::schemas::BuiltInMaterialSpec::NONE) {
    return absl::InvalidArgumentError("Cannot get creator for NONE spec type.");
  }
  BuiltInMaterialSpecValue type_value = GetBuiltInMaterialSpecValue(type);
  if (type_value > GetBuiltInMaterialSpecValue(
                       android_xr::schemas::BuiltInMaterialSpec::MAX)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid spec type: ", type));
  }
  if (!creators[type_value].has_value()) {
    return absl::NotFoundError(absl::StrCat(
        "Material not found for type: ",
        android_xr::schemas::EnumNameBuiltInMaterialSpec(type), " (",
        static_cast<
            std::underlying_type_t<android_xr::schemas::BuiltInMaterialSpec>>(
            type),
        ")"));
  }
  return creators[type_value].value();
}

bool BuiltinMaterialRegistry::RegisterOrDie(
    android_xr::schemas::BuiltInMaterialSpec type,
    BuiltinMaterialCreator creator, SmallSourceLocation loc) {
  if (type == android_xr::schemas::BuiltInMaterialSpec::NONE) {
    IMP_LOG(imp::FATAL) << "Cannot register creator for NONE spec type."
               << " registered at " << loc.GetFileName() << ":"
               << loc.GetLineNumber();
  }

  BuiltInMaterialSpecValue type_value = GetBuiltInMaterialSpecValue(type);
  if (type_value > GetBuiltInMaterialSpecValue(
                       android_xr::schemas::BuiltInMaterialSpec::MAX)) {
    IMP_LOG(imp::FATAL) << "Invalid spec type: " << type_value << " registered at "
               << loc.GetFileName() << ":" << loc.GetLineNumber();
  }

  BuiltinMaterialCreators& creators = GetBuiltinMaterialCreators();
  if (creators[type_value].has_value()) {
    IMP_LOG(imp::FATAL) << "Creator already registered for type: "
               << android_xr::schemas::EnumNameBuiltInMaterialSpec(type) << " ("
               << type_value << ") registered at " << loc.GetFileName() << ":"
               << loc.GetLineNumber();
  }

  IMP_LOG(imp::INFO) << "Registering built-in material creator for type: "
            << android_xr::schemas::EnumNameBuiltInMaterialSpec(type);

  creators[type_value] = std::move(creator);
  return true;
}
}  // namespace imp::split_engine
