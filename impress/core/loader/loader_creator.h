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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_SANDBOXED_CREATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_SANDBOXED_CREATOR_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/buffer_access.h"
#include "core/common/invocable.h"
#include "core/loader/loader.h"
#include "core/loader/loader_options.h"
#include "core/material_library/material_package.h"
#include "core/view/base_view.h"

namespace imp::loader {

using GetLoaderFn = imp::Invocable<absl::StatusOr<std::unique_ptr<Loader>>(
    BaseView& view, absl::string_view, BufferAccess, MaterialPackage*,
    LoaderOptions)>;

// An interface for creating an asset loader. This allows some
// projects to avoid the binary size increase of sandboxing the loader with the
// caveat that you must then trust the sources of all your assets.
struct LoaderCreator {
  virtual ~LoaderCreator() {}

  // Returns a Future to a function that returns the loader.
  // This allows background setup to happen in parallel with other tasks.
  virtual Future<GetLoaderFn> Create(
      BaseView& view,
      std::optional<FutureGroup> future_group = std::nullopt) = 0;
};

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_SANDBOXED_CREATOR_H_
