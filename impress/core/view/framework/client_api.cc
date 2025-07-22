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

#include "core/view/framework/client_api.h"

#include <cassert>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
#include "core/common/trace.h"
#include "core/view/utils/string_map.h"

namespace imp {
namespace client_api {

namespace {

CreateViewFn& GetStoredCreateViewFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static CreateViewFn* stored_create_view_fn = new CreateViewFn();
  return *stored_create_view_fn;
}

StringMap<CreateViewForIdentifierFn>& GetStoredCreateViewForIdentifierFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static StringMap<CreateViewForIdentifierFn>*
      stored_create_view_for_identifier_fns =
          new StringMap<CreateViewForIdentifierFn>();
  return *stored_create_view_for_identifier_fns;
}

}  // namespace

bool SetCreateViewFn(CreateViewFn create_view_fn) {
  assert(create_view_fn);

  IMP_TRACE_INIT();

  CreateViewFn& stored_create_view_fn = GetStoredCreateViewFn();
  if (stored_create_view_fn) {
    IMP_LOG(imp::FATAL) << "Cannot call client_api::SetCreateViewFn more than once.";
    return false;
  }

  stored_create_view_fn = create_view_fn;
  return true;
}

bool SetCreateViewFn(absl::string_view identifier,
                     CreateViewForIdentifierFn create_view_for_identifier_fn) {
  assert(create_view_for_identifier_fn);

  IMP_TRACE_INIT();

  StringMap<CreateViewForIdentifierFn>& stored_create_view_for_identifier_fns =
      GetStoredCreateViewForIdentifierFn();
  auto try_emplace_result = stored_create_view_for_identifier_fns.try_emplace(
      std::string(identifier), std::move(create_view_for_identifier_fn));
  if (!try_emplace_result.second) {
    IMP_LOG(imp::FATAL) << "Cannot call client_api::SetCreateViewFn with identifier "
               << identifier << " more than once.";
    return false;
  }

  return true;
}

std::unique_ptr<View> CreateView(absl::string_view identifier) {
  // Check if a CreateViewFn was registered for this specific identifier.
  // If so, call it.
  if (!identifier.empty()) {
    StringMap<CreateViewForIdentifierFn>&
        stored_create_view_for_identifier_fns =
            GetStoredCreateViewForIdentifierFn();
    auto itr = stored_create_view_for_identifier_fns.find(identifier);
    if (itr != stored_create_view_for_identifier_fns.end()) {
      return itr.value()();
    }
  }

  // Get the app global CreateViewFn and call it since there is none for this
  // specific identifier.
  CreateViewFn& stored_create_view_fn = GetStoredCreateViewFn();
  if (!stored_create_view_fn) {
    IMP_LOG(imp::FATAL) << "Cannot create " << identifier
               << " View. client_api::SetCreateViewFn not called.";
  }

  return stored_create_view_fn(identifier);
}

bool& GetIsAppSandboxTargetInternal() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static bool* is_app_sandbox_target = new bool(false);
  return *is_app_sandbox_target;
}

bool GetIsAppSandboxTarget() { return GetIsAppSandboxTargetInternal(); }

bool MakeAppSandboxTarget() {
  bool& is_app_sandbox_target = GetIsAppSandboxTargetInternal();
  is_app_sandbox_target = true;
  return true;
}

}  // namespace client_api
}  // namespace imp
