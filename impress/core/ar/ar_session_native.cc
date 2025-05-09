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

#include "core/ar/ar_session_native.h"

namespace imp {
namespace ar {

CanCreateArSessionNativeFn&
ArSessionNative::GetCanCreateArSessionNativeOverrideFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static CanCreateArSessionNativeFn* stored_can_create_fn =
      new CanCreateArSessionNativeFn();
  return *stored_can_create_fn;
}

CreateArSessionNativeFn& ArSessionNative::GetCreateArSessionNativeOverrideFn() {
  // It's best to store static data structures as a static pointer inside of
  // a function, per (broken link) and (broken link).
  static CreateArSessionNativeFn* stored_create_fn =
      new CreateArSessionNativeFn();
  return *stored_create_fn;
}

void ArSessionNative::SetCreateArSessionNativeOverride(
    CanCreateArSessionNativeFn can_create_session_fn,
    CreateArSessionNativeFn create_session_fn) {
  CanCreateArSessionNativeFn& can_create_override_fn =
      GetCanCreateArSessionNativeOverrideFn();
  can_create_override_fn = can_create_session_fn;
  CreateArSessionNativeFn& create_override_fn =
      GetCreateArSessionNativeOverrideFn();
  create_override_fn = create_session_fn;
}

}  // namespace ar
}  // namespace imp
