// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_DESKTOP_HOOKS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_DESKTOP_HOOKS_H_

#include <functional>

#include "core/async/executor.h"
#include "core/view/base_view.h"

namespace imp::desktop_api {

// TODO: (broken link) - consider moving this to view_hooks or client_api.
//
// # Overview
//
// Today, desktop_split_engine apps are based on desktop_main.cc (which is used
// by pure desktop apps too) and desktop_renderer_hook.cc.
//
// `desktop_renderer_hook.cc` calls `SetPostHostSetupFnand`, and
// `desktop_main.cc` will call that PostHostSetup at the right time.
//
// So, desktop_hooks.* can be seen as hooks into desktop_main.cc.
//
// # Food for thought
//
// Can we scale this "PostHostSetup" to be cross-platform hook (e.g. to work on
// Android)?
//
// # Alternatives
// ## New Dispatcher Events
//
// If we'll switch to Dispatcher Events, desktop_main.cc will
// have to subscribe to event and call something _external_ and we'll end up
// having some form of desktop_hooks.*.
//
// ## New ViewHook or integration into existing ViewHook
//
// Same problem as above: we'll have to find a way to let PostHostSetup hook to
// call external code.
//
// # Summary
//
// In order to scale it to Android, Android's version of desktop_main.cc
// (split_engine_jni.cc?) will have to e.g. subscribe to the event and call
// something external too. Since we are not sharing main file between platforms
// for obvious reasons, does it make sense to find cross-platform way to hook
// into PostHostSetup?
//
using PostHostSetupFn = std::function<void(BaseView&, Executor&)>;

bool SetPostHostSetupFn(PostHostSetupFn post_host_setup_fn);

void PostHostSetup(BaseView& view, Executor& executor);

}  // namespace imp::desktop_api

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_DESKTOP_DESKTOP_HOOKS_H_
