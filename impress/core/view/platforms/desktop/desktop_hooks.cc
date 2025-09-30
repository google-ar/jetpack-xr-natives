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

#include "core/view/platforms/desktop/desktop_hooks.h"

#include <cassert>

#include "core/common/log.h"
#include "core/async/executor.h"
#include "core/view/base_view.h"

namespace imp::desktop_api {

PostHostSetupFn& GetPostHostSetupFn() {
  static PostHostSetupFn* post_host_setup_fn = new PostHostSetupFn();
  return *post_host_setup_fn;
}

bool SetPostHostSetupFn(PostHostSetupFn post_host_setup_fn) {
  assert(post_host_setup_fn);

  PostHostSetupFn& stored_post_host_setup_fn = GetPostHostSetupFn();
  if (stored_post_host_setup_fn) {
    IMP_LOG(imp::FATAL) << "Cannot call client_api::SetPostHostSetupFn more than once.";
    return false;
  }

  stored_post_host_setup_fn = post_host_setup_fn;
  return true;
}

void PostHostSetup(BaseView& view, Executor& executor) {
  PostHostSetupFn& post_host_setup_fn = GetPostHostSetupFn();
  if (post_host_setup_fn) {
    post_host_setup_fn(view, executor);
  }
}

}  // namespace imp::desktop_api
