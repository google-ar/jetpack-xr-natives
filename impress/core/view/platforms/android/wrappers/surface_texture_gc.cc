// Copyright 2026 Google LLC
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

#include "core/view/platforms/android/wrappers/surface_texture_gc.h"

#include <jni.h>

#include <cstdint>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Fence.h"
#include "core/async/executor.h"
#include "core/common/jni_helpers.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp {

SurfaceTextureGarbageCollector::SurfaceTextureGarbageCollector(BaseView& view)
    : view_(view) {
  connection_ = view.GetDispatcher().Connect(
      [this](const ViewPostFrameEvent& event) { OnPostFrame(event); });
}

SurfaceTextureGarbageCollector::~SurfaceTextureGarbageCollector() {
  // Cleanup any remaining fences synchronously to prevent leaks on shutdown
  constexpr int64_t fence_wait_timeout =
      absl::ToInt64Nanoseconds(absl::Milliseconds(500));

  for (PendingRelease& pending_release : pending_releases_) {
    if (pending_release.fence != nullptr) {
      pending_release.fence->wait(filament::Fence::Mode::FLUSH,
                                  fence_wait_timeout);
    }
  }
}

void SurfaceTextureGarbageCollector::QueueForRelease(
    JniUniquePtr<jobject> surface_texture_object) {
  
  IMP_LOG(imp::INFO) << "[SurfaceTextureGarbageCollector] Queuing SurfaceTexture for "
               "release";
  pending_releases_.emplace_back(view_, std::move(surface_texture_object));
}

void SurfaceTextureGarbageCollector::OnPostFrame(
    const ViewPostFrameEvent& event) {
  // Iterate through the linked list to create fences for any SurfaceTextures
  // that don't have a fence yet, and to check the status of the fences for
  // those that do.
  for (auto it = pending_releases_.begin(); it != pending_releases_.end();) {
    if (it->fence == nullptr) {
      it->fence = view_.GetSharedEngine()->createFence();
      ++it;
      continue;
    }

    filament::Fence::FenceStatus status =
        it->fence->wait(filament::Fence::Mode::FLUSH, 0);

    if (status == filament::Fence::FenceStatus::CONDITION_SATISFIED ||
        status == filament::Fence::FenceStatus::ERROR) {
      // Erasing from a list does not invalidate other iterators or shift
      // elements
      IMP_LOG(imp::INFO) << "[SurfaceTextureGarbageCollector] Releasing SurfaceTexture";
      it = pending_releases_.erase(it);
    } else {
      ++it;
    }
  }
}

SurfaceTextureGarbageCollector::SurfaceTextureHolder::SurfaceTextureHolder(
    const Context& context, JniUniquePtr<jobject> surface_texture_object)
    : JavaWrapper(context.GetJniEnv(), std::move(surface_texture_object),
                  "android/graphics/SurfaceTexture"),
      release_(GetMethodHandle("release", "()V")) {}

SurfaceTextureGarbageCollector::SurfaceTextureHolder::~SurfaceTextureHolder() {
  CallVoidMethod(release_);
}

SurfaceTextureGarbageCollector::PendingRelease::PendingRelease(
    BaseView& view, JniUniquePtr<jobject> surface_texture_object)
    : view(view),
      surface_texture_holder(view.GetContext(),
                             std::move(surface_texture_object)) {}

SurfaceTextureGarbageCollector::PendingRelease::~PendingRelease() {
  if (fence != nullptr) {
    view.GetSharedEngine()->destroy(fence);
  }
}

}  // namespace imp
