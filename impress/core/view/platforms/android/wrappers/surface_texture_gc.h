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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_GC_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_GC_H_

#include <jni.h>

#include <list>

#include "filament/filament/include/filament/Engine.h"
#include "core/common/jni_helpers.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp {

// A garbage collector for SurfaceTextures that releases them asynchronously
// when their fences are ready. This prevents the destruction of the
// SurfaceTextures while they are still in use by the GPU.
class SurfaceTextureGarbageCollector {
 public:
  explicit SurfaceTextureGarbageCollector(BaseView& view);
  ~SurfaceTextureGarbageCollector();

  SurfaceTextureGarbageCollector(const SurfaceTextureGarbageCollector&) =
      default;
  SurfaceTextureGarbageCollector& operator=(
      const SurfaceTextureGarbageCollector&) = default;
  SurfaceTextureGarbageCollector(SurfaceTextureGarbageCollector&&) = default;
  SurfaceTextureGarbageCollector& operator=(SurfaceTextureGarbageCollector&&) =
      default;

  // Queues a SurfaceTexture for release. The SurfaceTexture will be released
  // asynchronously when its fence is ready.
  void QueueForRelease(JniUniquePtr<jobject> surface_texture_object);

 private:
  // A wrapper around the SurfaceTexture Java object that releases it when the
  // wrapper is destroyed.
  class SurfaceTextureHolder : public JavaWrapper {
   public:
    explicit SurfaceTextureHolder(const Context& context,
                                  JniUniquePtr<jobject> surface_texture_object);

    ~SurfaceTextureHolder() override;

   private:
    JniHandle release_;
  };

  struct PendingRelease {
    PendingRelease(BaseView& view,
                   JniUniquePtr<jobject> surface_texture_object);
    ~PendingRelease();

    PendingRelease(const PendingRelease&) = delete;
    PendingRelease& operator=(const PendingRelease&) = delete;

    PendingRelease(PendingRelease&&) = default;
    PendingRelease& operator=(PendingRelease&&) = default;

    BaseView& view;
    SurfaceTextureHolder surface_texture_holder;
    filament::Fence* fence = nullptr;
  };

  void OnPostFrame(const ViewPostFrameEvent& event);

  BaseView& view_;
  std::list<PendingRelease> pending_releases_;
  Dispatcher::ScopedConnection connection_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_SURFACE_TEXTURE_GC_H_
