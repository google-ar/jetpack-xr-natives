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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_SHARED_HOST_STATE_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_SHARED_HOST_STATE_H_

#include <functional>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/executor_helpers.h"
#include "core/async/thread_pool_executor.h"

#if IMP_PLATFORM(ANDROID)
#include <android/hardware_buffer.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp::window {

// Forward declared to avoid circular dependencies
class FilamentHost;

// Detail type for FilamentHost, for multiple views within the same process.
class SharedHostState {
 public:
  static SharedHostState& GetInstance();

  using SharedContextDeleter = std::function<void(void*)>;

  // Note: More than one Filament Host can acquire an engine, but they must each
  // be created with the same backend, platform and shared gl context.  Multiple
  // hosts wanting to use a specific shared context must all send the same
  // `shared_gl_context` pointer.  On Android, we internally create a shared
  // EGL context if the callers do not provide one.
  // Post-condition: If the StatusOr is ok() then the pointer is non-null.
  absl::StatusOr<filament::Engine*> GetOrCreateEngine(
      filament::Engine::Backend backend, filament::Engine::Platform* platform,
      void* shared_gl_context = nullptr, bool should_use_shared_context = true,
      const filament::Engine::Config& config = {},
      const filament::backend::FeatureLevel featureLevel =
          filament::backend::FeatureLevel::FEATURE_LEVEL_1,
      bool pause_rendering_thread = false,
      SharedContextDeleter shared_context_deleter = {},
      bool preinitialize_metal_platform = false);

  // Registers a new host.
  //
  // A matching call to UnregisterHostAndReturnIsLast() is required.
  void RegisterHost(FilamentHost* host);

  // Unregisters a previously-registered host.
  // This will detach and shutdown executors and delete the shared gl context.
  //
  // If host->OwnsFilament() is true, the filament engine will be destroyed if
  // this is the last host.
  // If host->OwnsFilament() is false, it only detaches the SharedHostState from
  // the engine pointer.
  bool UnregisterHostAndReturnIsLast(FilamentHost* host);
  Executor* GetForegroundExecutor();
  Executor* GetBackgroundExecutor();
  void* GetSharedGlContext() { return shared_gl_context_; }
  void RequestSynchronousShutdown();

  // Returns the OpenGL vendor string or an error.
  absl::StatusOr<std::string> GetVendorString();
  // Returns the OpenGL renderer string or an error.
  absl::StatusOr<std::string> GetRendererString();

#if IMP_PLATFORM(ANDROID)
  // Registers an external image handle with the Filament platform.
  filament::backend::Platform::ExternalImageHandle RegisterExternalImageHandle(
      const AHardwareBuffer* buffer, bool sRGB);
  struct ExternalImageMetadata {
    unsigned int width;                       // Texture width
    unsigned int height;                      // Texture height
    filament::backend::TextureFormat format;  // Texture format
    filament::backend::TextureUsage usage;    // Texture usage flags
  };
  // Returns the metadata for the given external image handle.
  ExternalImageMetadata GetImageMetadata(
      filament::backend::Platform::ExternalImageHandle externalImage);
#endif  // IMP_PLATFORM(ANDROID)

 private:
  SharedHostState() = default;

  filament::Engine* engine_ = nullptr;
  std::vector<FilamentHost*> hosts_ = {};

  filament::Engine::Backend backend_ = filament::Engine::Backend::DEFAULT;
  filament::Engine::Platform* platform_ = nullptr;
  void* shared_gl_context_ = nullptr;
  SharedContextDeleter deleter_ = {};
  bool using_external_context_ = false;
  bool use_async_shutdown_ = true;

  std::unique_ptr<ExecutorsHolder> executors_;
};

}  // namespace imp::window

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_SHARED_HOST_STATE_H_
