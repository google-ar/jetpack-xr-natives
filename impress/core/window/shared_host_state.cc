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

#include "core/window/shared_host_state.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/libs/utils/include/utils/Panic.h"
#include "core/async/executor.h"
#include "core/async/executor_helpers.h"
#include "core/async/simple_executor.h"
#include "core/async/thread_pool_executor.h"
#include "core/config.h"
#include "core/window/filament_host.h"

#if IMP_THREADS(GOOGLE3)
#include "thread/thread.h"
#endif  // IMP_THREADS(GOOGLE3)

#if IMP_PLATFORM(ANDROID)
#include <android/hardware_buffer.h>
#endif  // IMP_PLATFORM(ANDROID)

#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>

#include "filament/filament/backend/include/backend/platforms/OpenGLPlatform.h"
#include "filament/filament/backend/include/backend/platforms/PlatformEGLAndroid.h"
#endif  // IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)

#if IMP_MATERIAL_API(METAL)
#include "filament/filament/backend/include/backend/platforms/PlatformMetal.h"
#endif  // IMP_MATERIAL_API(METAL)

#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(VULKAN)
#include "filament/filament/backend/include/backend/platforms/VulkanPlatformAndroid.h"
#endif  // IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(VULKAN)

#if IMP_PLATFORM(WASM)
#include <emscripten/em_asm.h>
#endif  // IMP_PLATFORM(WASM)

namespace imp::window {

namespace {
#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
void* CreateDefaultEGLContext(EGLDisplay* out_display, EGLContext* out_context,
                              EGLSurface* out_surface);
void DestroyDefaultEGLContext(EGLDisplay egl_display, EGLContext egl_context,
                              EGLSurface egl_surface);
#endif  // IMP_MATERIAL_API(OPENGL) && IMP_PLATFORM(ANDROID)

void FilamentPanicHandler(void* user, utils::Panic const& panic) {
  // TODO Uncomment once utils::Panic::whatNonSensitive is
  // available in google3.
  //
  // Before fataling, log the non sensitive information from the filament panic
  // as an error. This allows the error to be included in crash reporting while
  // excluding any information that could contain PII (i.e. information from
  // non-literal strings).
  // IMP_LOG(imp::ERROR).AtLocation(panic.getFile(), panic.getLine())
  //    << "Filament panic occurred.Non sensitive information : "
  //    << absl::LogAsLiteral(panic.whatNonSensitive());

#if IMP_PLATFORM(WASM)
  EM_ASM(
      { throw new Error('Filament panic occurred: ' + UTF8ToString($0)); },
      panic.what());
#else
  IMP_LOG(imp::FATAL).AtLocation(panic.getFile(), panic.getLine())
      << "Filament panic occurred: " << panic.what();
#endif
}

}  // namespace

using ::filament::Engine;

SharedHostState& SharedHostState::GetInstance() {
  // State is shared across each FilamentHost that exists on the same thread by
  // using thread_local storage duration.
  thread_local SharedHostState instance;
  return instance;
}

absl::StatusOr<Engine*> SharedHostState::GetOrCreateEngine(
    Engine::Backend backend, Engine::Platform* platform,
    void* shared_gl_context, bool should_use_shared_context,
    const filament::Engine::Config& config,
    const filament::backend::FeatureLevel featureLevel,
    bool pause_rendering_thread, SharedContextDeleter shared_context_deleter,
    bool preinitialize_metal_platform) {
  // If an engine already exists then it is reused, but only if it is
  // initialized with the same backend, platform, and context.
  if (engine_ != nullptr) {
    void* expected_context =
        using_external_context_ ? shared_gl_context_ : nullptr;
    if (backend != backend_ || platform != platform_ ||
        shared_gl_context != expected_context) {
      return absl::InvalidArgumentError(
          "Requested a different engine type than the single shared instance");
    }
    return engine_;
  }

  // Otherwise create a new one.

  // If no external context is provided, then create one automatically if a
  // shared_context should be used.
  bool using_external_context = true;
  if (!shared_gl_context && should_use_shared_context) {
    if (shared_context_deleter) {
      return absl::InvalidArgumentError(
          "Cannot supply a deleter without a context");
    }
#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
    if (backend == filament::backend::Backend::OPENGL) {
      EGLDisplay egl_display;
      EGLContext egl_context;
      EGLSurface egl_surface;

      shared_gl_context =
          CreateDefaultEGLContext(&egl_display, &egl_context, &egl_surface);
      if (!shared_gl_context) {
        IMP_LOG(imp::ERROR) << "Failed to create default EGL context";
        return absl::InternalError("Failed to create default EGL context");
      }
      shared_context_deleter = [egl_display, egl_context, egl_surface](void*) {
        DestroyDefaultEGLContext(egl_display, egl_context, egl_surface);
      };
    }
#endif  // IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)

    using_external_context = false;
  }

  // Just before creating the filament engine for the first time, set a filament
  // panic handler. This panic handler re-directs filament panics to absl so
  // that the logs can be included in crash reporting tools.
  //
  // Note, once this is set, it's never unset. That's because the handler is a
  // static global but filament::Engines are per-thread. If Impress is used on
  // multiple threads, when the engine is destroyed on one thread we don't
  // know if it is still in-use on another, so we don't want to unset the
  // handler, which is just a function pointer to a free function with no state
  // anyways.
  //
  // It is possible this will get set multiple times, but since it's just a free
  // function with no state, that should be safe. If we need the callback to be
  // stateful, we might need filament to change the handler to be thread local.
  //
  // It is also possible for user code to override the handler.
  //
  // NOTE: This is done prior to creating the filament engine so that any panics
  // that occur during engine creation can be caught.
  utils::Panic::setPanicHandler(FilamentPanicHandler, nullptr);

  bool using_precreated_platform = false;
#if IMP_MATERIAL_API(METAL)
  if (preinitialize_metal_platform &&
      backend == filament::backend::Backend::METAL) {
    using_precreated_platform = platform == nullptr;
    filament::backend::PlatformMetal* platform_metal =
        platform == nullptr
            ? (new filament::backend::PlatformMetal())
            : static_cast<filament::backend::PlatformMetal*>(platform);

    if (!platform_metal->initialize()) {
      return absl::ResourceExhaustedError(
          "Failed to initialize Metal platform.");
    }

    platform = platform_metal;
  }
#endif

  engine_ = filament::Engine::Builder()
                .backend(backend)
                .platform(platform)
                .sharedContext(should_use_shared_context ? shared_gl_context
                                                         : nullptr)
                .config(&config)
                .featureLevel(featureLevel)
                .paused(pause_rendering_thread)
                .build();
  if (!engine_) {
    return absl::InternalError("Failed to create a filament engine");
  }

  backend_ = backend;
  platform_ = using_precreated_platform ? nullptr : platform;
  shared_gl_context_ = shared_gl_context;
  deleter_ = std::move(shared_context_deleter);
  using_external_context_ = using_external_context;

  return engine_;
}

void SharedHostState::RegisterHost(FilamentHost* host) {
  if (!host) {
    IMP_LOG(imp::FATAL) << "Missing required argument";
  }
  if (absl::c_any_of(hosts_, [host](FilamentHost* tracked_host) {
        return tracked_host == host;
      })) {
    IMP_LOG(imp::FATAL) << "Double-registration of filament host.";
  }
  hosts_.push_back(host);

  if (!executors_) {
    absl::StatusOr<std::unique_ptr<ExecutorsHolder>> executors_or =
        TryCreateAndSetExecutors([]() {
          std::unique_ptr<Executor> foreground_executor =
              std::make_unique<SimpleForegroundExecutor>();

#if IMP_PLATFORM(WASM) && !defined(__EMSCRIPTEN_PTHREADS__)
          // Building for Wasm without threads enabled.
          // Use a simple executor that is explicitly pumped.
          std::unique_ptr<Executor> background_executor =
              std::make_unique<SimpleExecutor>();
#else
          std::unique_ptr<Executor> background_executor =
              std::make_unique<ThreadPoolExecutor>(foreground_executor.get());
#endif

          return ExecutorsHolder{
              .foreground_executor = std::move(foreground_executor),
              .background_executor = std::move(background_executor)};
        });

    if (executors_or.ok()) {
      executors_ = std::move(executors_or.value());
    }
  }
}

bool SharedHostState::UnregisterHostAndReturnIsLast(FilamentHost* host) {
  auto erase_it = std::remove_if(
      hosts_.begin(), hosts_.end(),
      [host](FilamentHost* tracked_host) { return tracked_host == host; });
  if (erase_it == hosts_.end()) {
    IMP_LOG(imp::FATAL) << "Unknown FilamentHost";
  }
  hosts_.erase(erase_it, hosts_.end());
  bool last_host = hosts_.empty();
  if (last_host) {
    if (executors_) {
      auto unused = DetachAndShutdownExecutors(std::move(executors_),
                                               use_async_shutdown_);
    }

    if (engine_ != nullptr && host->OwnsFilament()) {
      IMP_LOG(imp::INFO) << "Deleting shared filament Engine";
      filament::Engine::destroy(&engine_);
    }

    engine_ = nullptr;
    backend_ = Engine::Backend::DEFAULT;
    platform_ = nullptr;
    if (deleter_) {
      deleter_(shared_gl_context_);
    }
    shared_gl_context_ = nullptr;
  }
  return last_host;
}

Executor* SharedHostState::GetForegroundExecutor() {
  if (executors_) {
    return executors_->foreground_executor.get();
  }
  return nullptr;
}

Executor* SharedHostState::GetBackgroundExecutor() {
  if (executors_) {
    return executors_->background_executor.get();
  }
  return nullptr;
}

void SharedHostState::RequestSynchronousShutdown() {
  use_async_shutdown_ = false;
}

absl::StatusOr<std::string> SharedHostState::GetVendorString() {
#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
  filament::backend::OpenGLPlatform* opengl_platform =
      static_cast<filament::backend::OpenGLPlatform*>(platform_);
  if (!engine_ || !opengl_platform) {
    return absl::InternalError("Filament is not initialized");
  }
  auto driver = engine_->getDriver();
  if (!driver) {
    return absl::InternalError("Driver is null");
  }
  utils::CString vendor_string = opengl_platform->getVendorString(driver);
  if (vendor_string.empty()) {
    return absl::InternalError("Vendor string is empty");
  }
  return std::string(vendor_string.data(), vendor_string.size());
#else
  return absl::InternalError(
      "Vendor string is not supported for non-OpenGL and Android platforms");
#endif
}

absl::StatusOr<std::string> SharedHostState::GetRendererString() {
#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
  filament::backend::OpenGLPlatform* opengl_platform =
      static_cast<filament::backend::OpenGLPlatform*>(platform_);
  if (!engine_ || !opengl_platform) {
    return absl::InternalError("Filament is not initialized");
  }
  auto driver = engine_->getDriver();
  if (!driver) {
    return absl::InternalError("Driver is null");
  }
  utils::CString renderer_string = opengl_platform->getRendererString(driver);
  if (renderer_string.empty()) {
    return absl::InternalError("Renderer string is empty");
  }
  return std::string(renderer_string.data(), renderer_string.size());
#else
  return absl::InternalError(
      "Renderer string is not supported for non-OpenGL and Android platforms");
#endif
}

#if IMP_PLATFORM(ANDROID)
filament::backend::Platform::ExternalImageHandle
SharedHostState::RegisterExternalImageHandle(const AHardwareBuffer* buffer,
                                             bool sRGB) {
  filament::backend::Platform::ExternalImageHandle buffer_handle;
#if IMP_MATERIAL_API(OPENGL)
  buffer_handle = static_cast<filament::backend::PlatformEGLAndroid*>(platform_)
                      ->createExternalImage(buffer, sRGB);
#elif IMP_MATERIAL_API(VULKAN)
  buffer_handle =
      static_cast<filament::backend::VulkanPlatformAndroid*>(platform_)
          ->createExternalImage(buffer, sRGB);
#endif
  return buffer_handle;
}

SharedHostState::ExternalImageMetadata SharedHostState::GetImageMetadata(
    filament::backend::Platform::ExternalImageHandle externalImage) {
  ExternalImageMetadata metadata;
#if IMP_MATERIAL_API(OPENGL)
  auto eglExternalImageMetadata =
      static_cast<filament::backend::PlatformEGLAndroid*>(platform_)
          ->getExternalImageDesc(externalImage);
  metadata.height = eglExternalImageMetadata.height;
  metadata.width = eglExternalImageMetadata.width;
  metadata.format = eglExternalImageMetadata.format;
  metadata.usage = eglExternalImageMetadata.usage;
#elif IMP_MATERIAL_API(VULKAN)
  auto fvkExternalImage =
      static_cast<filament::backend::VulkanPlatformAndroid*>(platform_)
          ->getExternalImageDesc(externalImage);
  metadata.height = fvkExternalImage.height;
  metadata.width = fvkExternalImage.width;
  metadata.format = fvkExternalImage.format;
  metadata.usage = fvkExternalImage.usage;
#endif
  return metadata;
};
#endif  // IMP_PLATFORM(ANDROID)

#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
namespace {

static const char* GetEGLError() {
  const char* err;
  switch (eglGetError()) {
    case EGL_NOT_INITIALIZED:
      err = "EGL_NOT_INITIALIZED";
      break;
    case EGL_BAD_ACCESS:
      err = "EGL_BAD_ACCESS";
      break;
    case EGL_BAD_ALLOC:
      err = "EGL_BAD_ALLOC";
      break;
    case EGL_BAD_ATTRIBUTE:
      err = "EGL_BAD_ATTRIBUTE";
      break;
    case EGL_BAD_CONTEXT:
      err = "EGL_BAD_CONTEXT";
      break;
    case EGL_BAD_CONFIG:
      err = "EGL_BAD_CONFIG";
      break;
    case EGL_BAD_CURRENT_SURFACE:
      err = "EGL_BAD_CURRENT_SURFACE";
      break;
    case EGL_BAD_DISPLAY:
      err = "EGL_BAD_DISPLAY";
      break;
    case EGL_BAD_SURFACE:
      err = "EGL_BAD_SURFACE";
      break;
    case EGL_BAD_MATCH:
      err = "EGL_BAD_MATCH";
      break;
    case EGL_BAD_PARAMETER:
      err = "EGL_BAD_PARAMETER";
      break;
    case EGL_BAD_NATIVE_PIXMAP:
      err = "EGL_BAD_NATIVE_PIXMAP";
      break;
    case EGL_BAD_NATIVE_WINDOW:
      err = "EGL_BAD_NATIVE_WINDOW";
      break;
    case EGL_CONTEXT_LOST:
      err = "EGL_CONTEXT_LOST";
      break;
    default:
      err = "unknown";
      break;
  }
  return err;
}

void* CreateDefaultEGLContext(EGLDisplay* out_display, EGLContext* out_context,
                              EGLSurface* out_surface) {
  if (eglGetCurrentContext() != EGL_NO_CONTEXT) {
    IMP_LOG(imp::ERROR) << "Cannot create EGL context - one already exists.";
    return EGL_NO_CONTEXT;
  }

  EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  EGLint major = 0;
  EGLint minor = 0;
  if (egl_display == EGL_NO_DISPLAY ||
      !eglInitialize(egl_display, &major, &minor)) {
    IMP_LOG(imp::ERROR) << "Error: Failed to initialize display: " << GetEGLError();
    return EGL_NO_CONTEXT;
  }

  EGLint attribs[] = {
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT, EGL_SURFACE_TYPE,
      EGL_PBUFFER_BIT,     EGL_NONE,
  };
  EGLConfig egl_config;
  int numconfig;
  eglChooseConfig(egl_display, attribs, &egl_config, 1, &numconfig);
  if (numconfig != 1) {
    IMP_LOG(imp::ERROR) << "Cannot create EGL context - no OpenGL ES3 support";
    return EGL_NO_CONTEXT;
  }

  // Safety.
  if (!eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                      EGL_NO_CONTEXT)) {
    IMP_LOG(imp::ERROR) << "Error: failed to clear context: " << GetEGLError();
    return EGL_NO_CONTEXT;
  }

  EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
  EGLContext egl_context = eglCreateContext(egl_display, egl_config,
                                            EGL_NO_CONTEXT, context_attribs);
  if (egl_context == EGL_NO_CONTEXT) {
    IMP_LOG(imp::ERROR) << "Error: eglCreateContext failed: " << GetEGLError();
    return EGL_NO_CONTEXT;
  }

  EGLint surface_attribs[] = {EGL_WIDTH, 2, EGL_HEIGHT, 2, EGL_NONE};
  EGLSurface egl_surface =
      eglCreatePbufferSurface(egl_display, egl_config, surface_attribs);
  if (egl_surface == EGL_NO_SURFACE) {
    IMP_LOG(imp::ERROR) << "Error: egl surface failed: " << GetEGLError();
    return EGL_NO_CONTEXT;
  }

  if (!eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context)) {
    IMP_LOG(imp::ERROR) << "Error: failed to create surface: " << GetEGLError();
    return EGL_NO_CONTEXT;
  }

  *out_display = egl_display;
  *out_context = egl_context;
  *out_surface = egl_surface;

  return egl_context;
}

void DestroyDefaultEGLContext(EGLDisplay egl_display, EGLContext egl_context,
                              EGLSurface egl_surface) {
  eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  eglDestroySurface(egl_display, egl_surface);
  eglDestroyContext(egl_display, egl_context);
  eglTerminate(egl_display);
  eglReleaseThread();
}

}  // namespace
#endif  // IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)

}  // namespace imp::window
