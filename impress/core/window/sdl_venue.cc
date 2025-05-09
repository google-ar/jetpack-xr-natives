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

#include "core/window/sdl_venue.h"

#include <SDL.h>
#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "SDL2/include/SDL_keyboard.h"
#include "SDL2/include/SDL_keycode.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/common/enum_flags.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/file_helpers.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/view/platforms/desktop/clipboard/sdl_clipboard_handler.h"
#include "core/window/native_window_helper.h"
#include "core/window/sdl_input_processor.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_PLATFORM(LINUX)
#include <SDL_syswm.h>
#endif  // IMP_PLATFORM(LINUX)

// Fix for X11 defining a Status macro that interferes with absl::Status.
#if defined(Status)
#undef Status
typedef int Status;
#endif

namespace imp {
namespace window {

namespace {

using ::filament::math::float2;
using ::filament::math::int2;
using ::filament::math::uint2;
using ::imp::FlushEngineAndWait;
using ::imp::InputManager;

static constexpr int kDefaultMousePointerId = 0;

// Venue is the current strawman-name for the thing that hosts a host.
class SdlVenueImpl {
 public:
  explicit SdlVenueImpl(FilamentHost* host, InputManager* inputManager,
                        Device* device)
      : host_(host),
        input_manager_(inputManager),
        device_(device),
        window_(nullptr),
        do_post_loop_(true) {
    host->SetClipboardHandler(
        std::make_unique<SdlClipboardHandler>());
  }
  OptionalError CreateWindow(const uint2& dimensions);
  void DestroyWindow();
  SDL_Window* GetWindow() { return window_; }
  bool CanContinueRendering();
  OptionalError Cleanup(bool encountered_errors);

 private:
  void OnWindowSizeChanged();
  void PostLoop();
  bool PumpMessagesAndCheckForQuit();

  FilamentHost* host_;
  InputManager* input_manager_;
  Device* device_;
  SDL_Window* window_;
  bool do_post_loop_;
};

bool SdlVenueImpl::PumpMessagesAndCheckForQuit() {
  SDL_Event e;
  bool quit = false;
  while (SDL_PollEvent(&e)) {
    absl::Status process_input_status =
        ProcessInputFromSdlEvent(host_, input_manager_, &e);
    if (!process_input_status.ok()) {
      IMP_LOG(imp::ERROR) << process_input_status;
    }

    switch (e.type) {
      case SDL_QUIT: {
        quit = true;
        break;
      }
      case SDL_WINDOWEVENT: {
        switch (e.window.event) {
          case SDL_WINDOWEVENT_RESIZED: {
            OnWindowSizeChanged();
            host_->UpdateCamerasForWindow();
            break;
          }
          default: {
            break;
          }
        }
        break;
      }
      case SDL_DROPFILE: {
        std::string input_path(e.drop.file);
        SDL_free(e.drop.file);
        if (auto status = host_->OnFileDrop(input_path); !status.ok()) {
          IMP_LOG(imp::ERROR) << status;
          return true;
        }
        break;
      }
    }
  }
  return quit;
}

void SdlVenueImpl::PostLoop() {
  std::string post_loop_title = absl::StrFormat(
      "%s - Finished Rendering", host_->GetState()->Title(host_));
  SDL_SetWindowTitle(window_, post_loop_title.c_str());

  while (!PumpMessagesAndCheckForQuit()) {
    absl::SleepFor(absl::Milliseconds(16));
  }
}

void SdlVenueImpl::OnWindowSizeChanged() {
  if (auto status = host_->DestroySwapChain(); !status.ok()) {
    IMP_LOG(imp::ERROR) << status;
  }
  if (auto status = host_->CreateSwapChain(Impress_getNativeWindow(window_));
      !status.ok()) {
    IMP_LOG(imp::FATAL) << status;
  }
  int draw_width, draw_height;
  SDL_GL_GetDrawableSize(window_, &draw_width, &draw_height);
  int virtual_width, virtual_height;
  SDL_GetWindowSize(window_, &virtual_width, &virtual_height);
  uint2 subpixel_ratio = {draw_width / virtual_width,
                          draw_height / virtual_height};
  host_->Resize(float2{virtual_width, virtual_height} * subpixel_ratio,
                subpixel_ratio);
}

OptionalError SdlVenueImpl::CreateWindow(const uint2& dimensions) {
  FilamentHost::State* state = host_->GetState();
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) {
    return Error("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
  }

  // Create window
  std::string title = state->Title(host_);
  const uint32_t window_flags =
      SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
  window_ = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, dimensions.x,
                             dimensions.y, window_flags);
  if (window_ == nullptr) {
    return Error("Window could not be created! SDL_Error: %s\n",
                 SDL_GetError());
  }
  MP_RETURN_IF_ERROR(host_->CreateSwapChain(Impress_getNativeWindow(window_), 0));

  int draw_width, draw_height;
  SDL_GL_GetDrawableSize(window_, &draw_width, &draw_height);
  int virtual_width, virtual_height;
  SDL_GetWindowSize(window_, &virtual_width, &virtual_height);
  filament::math::uint2 subpixel_ratio = {draw_width / virtual_width,
                                          draw_height / virtual_height};
  host_->Resize(float2{virtual_width, virtual_height} * subpixel_ratio,
                subpixel_ratio);

  return NoError();
}

bool SdlVenueImpl::CanContinueRendering() {
  if (!host_->GetState()->IsStillRendering(host_)) {
    return false;  // Rendering Complete.
  }
  if (PumpMessagesAndCheckForQuit()) {
    do_post_loop_ = false;
    return false;  // Quit signal received from Filament Host.
  }
  return true;
}

OptionalError SdlVenueImpl::Cleanup(bool encountered_errors) {
  if (!encountered_errors && do_post_loop_) {
    PostLoop();
  }
  MP_RETURN_IF_ERROR(host_->Cleanup());
  DestroyWindow();
  return NoError();
}

void SdlVenueImpl::DestroyWindow() {
  FlushEngineAndWait(host_->GetEngine());
  if (auto status = host_->DestroySwapChain(); !status.ok()) {
    IMP_LOG(imp::ERROR) << status;
  }
  SDL_DestroyWindow(window_);
  SDL_Quit();
}
}  // namespace

OptionalError SdlVenue(FilamentHost* host, InputManager* input,
                       Device* device) {
  return SdlVenue(host, input, device,
                  host->GetState()->DesiredDimensions(host));
}

OptionalError SdlVenue(FilamentHost* host, InputManager* input, Device* device,
                       const uint2& dimensions) {
  SdlVenueImpl venue(host, input, device);
  MP_RETURN_IF_ERROR(venue.CreateWindow(dimensions));
  auto status = NoError();
  auto first = absl::Now();
  auto start = first;
  static const absl::Duration kMinDelta = absl::Seconds(1) / 60;

  while (status.ok() && venue.CanContinueRendering()) {
    auto end = absl::Now();
    auto delta = end - start;
    if (delta < kMinDelta) {
      absl::SleepFor(kMinDelta - delta);
      continue;
    }

    status = host->RenderNextFrame(start - first, end - first).status();
    start = end;
  }
  venue.DestroyWindow();
  return NoError();
}

}  // namespace window
}  // namespace imp
