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

#include <memory>
#include <utility>


#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "core/async/executor.h"
#include "core/common/context.h"
#include "core/input/input_manager.h"
#include "core/view/framework/view.h"
#include "core/view/platforms/desktop/desktop_hooks.h"
#include "core/view/utils/device.h"
#include "core/view/view_host.h"
#include "core/window/sdl_venue.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace desktop {
namespace {

absl::Status PresentHost(ViewHost* host, InputManager* inputManager,
                         Device* device) {
  // TODO: Refactor so that host->setup is called after
  // the DPI values are set on View::Device for consistency with other platforms
  MP_RETURN_IF_ERROR(host->Setup());
  
  imp::desktop_api::PostHostSetup(*host->GetView(),
                                  *Executor::ForegroundExecutor());
  MP_RETURN_IF_ERROR(window::SdlVenue(host, inputManager, device));
  return absl::OkStatus();
}

absl::Status PresentHostAndCleanup(ViewHost* host, InputManager* inputManager,
                                   Device* device) {
  if (auto status = PresentHost(host, inputManager, device); !status.ok()) {
    if (auto cleanup_status = host->Cleanup(); !cleanup_status.ok()) {
      return absl::InternalError(absl::StrFormat(
          "%.*s after earlier error %.*s",
          static_cast<int>(cleanup_status.message().size()),
          cleanup_status.message().data(),
          static_cast<int>(status.message().size()), status.message().data()));
    }
    return status;
  }
  MP_RETURN_IF_ERROR(host->Cleanup());
  return absl::OkStatus();
}

absl::Status Main(int argc, char* argv[]) {
  

  std::unique_ptr<View> view =
      imp::View::CreateClient(std::make_unique<Context>(argc, argv));

  auto view_host = std::make_unique<ViewHost>(std::move(view));

  MP_RETURN_IF_ERROR(PresentHostAndCleanup(
      view_host.get(), &view_host->GetView()->GetInputManager(),
      &view_host->GetView()->GetDevice()));

  return absl::OkStatus();
}

}  // namespace
}  // namespace desktop
}  // namespace imp

int main(int argc, char* argv[]) {
  absl::InitializeSymbolizer(argv[0]);

  absl::FailureSignalHandlerOptions options;
  absl::InstallFailureSignalHandler(options);

  if (auto status = imp::desktop::Main(argc, argv); !status.ok()) {
    IMP_LOG(imp::ERROR) << status;
    return -1;
  }
  return 0;
}
