/*
 * Copyright 2025 Google LLC
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
#include "core/materials/compiler/desktop_runtime_material_compiler.h"

#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "devtools/build/runtime/get_runfiles_dir.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/async/future.h"
#include "core/materials/compiler/cache/material_cache.h"
#include "core/materials/compiler/material_compiler_client.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/view/base_view.h"

namespace imp {

constexpr char kDesktopMaterialCompilerMainPath[] =
    "google3/third_party/impress/core/materials/compiler/"
    "desktop_runtime_material_compiler_main";

Future<std::unique_ptr<RuntimeMaterialCompiler>>
DesktopRuntimeMaterialCompiler::Create(BaseView& view) {
  int fds[2];
  if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
    return absl::InternalError("Failed to create socket pair");
  }

  // Client fd: fds[0], Service (new process) fd: fds[1].
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  // When a new process is spawned, the child process also has both file
  // descriptors. We need to close client fd in service, and close service fd
  // in client.
  // In the child process (Service), close the parent fd (Client).
  posix_spawn_file_actions_addclose(&actions, fds[0]);

  std::string compiler_path = devtools_build::GetDataDependencyFilepath(
      kDesktopMaterialCompilerMainPath);
  std::string fd_str = absl::StrCat(fds[1]);
  char* const child_argv[] = {compiler_path.data(), fd_str.data(), nullptr};

  // Launch the material compiler service.
  // We don't want to inherit the parent process as the material service is
  // meant to be isolated.
  char* const empty_env[] = {nullptr};
  pid_t pid;
  if (int status = posix_spawn(&pid, compiler_path.c_str(), &actions, nullptr,
                               child_argv, empty_env);
      status != 0) {
    // Failed to spawn a process, clean up everything and return failure.
    posix_spawn_file_actions_destroy(&actions);
    close(fds[0]);
    close(fds[1]);
    return absl::InternalError(absl::StrCat(
        "Failed to spawn MaterialCompilerService, reason: ", strerror(status)));
  }

  // Clean up the action.
  posix_spawn_file_actions_destroy(&actions);
  // In the parent process (Client), close the child fd (Service).
  close(fds[1]);

  return MaterialCache::Create(view.GetContext())
      .Then([&view, pid, client_fd = fds[0]](
                absl::StatusOr<std::unique_ptr<MaterialCache>> cache)
                -> absl::StatusOr<std::unique_ptr<RuntimeMaterialCompiler>> {
        if (!cache.status().ok()) {
          // Failed to create cache, clean up and return failure.
          close(client_fd);
          waitpid(pid, /*status=*/nullptr, /*options=*/0);
          return cache.status();
        }
        return absl::WrapUnique(new DesktopRuntimeMaterialCompiler(
            view, std::make_unique<MaterialCompilerClient>(client_fd), pid,
            std::move(*cache)));
      });
}

DesktopRuntimeMaterialCompiler::~DesktopRuntimeMaterialCompiler() {
  // The service process (desktop_runtime_material_compiler_main) waits for
  // the client to close the connection, so we need to close the socket here
  // otherwise `waitpid` will hang.
  if (native_client_) {
    native_client_->Close();
  }
  // Wait for the service process to close and clean up.
  waitpid(material_service_pid_, /*status=*/nullptr, /*options=*/0);
}

}  // namespace imp
