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
#include <unistd.h>

#include "absl/strings/numbers.h"
#include "absl/synchronization/notification.h"
#include "core/materials/compiler/material_compiler_service.h"

int main(int argc, char* argv[]) {

  // We get the file descriptor via arg.
  if (argc < 2) {
    return 1;
  }

  absl::Notification on_close;

  int fd;
  if (!absl::SimpleAtoi(argv[1], &fd)) {
    return 1;
  }

  imp::MaterialCompilerService service(fd,
                                       [&on_close]() { on_close.Notify(); });

  // Wait until the service is closed.
  on_close.WaitForNotification();

  return 0;
}
