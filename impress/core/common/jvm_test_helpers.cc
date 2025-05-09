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

#include "core/common/jvm_test_helpers.h"

#include <string>
#include <vector>

#include "devtools/java/build/java_locations.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/flags/flag.h"

namespace imp {

JavaVM *InitializeJVM() {
  static bool initialized = false;
  if (!initialized) {
    // JRE uses relative path to find other .so files. Files on Forge are
    // replaced with symbolic links. Real directory structures are flattened. We
    // have to copy and dereference the links so that JRE can find the files.
    std::string jdk_dir = absl::GetFlag(::testing::TempDir()) + "/jdk-64";
    std::string cmd =
        "cp -rL " + devtools_java_build::GetJavaHomePath() + " " + jdk_dir;
    system(cmd.c_str());

    std::vector<std::string> args;
    EXPECT_TRUE(util::java::Jvm::Initialize(JNI_VERSION_1_8, jdk_dir, args));
    initialized = true;
  }
  return util::java::Jvm::Get();
}

}  // namespace imp
