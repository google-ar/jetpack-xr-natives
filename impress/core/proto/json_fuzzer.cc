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

#include <cstddef>
#include <cstdint>

#include "glog/logging.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/proto/fuzz.proto.imp.h"
#include "core/proto/json_reader.h"
#include "core/proto/json_writer.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  absl::string_view view(reinterpret_cast<const char*>(data), size);
  fuzz::FuzzEverything msg;
  if (imp::proto::ParseJson(view, &msg).ok()) {
    std::string gen1;
    

    fuzz::FuzzEverything readback;
    

    std::string gen2;
    

    
  } else {
    absl::PrintF("couldn't parse %u bytes\n", size);
  }
  return 0;
}
