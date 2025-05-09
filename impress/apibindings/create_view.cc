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

#include "absl/strings/string_view.h"
#include "apibindings/impress_api_view.h"
#include "imp.h"

namespace imp {

// Tells Impress which subclass of View to instantiate when
// ImpSplitEngineRenderer is created.
const bool kIsCreateViewAssigned =
    client_api::SetCreateViewFn([](absl::string_view identifier) {
      return View::Create<ImpressApiView>("Impress Api View");
    });

}  // namespace imp
