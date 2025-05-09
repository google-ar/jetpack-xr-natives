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

#include <algorithm>
#include <cmath>
#include <memory>

#include "absl/memory/memory.h"
#include "absl/strings/str_format.h"
#include "core/scripting/basic_api.h"
#include "core/scripting/proto/test.proto.imp.h"
#include "core/scripting/scripting_system.h"
#include "core/scripting/scripting_system_web.h"
#include "core/view/framework/tests/data/test_model_resources.h"
#include "imp.h"

namespace imp {
namespace scripting {

// An Imp View for running a WASM end-to-end test.
class E2EView : public View {
 protected:
  void Setup() override {
    test_data::
        RegisterCoreViewFrameworkTestsDataTestModelResources();  // NOLINT

    WebViewParams params;
    // Create a WebView with the specified location, dimensions, and url.
    scripting_system_ = CreateScriptingSystemWeb(*this, params);
    AddBasicApiMessageHandlers(*scripting_system_, *this);

    // Register the Event1 type so we can test sending/receiving events.
    scripting_system_->RegisterEventType<Event1>();
    // TODO: Get something like this to work instead of the model
    // URI being hardcoded in the test javascript.
    // imp::AssetDefinition test_model = imp::test_data::kSceneABGltf;
    // InjectScriptToBridge(
    //     absl::StrFormat("window.modelUri = '%s';", test_model.url).c_str());
  }

 private:
  std::unique_ptr<ScriptingSystem> scripting_system_;
};

}  // namespace scripting

// Tells the platform-code which View to instantiate.
const bool kIsCreateViewAssigned =
    client_api::SetCreateViewFn([](absl::string_view identifier) {
      return View::Create<scripting::E2EView>("WasmE2ETestView");
    });

}  // namespace imp
