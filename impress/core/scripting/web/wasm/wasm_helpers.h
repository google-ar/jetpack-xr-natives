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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WASM_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WASM_HELPERS_H_

#include "absl/strings/string_view.h"
#include "core/config.h"
#if IMP_PLATFORM(WASM)
#include <emscripten/emscripten.h>

#include <cstdio>
#include <string>

#include "absl/strings/escaping.h"
#include "absl/strings/str_format.h"
#endif

namespace imp::scripting {

inline constexpr absl::string_view kInjectToBridge =
    "window.javaScriptEntryPoint.incoming.injectScript(`%s`);";

// Injects the given script into the IframeBridge's inject endpoint.
static void InjectScriptToBridge(absl::string_view script) {
#if IMP_PLATFORM(WASM)
  std::string encoded;
  absl::Base64Escape(script, &encoded);
  std::string inject_script_str(absl::StrFormat(kInjectToBridge, encoded));
  // TODO: use MAIN_THREAD_ASM here?
  emscripten_run_script(inject_script_str.c_str());
#endif
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WASM_HELPERS_H_
