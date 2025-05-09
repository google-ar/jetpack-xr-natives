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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_KEY_CODES_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_KEY_CODES_H_

#include "absl/strings/string_view.h"
#include "core/input/key_codes.h"

namespace imp::scripting {
VirtualKeyCode ToVirtualKeyCode(absl::string_view web_key_code);

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_KEY_CODES_H_
