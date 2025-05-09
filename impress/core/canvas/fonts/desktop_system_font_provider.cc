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

#include "core/canvas/fonts/desktop_system_font_provider.h"

#include <memory>

#include "absl/strings/string_view.h"
#include "core/canvas/fonts/desktop_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"

namespace imp {

std::unique_ptr<FontHolder> LoadSystemDesktopFont(absl::string_view family_name,
                                                  FontWeight font_weight,
                                                  TextStyle text_style) {
  return std::make_unique<DesktopFontHolder>(family_name, font_weight,
                                             text_style);
}

}  // namespace imp
