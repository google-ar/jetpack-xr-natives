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

#include "core/canvas/fonts/system_font_provider.h"

#include <memory>

#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/common/context.h"
#include "core/config.h"

#if IMP_PLATFORM(ANDROID)
#include "core/canvas/fonts/android_system_font_provider.h"
#elif IMP_PLATFORM(IOS)
#include "core/canvas/fonts/ios_system_font_provider.h"
#elif IMP_PLATFORM(WASM)
#include "core/canvas/fonts/wasm_system_font_provider.h"
#else
#include "core/canvas/fonts/desktop_system_font_provider.h"
#endif

namespace imp {

std::unique_ptr<FontHolder> LoadSystemFont(const Context& context,
                                           const SystemFontParams& params) {
#if IMP_PLATFORM(ANDROID)
  return LoadSystemAndroidFont(context, params.android_family_name,
                               params.font_weight, params.text_style);
#elif IMP_PLATFORM(IOS)
  return LoadSystemIosFont(params.ios_family_name, params.font_weight,
                           params.text_style);
#elif IMP_PLATFORM(WASM)
  return LoadSystemWasmFont(params.wasm_family_name, params.font_weight,
                            params.text_style);
#else
  return LoadSystemDesktopFont(params.desktop_family_name, params.font_weight,
                               params.text_style);
#endif
}

}  // namespace imp
