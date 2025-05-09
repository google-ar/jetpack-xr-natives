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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_SYSTEM_FONT_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_SYSTEM_FONT_PROVIDER_H_

#include <memory>
#include <string>

#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/common/context.h"

namespace imp {

// Loads default system fonts for use with the CanvasSource API.
//
// The fonts available vary by what fonts are built-in to each platform
// supported by CanvasSource.
std::unique_ptr<FontHolder> LoadSystemFont(const Context& context,
                                           const SystemFontParams& params);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_FONTS_SYSTEM_FONT_PROVIDER_H_
