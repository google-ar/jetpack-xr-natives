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

#include "core/canvas/fonts/desktop_font_holder.h"

#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

#include "absl/base/macros.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"
#include "third_party/skia/HEAD/include/core/SkRefCnt.h"
#include "third_party/skia/HEAD/include/core/SkString.h"
#if IMP_PLATFORM(MACOS)
#include "third_party/skia/HEAD/include/ports/SkFontMgr_mac_ct.h"
#elif IMP_PLATFORM(LINUX)
#include "absl/debugging/leak_check.h"
#include "third_party/fontconfig/fontconfig/fontconfig.h"
#include "third_party/skia/HEAD/include/ports/SkFontMgr_fontconfig.h"
#endif

namespace imp {
namespace {

sk_sp<SkFontMgr> CreateFallbackFontManager() {
#if IMP_PLATFORM(LINUX)
  static sk_sp<SkFontMgr> font_mgr = nullptr;

  if (font_mgr == nullptr) {
    // FontConfig has a memory leak we should suppress.
    // (broken link)
    absl::LeakCheckDisabler disabler;
    FcConfig* config = FcConfigCreate();
    FcConfigSetSysRoot(config, reinterpret_cast<const FcChar8*>(
                                   "third_party/impress/core/fonts"));
    /*
    Available fonts for tests:
      Noto Sans
      Google Sans Arabic
      Google Sans CJK Japanese 144pt
      Noto Color Emoji
      Noto Sans Hebrew
      Noto Sans Khmer
    */
    absl::string_view fonts[] = {
        "/googlesans_arabic_medium.ttf",
        "/googlesans_japanese_medium.ttf",
        "/googlesans_myanmar_medium.ttf",
    };
    for (int i = 0; i < ABSL_ARRAYSIZE(fonts); ++i) {
      SkString fontFilePath(
          reinterpret_cast<const char*>(FcConfigGetSysRoot(config)));
      fontFilePath.append(fonts[i]);
      FcConfigAppFontAddFile(
          config, reinterpret_cast<const FcChar8*>(fontFilePath.c_str()));
    }
    FcConfigBuildFonts(config);
    font_mgr = SkFontMgr_New_FontConfig(config);
  }
  return font_mgr;
#else
  return nullptr;
#endif
}
}  // namespace

DesktopFontHolder::DesktopFontHolder()
    : DesktopFontHolder(/*family_name=*/std::nullopt,
                        FontWeight::FONT_WEIGHT_NORMAL,
                        TextStyle::TEXT_STYLE_NORMAL) {}

DesktopFontHolder::DesktopFontHolder(
    std::optional<absl::string_view> family_name, FontWeight font_weight,
    TextStyle text_style)
    : font_weight_(font_weight), text_style_(text_style) {
  font_collection_ = sk_make_sp<FontCollection>();
  sk_sp<SkFontMgr> font_mgr = GetFontMgr();
  if (font_mgr != nullptr) {
    if (family_name.has_value()) {
      font_name_ = std::string(*family_name);
      font_collection_->setDefaultFontManager(std::move(font_mgr),
                                              {SkString(*family_name)});
    } else {
      font_collection_->setDefaultFontManager(std::move(font_mgr));
    }
  } else {
    IMP_LOG(imp::ERROR) << "Unable to create a font manager; text will not be rendered";
  }
}

void* DesktopFontHolder::GetPlatformFont() {
  return static_cast<void*>(font_collection_.get());
}

sk_sp<SkFontMgr> DesktopFontHolder::GetFontMgr() {
  sk_sp<SkFontMgr> font_mgr = nullptr;
  // Each platform has its own respective default font manager implementation
  // that will use fonts provided from that system.
#if IMP_PLATFORM(MACOS)
  font_mgr = SkFontMgr_New_CoreText(nullptr);
#elif IMP_PLATFORM(LINUX)
  // Check to see if we are in a testing environment. Forge doesn't supply us
  // with a FontConfig so we need to use the fallback font manager.
  if (getenv("TEST_TMPDIR") == nullptr) {
    font_mgr = SkFontMgr_New_FontConfig(nullptr);
  }
  if (font_mgr == nullptr) {
    IMP_LOG(imp::ERROR) << "FontConfig not found on system, creating fallback.";
    font_mgr = CreateFallbackFontManager();
  }
#endif
  return font_mgr;
}

absl::string_view DesktopFontHolder::GetFontName() const { return font_name_; }

FontWeight DesktopFontHolder::GetFontWeight() const { return font_weight_; }

TextStyle DesktopFontHolder::GetTextStyle() const { return text_style_; }

}  // namespace imp
