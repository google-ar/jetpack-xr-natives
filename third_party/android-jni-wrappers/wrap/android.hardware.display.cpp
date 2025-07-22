// Copyright 2022-2023, Qualcomm Innovation Center, Inc.
// Copyright 2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Jarvis Huang

#include "android.hardware.display.h"

namespace wrap {
namespace android::hardware::display {
DisplayManager::Meta::Meta(bool deferDrop)
    : MetaBaseDroppable(DisplayManager::getTypeName()),
      DISPLAY_CATEGORY_PRESENTATION(classRef(),
                                    "DISPLAY_CATEGORY_PRESENTATION"),
      getDisplay(
          classRef().getMethod("getDisplay", "(I)Landroid/view/Display;")),
      getDisplays(
          classRef().getMethod("getDisplays", "()[Landroid/view/Display;")),
      getDisplays1(classRef().getMethod(
          "getDisplays", "(Ljava/lang/String;)[Landroid/view/Display;")) {
    if (!deferDrop) {
        MetaBaseDroppable::dropClassRef();
    }
}
DeviceProductInfo::Meta::Meta()
    : MetaBaseDroppable(DeviceProductInfo::getTypeName()),
      getName(classRef().getMethod("getName", "()Ljava/lang/String;")) {
    MetaBaseDroppable::dropClassRef();
}
} // namespace android::hardware::display
} // namespace wrap
