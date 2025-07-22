// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>

#include "android.view.h"

namespace wrap {
namespace android::view {
Display::Meta::Meta(bool deferDrop)
    : MetaBaseDroppable(Display::getTypeName()),
      DEFAULT_DISPLAY(classRef(), "DEFAULT_DISPLAY"),
      getDisplayId(classRef().getMethod("getDisplayId", "()I")),
      getName(classRef().getMethod("getName", "()Ljava/lang/String;")),
      getDeviceProductInfo(classRef().getMethod(
          "getDeviceProductInfo",
          "()Landroid/hardware/display/DeviceProductInfo;")),
      getRealSize(
          classRef().getMethod("getRealSize", "(Landroid/graphics/Point;)V")),
      getRealMetrics(classRef().getMethod("getRealMetrics",
                                          "(Landroid/util/DisplayMetrics;)V")) {
    if (!deferDrop) {
        MetaBaseDroppable::dropClassRef();
    }
}
Surface::Meta::Meta()
    : MetaBaseDroppable(Surface::getTypeName()),
      isValid(classRef().getMethod("isValid", "()Z")) {
    MetaBaseDroppable::dropClassRef();
}
SurfaceHolder::Meta::Meta()
    : MetaBaseDroppable(SurfaceHolder::getTypeName()),
      getSurface(
          classRef().getMethod("getSurface", "()Landroid/view/Surface;")) {
    MetaBaseDroppable::dropClassRef();
}
WindowManager::Meta::Meta()
    : MetaBaseDroppable(WindowManager::getTypeName()),
      getDefaultDisplay(classRef().getMethod("getDefaultDisplay",
                                             "()Landroid/view/Display;")) {
    MetaBaseDroppable::dropClassRef();
}
WindowManager_LayoutParams::Meta::Meta()
    : MetaBaseDroppable(WindowManager_LayoutParams::getTypeName()),
      FLAG_FULLSCREEN(classRef(), "FLAG_FULLSCREEN"),
      FLAG_NOT_FOCUSABLE(classRef(), "FLAG_NOT_FOCUSABLE"),
      FLAG_NOT_TOUCHABLE(classRef(), "FLAG_NOT_TOUCHABLE"),
      TYPE_APPLICATION(classRef(), "TYPE_APPLICATION"),
      TYPE_APPLICATION_OVERLAY(classRef(), "TYPE_APPLICATION_OVERLAY"),
      init(classRef().getMethod("<init>", "()V")),
      init1(classRef().getMethod("<init>", "(I)V")),
      init2(classRef().getMethod("<init>", "(II)V")),
      init3(classRef().getMethod("<init>", "(III)V")),
      init4(classRef().getMethod("<init>", "(IIIII)V")),
      init5(classRef().getMethod("<init>", "(IIIIIII)V")),
      init6(classRef().getMethod("<init>", "(Landroid/os/Parcel;)V")),
      setTitle(
          classRef().getMethod("setTitle", "(Ljava/lang/CharSequence;)V")) {
    MetaBaseDroppable::dropClassRef();
}
Display_Mode::Meta::Meta()
    : MetaBaseDroppable(Display_Mode::getTypeName()),
      getModeId(classRef().getMethod("getModeId", "()I")),
      getPhysicalWidth(classRef().getMethod("getPhysicalWidth", "()I")),
      getPhysicalHeight(classRef().getMethod("getPhysicalHeight", "()I")),
      getRefreshRate(classRef().getMethod("getRefreshRate", "()F")) {
    MetaBaseDroppable::dropClassRef();
}
} // namespace android::view
} // namespace wrap
