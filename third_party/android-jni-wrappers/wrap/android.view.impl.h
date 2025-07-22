// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>
// Inline implementations: do not include on its own!

#pragma once

#include "android.graphics.h"
#include "android.hardware.display.h"
#include "android.util.h"
#include <string>

namespace wrap {
namespace android::view {
inline int32_t Display::DEFAULT_DISPLAY() {
    auto &data = Meta::data(true);
    auto ret = get(data.DEFAULT_DISPLAY, data.clazz());
    data.dropClassRef();
    return ret;
}

inline int32_t Display::getDisplayId() const {
    assert(!isNull());
    return object().call<int32_t>(Meta::data().getDisplayId);
}

inline std::string Display::getName() const {
    assert(!isNull());
    return object().call<std::string>(Meta::data().getName);
}

inline hardware::display::DeviceProductInfo
Display::getDeviceProductInfo() const {
    assert(!isNull());
    return hardware::display::DeviceProductInfo(
        object().call<jni::Object>(Meta::data().getDeviceProductInfo));
}

inline void Display::getRealSize(graphics::Point &out_size) {
    assert(!isNull());
    return object().call<void>(Meta::data().getRealSize, out_size.object());
}

inline void Display::getRealMetrics(util::DisplayMetrics &out_displayMetrics) {
    assert(!isNull());
    return object().call<void>(Meta::data().getRealMetrics,
                               out_displayMetrics.object());
}

inline bool Surface::isValid() const {
    assert(!isNull());
    return object().call<bool>(Meta::data().isValid);
}

inline Surface SurfaceHolder::getSurface() {
    assert(!isNull());
    return Surface(object().call<jni::Object>(Meta::data().getSurface));
}

inline Display WindowManager::getDefaultDisplay() const {
    assert(!isNull());
    return Display(object().call<jni::Object>(Meta::data().getDefaultDisplay));
}

inline int32_t WindowManager_LayoutParams::FLAG_FULLSCREEN() {
    return get(Meta::data().FLAG_FULLSCREEN, Meta::data().clazz());
}

inline int32_t WindowManager_LayoutParams::FLAG_NOT_FOCUSABLE() {
    return get(Meta::data().FLAG_NOT_FOCUSABLE, Meta::data().clazz());
}

inline int32_t WindowManager_LayoutParams::FLAG_NOT_TOUCHABLE() {
    return get(Meta::data().FLAG_NOT_TOUCHABLE, Meta::data().clazz());
}

inline int32_t WindowManager_LayoutParams::TYPE_APPLICATION() {
    return get(Meta::data().TYPE_APPLICATION, Meta::data().clazz());
}

inline int32_t WindowManager_LayoutParams::TYPE_APPLICATION_OVERLAY() {
    return get(Meta::data().TYPE_APPLICATION_OVERLAY, Meta::data().clazz());
}

inline WindowManager_LayoutParams WindowManager_LayoutParams::construct() {
    return WindowManager_LayoutParams(
        Meta::data().clazz().newInstance(Meta::data().init));
}

inline WindowManager_LayoutParams
WindowManager_LayoutParams::construct(int32_t type) {
    return WindowManager_LayoutParams(
        Meta::data().clazz().newInstance(Meta::data().init1, type));
}

inline WindowManager_LayoutParams
WindowManager_LayoutParams::construct(int32_t type, int32_t flags) {
    return WindowManager_LayoutParams(
        Meta::data().clazz().newInstance(Meta::data().init2, type, flags));
}

inline WindowManager_LayoutParams
WindowManager_LayoutParams::construct(int32_t intParam, int32_t intParam1,
                                      int32_t intParam2) {
    return WindowManager_LayoutParams(Meta::data().clazz().newInstance(
        Meta::data().init3, intParam, intParam1, intParam2));
}

inline WindowManager_LayoutParams
WindowManager_LayoutParams::construct(int32_t w, int32_t h, int32_t type,
                                      int32_t flags, int32_t format) {
    return WindowManager_LayoutParams(Meta::data().clazz().newInstance(
        Meta::data().init4, w, h, type, flags, format));
}

inline WindowManager_LayoutParams WindowManager_LayoutParams::construct(
    int32_t intParam, int32_t intParam1, int32_t intParam2, int32_t intParam3,
    int32_t intParam4, int32_t intParam5, int32_t intParam6) {
    return WindowManager_LayoutParams(Meta::data().clazz().newInstance(
        Meta::data().init5, intParam, intParam1, intParam2, intParam3,
        intParam4, intParam5, intParam6));
}

inline WindowManager_LayoutParams
WindowManager_LayoutParams::construct(jni::Object const &parcel) {
    return WindowManager_LayoutParams(
        Meta::data().clazz().newInstance(Meta::data().init6, parcel));
}

inline void WindowManager_LayoutParams::setTitle(std::string const &title) {
    assert(!isNull());
    return object().call<void>(Meta::data().setTitle, title);
}

inline int32_t Display_Mode::getModeId() {
    assert(!isNull());
    return object().call<int32_t>(Meta::data().getModeId);
}

inline int32_t Display_Mode::getPhysicalWidth() {
    assert(!isNull());
    return object().call<int32_t>(Meta::data().getPhysicalWidth);
}

inline int32_t Display_Mode::getPhysicalHeight() {
    assert(!isNull());
    return object().call<int32_t>(Meta::data().getPhysicalHeight);
}

inline float Display_Mode::getRefreshRate() {
    assert(!isNull());
    return object().call<float>(Meta::data().getRefreshRate);
}

} // namespace android::view
} // namespace wrap
