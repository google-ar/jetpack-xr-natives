// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>
// Inline implementations: do not include on its own!

#pragma once

namespace wrap {
namespace android::graphics {
inline int32_t Point::getX() const {
    assert(!isNull());
    return get(Meta::data().x, object());
}

inline int32_t Point::getY() const {
    assert(!isNull());
    return get(Meta::data().y, object());
}

inline int32_t PixelFormat::OPAQUE() {
    return get(Meta::data().OPAQUE, Meta::data().clazz());
}

} // namespace android::graphics
} // namespace wrap
