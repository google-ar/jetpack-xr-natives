// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>

#include "android.graphics.h"

namespace wrap {
namespace android::graphics {
Point::Meta::Meta()
    : MetaBaseDroppable(Point::getTypeName()), x(classRef(), "x"),
      y(classRef(), "y") {
    MetaBaseDroppable::dropClassRef();
}
PixelFormat::Meta::Meta()
    : MetaBaseDroppable(PixelFormat::getTypeName()),
      OPAQUE(classRef(), "OPAQUE") {
    MetaBaseDroppable::dropClassRef();
}
} // namespace android::graphics
} // namespace wrap
