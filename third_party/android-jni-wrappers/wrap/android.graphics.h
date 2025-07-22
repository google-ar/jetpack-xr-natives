// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>

#pragma once

#include "ObjectWrapperBase.h"

namespace wrap {
namespace android::graphics {
/*!
 * Wrapper for android.graphics.Point objects.
 */
class Point : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/graphics/Point";
    }

    /*!
     * Getter for the x field value
     *
     * Java prototype:
     * `public int x;`
     *
     * JNI signature: I
     *
     */
    int32_t getX() const;

    /*!
     * Getter for the y field value
     *
     * Java prototype:
     * `public int y;`
     *
     * JNI signature: I
     *
     */
    int32_t getY() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::FieldId<int32_t> x;
        impl::FieldId<int32_t> y;

        /*!
         * Singleton accessor
         */
        static Meta &data() {
            static Meta instance{};
            return instance;
        }

      private:
        Meta();
    };
};

/*!
 * Wrapper for android.graphics.PixelFormat objects.
 */
class PixelFormat : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/graphics/PixelFormat";
    }

    /*!
     * Getter for the OPAQUE static field value
     *
     * Java prototype:
     * `public static final int OPAQUE;`
     *
     * JNI signature: I
     *
     */
    static int32_t OPAQUE();

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::StaticFieldId<int32_t> OPAQUE;

        /*!
         * Singleton accessor
         */
        static Meta &data() {
            static Meta instance{};
            return instance;
        }

      private:
        Meta();
    };
};

} // namespace android::graphics
} // namespace wrap
#include "android.graphics.impl.h"
