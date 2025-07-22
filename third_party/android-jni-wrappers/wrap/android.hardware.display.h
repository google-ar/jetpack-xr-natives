// Copyright 2022-2023, Qualcomm Innovation Center, Inc.
// Copyright 2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Jarvis Huang

#pragma once

#include "ObjectWrapperBase.h"
#include <string>

namespace wrap {
namespace android::view {
class Display;
} // namespace android::view

} // namespace wrap

namespace wrap {
namespace android::hardware::display {
/*!
 * Wrapper for android.hardware.display.DisplayManager objects.
 */
class DisplayManager : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/hardware/display/DisplayManager";
    }

    /*!
     * Getter for the DISPLAY_CATEGORY_PRESENTATION static field value
     *
     * Java prototype:
     * `public static final java.lang.String DISPLAY_CATEGORY_PRESENTATION;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    static std::string DISPLAY_CATEGORY_PRESENTATION();

    /*!
     * Wrapper for the getDisplay method
     *
     * Java prototype:
     * `public android.view.Display getDisplay(int);`
     *
     * JNI signature: (I)Landroid/view/Display;
     *
     */
    view::Display getDisplay(int32_t displayId) const;

    /*!
     * Wrapper for the getDisplays method
     *
     * Java prototype:
     * `public android.view.Display[] getDisplays();`
     *
     * JNI signature: ()[Landroid/view/Display;
     *
     */
    jni::Array<jni::Object> getDisplays() const;

    /*!
     * Wrapper for the getDisplays method
     *
     * Java prototype:
     * `public android.view.Display[] getDisplays(java.lang.String);`
     *
     * JNI signature: (Ljava/lang/String;)[Landroid/view/Display;
     *
     */
    jni::Array<jni::Object> getDisplays(std::string const &category) const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::StaticFieldId<std::string> DISPLAY_CATEGORY_PRESENTATION;
        jni::method_t getDisplay;
        jni::method_t getDisplays;
        jni::method_t getDisplays1;

        /*!
         * Singleton accessor
         */
        static Meta &data(bool deferDrop = false) {
            static Meta instance{deferDrop};
            return instance;
        }

      private:
        explicit Meta(bool deferDrop);
    };
};

/*!
 * Wrapper for android.hardware.display.DeviceProductInfo objects.
 */
class DeviceProductInfo : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/hardware/display/DeviceProductInfo";
    }

    /*!
     * Wrapper for the getName method
     *
     * Java prototype:
     * `public java.lang.String getName();`
     *
     * JNI signature: ()Ljava/lang/String;
     *
     */
    std::string getName() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        jni::method_t getName;

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

} // namespace android::hardware::display
} // namespace wrap
#include "android.hardware.display.impl.h"
