// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>

#pragma once

#include "ObjectWrapperBase.h"
#include "android.os.h"
#include <string>

namespace wrap {
namespace android::content {
class Intent;
} // namespace android::content

namespace android::content::pm {
class ApplicationInfo;
class PackageInfo;
class ProviderInfo;
class ServiceInfo;
class Signature;
} // namespace android::content::pm

namespace android::os {
class Bundle;
} // namespace android::os

namespace java::util {
class List;
} // namespace java::util

} // namespace wrap

namespace wrap {
namespace android::content::pm {
/*!
 * Wrapper for android.content.pm.PackageItemInfo objects.
 */
class PackageItemInfo : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/PackageItemInfo";
    }

    /*!
     * Getter for the metaData field value
     *
     * Java prototype:
     * `public android.os.Bundle metaData;`
     *
     * JNI signature: Landroid/os/Bundle;
     *
     */
    os::Bundle getMetaData() const;

    /*!
     * Getter for the name field value
     *
     * Java prototype:
     * `public java.lang.String name;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getName() const;

    /*!
     * Getter for the packageName field value
     *
     * Java prototype:
     * `public java.lang.String packageName;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getPackageName() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::WrappedFieldId<os::Bundle> metaData;
        impl::FieldId<std::string> name;
        impl::FieldId<std::string> packageName;

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
 * Wrapper for android.content.pm.ComponentInfo objects.
 */
class ComponentInfo : public PackageItemInfo {
  public:
    using PackageItemInfo::PackageItemInfo;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/ComponentInfo";
    }

    /*!
     * Getter for the applicationInfo field value
     *
     * Java prototype:
     * `public android.content.pm.ApplicationInfo applicationInfo;`
     *
     * JNI signature: Landroid/content/pm/ApplicationInfo;
     *
     */
    ApplicationInfo getApplicationInfo() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::WrappedFieldId<ApplicationInfo> applicationInfo;

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
 * Wrapper for android.content.pm.ServiceInfo objects.
 */
class ServiceInfo : public ComponentInfo {
  public:
    using ComponentInfo::ComponentInfo;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/ServiceInfo";
    }

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {

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
 * Wrapper for android.content.pm.ProviderInfo objects.
 */
class ProviderInfo : public ComponentInfo {
  public:
    using ComponentInfo::ComponentInfo;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/ProviderInfo";
    }

    /*!
     * Getter for the authority field value
     *
     * Java prototype:
     * `public java.lang.String authority;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getAuthority() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::FieldId<std::string> authority;

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
 * Wrapper for android.content.pm.ApplicationInfo objects.
 */
class ApplicationInfo : public PackageItemInfo {
  public:
    using PackageItemInfo::PackageItemInfo;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/ApplicationInfo";
    }

    /*!
     * Getter for the nativeLibraryDir field value
     *
     * Java prototype:
     * `public java.lang.String nativeLibraryDir;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getNativeLibraryDir() const;

    /*!
     * Getter for the publicSourceDir field value
     *
     * Java prototype:
     * `public java.lang.String publicSourceDir;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getPublicSourceDir() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::FieldId<std::string> nativeLibraryDir;
        impl::FieldId<std::string> publicSourceDir;

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
 * Wrapper for android.content.pm.Signature objects.
 */
class Signature : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/Signature";
    }

    /*!
     * Wrapper for the toCharsString method
     *
     * Java prototype:
     * `public java.lang.String toCharsString();`
     *
     * JNI signature: ()Ljava/lang/String;
     *
     */
    std::string toCharsString();

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        jni::method_t toCharsString;

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
 * Wrapper for android.content.pm.PackageInfo objects.
 */
class PackageInfo : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/PackageInfo";
    }

    /*!
     * Getter for the applicationInfo field value
     *
     * Java prototype:
     * `public android.content.pm.ApplicationInfo applicationInfo;`
     *
     * JNI signature: Landroid/content/pm/ApplicationInfo;
     *
     */
    ApplicationInfo getApplicationInfo() const;

    /*!
     * Getter for the packageName field value
     *
     * Java prototype:
     * `public java.lang.String packageName;`
     *
     * JNI signature: Ljava/lang/String;
     *
     */
    std::string getPackageName() const;

    /*!
     * Getter for the signatures field value
     *
     * Java prototype:
     * `public android.content.pm.Signature[] signatures;`
     *
     * JNI signature: [Landroid/content/pm/Signature;
     *
     */
    jni::Array<jni::Object> getSignatures() const;

    /*!
     * Getter for the signatures field value, first element.
     *
     * Custom wrapper.
     *
     * Java prototype:
     * `public android.content.pm.Signature[] signatures;`
     *
     * JNI signature: [Landroid/content/pm/Signature;
     *
     */
    Signature getSignature() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::WrappedFieldId<ApplicationInfo> applicationInfo;
        impl::FieldId<std::string> packageName;
        jni::field_t signatures;

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
 * Wrapper for android.content.pm.ResolveInfo objects.
 */
class ResolveInfo : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/ResolveInfo";
    }

    /*!
     * Getter for the serviceInfo field value
     *
     * Java prototype:
     * `public android.content.pm.ServiceInfo serviceInfo;`
     *
     * JNI signature: Landroid/content/pm/ServiceInfo;
     *
     */
    ServiceInfo getServiceInfo() const;

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        impl::WrappedFieldId<ServiceInfo> serviceInfo;

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
 * Wrapper for android.content.pm.PackageManager objects.
 */
class PackageManager : public ObjectWrapperBase {
  public:
    using ObjectWrapperBase::ObjectWrapperBase;
    static constexpr const char *getTypeName() noexcept {
        return "android/content/pm/PackageManager";
    }

    /*!
     * Wrapper for the getPackageInfo method
     *
     * Java prototype:
     * `public abstract android.content.pm.PackageInfo
     * getPackageInfo(java.lang.String, int) throws
     * android.content.pm.PackageManager$NameNotFoundException;`
     *
     * JNI signature: (Ljava/lang/String;I)Landroid/content/pm/PackageInfo;
     *
     */
    PackageInfo getPackageInfo(std::string const &name, int32_t flags);

    /*!
     * Wrapper for the getPackageInfo method
     *
     * Java prototype:
     * `public abstract android.content.pm.PackageInfo
     * getPackageInfo(android.content.pm.VersionedPackage, int) throws
     * android.content.pm.PackageManager$NameNotFoundException;`
     *
     * JNI signature:
     * (Landroid/content/pm/VersionedPackage;I)Landroid/content/pm/PackageInfo;
     *
     */
    PackageInfo getPackageInfo(jni::Object const &versionedPackage,
                               int32_t intParam);

    /*!
     * Wrapper for the getApplicationInfo method
     *
     * Java prototype:
     * `public abstract android.content.pm.ApplicationInfo
     * getApplicationInfo(java.lang.String, int) throws
     * android.content.pm.PackageManager$NameNotFoundException;`
     *
     * JNI signature: (Ljava/lang/String;I)Landroid/content/pm/ApplicationInfo;
     *
     */
    ApplicationInfo getApplicationInfo(std::string const &packageName,
                                       int32_t flags);

    /*!
     * Wrapper for the queryIntentServices method
     *
     * Java prototype:
     * `public abstract java.util.List<android.content.pm.ResolveInfo>
     * queryIntentServices(android.content.Intent, int);`
     *
     * JNI signature: (Landroid/content/Intent;I)Ljava/util/List;
     *
     */
    java::util::List queryIntentServices(Intent const &intent, int32_t flags);

    /*!
     * Wrapper for the resolveContentProvider method
     *
     * Java prototype:
     * `public abstract android.content.pm.ProviderInfo
     * resolveContentProvider(java.lang.String, int)
     * `
     * JNI signature: (Ljava/lang/String;I)Landroid/content/pm/ProviderInfo;
     *
     */
    ProviderInfo resolveContentProvider(std::string const &authority, int32_t flags);

    /*!
     * Wrapper for the checkSignatures method
     *
     * Java prototype:
     * `public abstract int checkSignatures(java.lang.String, java.lang.String);`
     *
     * JNI signature: (Ljava/lang/String;Ljava/lang/String;)I
     *
     */
    int checkSignatures(std::string packageName1, std::string packageName2);

    enum {
        GET_META_DATA = 128,
        GET_SHARED_LIBRARY_FILES = 1024,
    };

    enum {
        SIGNATURE_MATCH = 0,
        SIGNATURE_NEITHER_SIGNED = 1,
        SIGNATURE_SECOND_NOT_SIGNED = -2,
        SIGNATURE_NO_MATCH = -3,
        SIGNATURE_UNKNOWN_PACKAGE = -4,
    };

    /*!
     * Class metadata
     */
    struct Meta : public MetaBaseDroppable {
        jni::method_t getPackageInfo;
        jni::method_t getPackageInfo2;
        jni::method_t getApplicationInfo;
        jni::method_t queryIntentServices;
        jni::method_t resolveContentProvider;
        jni::method_t checkSignatures;

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

}  // namespace android::content::pm
}  // namespace wrap
#include "android.content.pm.impl.h"

