// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>

#include "android.content.pm.h"

namespace wrap {
namespace android::content::pm {
PackageItemInfo::Meta::Meta()
    : MetaBaseDroppable(PackageItemInfo::getTypeName()),
      metaData(classRef(), "metaData"), name(classRef(), "name"),
      packageName(classRef(), "packageName") {
    MetaBaseDroppable::dropClassRef();
}
ComponentInfo::Meta::Meta()
    : MetaBaseDroppable(ComponentInfo::getTypeName()),
      applicationInfo(classRef(), "applicationInfo") {
    MetaBaseDroppable::dropClassRef();
}
ProviderInfo::Meta::Meta()
    : MetaBaseDroppable(ProviderInfo::getTypeName()),
      authority(classRef(), "authority") {
    MetaBaseDroppable::dropClassRef();
}
ServiceInfo::Meta::Meta() : MetaBaseDroppable(ServiceInfo::getTypeName()) {
    MetaBaseDroppable::dropClassRef();
}
ApplicationInfo::Meta::Meta()
    : MetaBaseDroppable(ApplicationInfo::getTypeName()),
      nativeLibraryDir(classRef(), "nativeLibraryDir"),
      publicSourceDir(classRef(), "publicSourceDir") {
    MetaBaseDroppable::dropClassRef();
}
Signature::Meta::Meta()
    : MetaBaseDroppable(Signature::getTypeName()),
      toCharsString(
          classRef().getMethod("toCharsString", "()Ljava/lang/String;")) {
    MetaBaseDroppable::dropClassRef();
}
PackageInfo::Meta::Meta()
    : MetaBaseDroppable(PackageInfo::getTypeName()),
      applicationInfo(classRef(), "applicationInfo"),
      packageName(classRef(), "packageName"),
      signatures(classRef().getField("signatures",
                                     "[Landroid/content/pm/Signature;")) {
    MetaBaseDroppable::dropClassRef();
}
ResolveInfo::Meta::Meta()
    : MetaBaseDroppable(ResolveInfo::getTypeName()),
      serviceInfo(classRef(), "serviceInfo") {
    MetaBaseDroppable::dropClassRef();
}
PackageManager::Meta::Meta()
    : MetaBaseDroppable(PackageManager::getTypeName()),
      getPackageInfo(classRef().getMethod(
          "getPackageInfo",
          "(Ljava/lang/String;I)Landroid/content/pm/PackageInfo;")),
      getPackageInfo2(classRef().getMethod(
          "getPackageInfo", "(Landroid/content/pm/VersionedPackage;I)Landroid/"
                            "content/pm/PackageInfo;")),
      getApplicationInfo(classRef().getMethod(
          "getApplicationInfo",
          "(Ljava/lang/String;I)Landroid/content/pm/ApplicationInfo;")),
      queryIntentServices(
          classRef().getMethod("queryIntentServices",
                               "(Landroid/content/Intent;I)Ljava/util/List;")),
      resolveContentProvider(
          classRef().getMethod("resolveContentProvider",
                               "(Ljava/lang/String;I)Landroid/content/pm/ProviderInfo;")),
      checkSignatures(
          classRef().getMethod("checkSignatures",
                               "(Ljava/lang/String;Ljava/lang/String;)I")) {
    MetaBaseDroppable::dropClassRef();
}
} // namespace android::content::pm
} // namespace wrap
