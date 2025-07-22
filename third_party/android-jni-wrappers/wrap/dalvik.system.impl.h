// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Rylie Pavlik <rylie.pavlik@collabora.com>
// Inline implementations: do not include on its own!

#pragma once

#include "java.lang.h"
#include <string>

namespace wrap {
namespace dalvik::system {
inline DexClassLoader
DexClassLoader::construct(std::string const &searchPath,
                          std::string const &nativeSearchPath,
                          std::string const &stringParam,
                          java::lang::ClassLoader const &classLoader) {
    return DexClassLoader(Meta::data().clazz().newInstance(
        Meta::data().init, searchPath, nativeSearchPath, stringParam,
        classLoader.object()));
}

inline DexClassLoader
DexClassLoader::construct(std::string const &searchPath,
                          std::string const &nativeSearchPath,
                          jni::Object parentClassLoader) {
    return DexClassLoader{
        Meta::data().clazz().newInstance(Meta::data().init, searchPath, "",
                                         nativeSearchPath, parentClassLoader)};
}

} // namespace dalvik::system
} // namespace wrap
