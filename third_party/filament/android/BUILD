load("@build_bazel_rules_android//android:rules.bzl", "android_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "@third_party//filament:filament.bzl",
    "filament_copts",
    "filament_defines",
    "if_android",
    "if_android_min_sdk_at_least_26",
    "if_linux",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
        "no_layering_check",  # Incompatible with GL/EGL inclusion.
    ],
)

licenses(["notice"])

FILAMENT_HEADLESS_JNI_SRCS = [
    "common/CallbackUtils.cpp",
    "common/NioUtils.cpp",
    "filament-android/src/main/cpp/BufferObject.cpp",
    "filament-android/src/main/cpp/Camera.cpp",
    "filament-android/src/main/cpp/Colors.cpp",
    "filament-android/src/main/cpp/Engine.cpp",
    "filament-android/src/main/cpp/EntityManager.cpp",
    "filament-android/src/main/cpp/Fence.cpp",
    "filament-android/src/main/cpp/IndexBuffer.cpp",
    "filament-android/src/main/cpp/IndirectLight.cpp",
    "filament-android/src/main/cpp/LightManager.cpp",
    "filament-android/src/main/cpp/Material.cpp",
    "filament-android/src/main/cpp/MaterialInstance.cpp",
    "filament-android/src/main/cpp/MathUtils.cpp",
    "filament-android/src/main/cpp/MorphTargetBuffer.cpp",
    "filament-android/src/main/cpp/NativeSurface.cpp",
    "filament-android/src/main/cpp/RenderTarget.cpp",
    "filament-android/src/main/cpp/RenderableManager.cpp",
    "filament-android/src/main/cpp/Renderer.cpp",
    "filament-android/src/main/cpp/Scene.cpp",
    "filament-android/src/main/cpp/SkinningBuffer.cpp",
    "filament-android/src/main/cpp/SkyBox.cpp",
    "filament-android/src/main/cpp/ColorGrading.cpp",
    "filament-android/src/main/cpp/Stream.cpp",
    "filament-android/src/main/cpp/SurfaceOrientation.cpp",
    "filament-android/src/main/cpp/Texture.cpp",
    "filament-android/src/main/cpp/TextureSampler.cpp",
    "filament-android/src/main/cpp/ToneMapper.cpp",
    "filament-android/src/main/cpp/TransformManager.cpp",
    "filament-android/src/main/cpp/VertexBuffer.cpp",
    "filament-android/src/main/cpp/View.cpp",
]

FILAMENT_HEADLESS_JNI_HDRS = [
    "common/CallbackUtils.h",
    "common/JniUtils.h",
    "common/NioUtils.h",
]

FILAMENT_HEADLESS_JNI_TAGS = [
    "manual",
    "nobuilder",
    "notap",
]

FILAMENT_HEADLESS_JNI_DEPS = [
    "@third_party//filament/filament:filament_hdrs",
    "@third_party//filament/filament/backend:backend_hdrs",
    "@third_party//filament/libs:filabridge_hdrs",
    "@third_party//filament/libs:math_hdrs",
    "@third_party//filament/libs:geometry",
    "@third_party//filament/libs:utils_hdrs",
    "@third_party//filament/libs:generatePrefilterMipmap",
    "@third_party//filament/filament/backend:virtual_machine_env",
] + if_linux([
    "@bazel_tools//tools/jdk:jni",
])

cc_library(
    name = "filament_headless_jni_lib",
    srcs = FILAMENT_HEADLESS_JNI_SRCS + [
        "filament-android/src/main/cpp/Filament.cpp",
    ],
    hdrs = FILAMENT_HEADLESS_JNI_HDRS,
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["."],
    linkstatic = 1,
    tags = FILAMENT_HEADLESS_JNI_TAGS,
    deps = FILAMENT_HEADLESS_JNI_DEPS,
    alwayslink = 1,
)

# Same as :filament_headless_jni_lib, but excludes Filament.cpp,
# which defines a JNI_OnLoad function that can result in symbol
# collisions if an app is already defining that function. Users
# must make sure to initialize filament manually!
cc_library(
    name = "filament_headless_jni_lib_noload",
    srcs = FILAMENT_HEADLESS_JNI_SRCS,
    hdrs = FILAMENT_HEADLESS_JNI_HDRS,
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["."],
    linkstatic = 1,
    tags = FILAMENT_HEADLESS_JNI_TAGS,
    deps = FILAMENT_HEADLESS_JNI_DEPS,
    alwayslink = 1,
)

# A version of filament's JNI lib which is compatible with Linux _Desktop_ use (i.e. AWT)
# has never been used in google3
cc_library(
    name = "filament_desktop_jni_lib",
    srcs = [
        "filament-android/src/main/cpp/nativewindow/JAWTUtils.cpp",
        "filament-android/src/main/cpp/nativewindow/JAWTUtils.h",
        "filament-android/src/main/cpp/nativewindow/Linux.cpp",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    linkopts = [
        "-Wl,--gc-sections",
    ],
    linkstatic = 1,
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":filament_headless_jni_lib",
        "@bazel_tools//tools/jdk:jni",
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:backend_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

# A version of filament's Android JNI lib which is compatible with Linux Robolectric use
[cc_library(
    name = "filament_jni_lib%s" % flavor,
    srcs = [] + if_android([
        "filament-android/src/main/cpp/nativewindow/Android.cpp",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    linkopts = [
        "-Wl,--gc-sections",
    ] + if_android([
        "-ljnigraphics",
        "-llog",
        "-lGLESv3",
        "-lEGL",
        "-landroid",
    ]) + if_android_min_sdk_at_least_26([
        # The native window library is only available on API level 26 and above.
        "-lnativewindow",
    ]),
    linkstatic = 1,
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":filament_headless_jni_lib%s" % flavor,
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:backend_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
) for flavor in [
    "",
    "_noload",
]]

# A smaller version of the filament-utils library that only contains some
# texture-related functionality. Used by Pixel 2021 Live Wallpapers.
cc_library(
    name = "filament_texture_utils_jni_lib",
    srcs = [
        "common/NioUtils.cpp",
        "filament-utils-android/src/main/cpp/HDRLoader.cpp",
        "filament-utils-android/src/main/cpp/Utils.cpp",
    ],
    hdrs = [
        "common/NioUtils.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["."],
    linkopts = [
        "-Wl,--gc-sections",
    ] + if_android([
        "-llog",
    ]),
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":filament-android-jni",
        "@bazel_tools//tools/jdk:jni",
        "@third_party//filament/libs:image",
        "@third_party//filament/libs:imageio",
        "@third_party//filament/libs:ktxreader",
        "@third_party//filament/libs:utils",
    ],
    alwayslink = 1,
)

exports_files(
    srcs = [
        "jni.lds",
    ],
)

android_library(
    name = "filament_texture_utils_android_no_jni",
    srcs = [
        "filament-texture-utils-android/src/main/java/com/google/android/filament/utils/TextureUtils.kt",
        "filament-utils-android/src/main/java/com/google/android/filament/utils/HDRLoader.kt",
        "filament-utils-android/src/main/java/com/google/android/filament/utils/Half.kt",
        "filament-utils-android/src/main/java/com/google/android/filament/utils/KTX1Loader.kt",
        "filament-utils-android/src/main/java/com/google/android/filament/utils/Scalar.kt",
    ],
    custom_package = "com.google.android.filament.utils",
    javacopts = ["-XepAllErrorsAsWarnings"],
    manifest = "filament-utils-android/src/main/AndroidManifest.xml",
    deps = [
        ":filament_android_no_jni",
    ],
)

android_library(
    name = "filament_android_no_jni",
    srcs = [
        "filament-android/src/main/java/com/google/android/filament/AndroidPlatform.java",
        "filament-android/src/main/java/com/google/android/filament/AndroidPlatform21.java",
        "filament-android/src/main/java/com/google/android/filament/Asserts.java",
        "filament-android/src/main/java/com/google/android/filament/Box.java",
        "filament-android/src/main/java/com/google/android/filament/BufferObject.java",
        "filament-android/src/main/java/com/google/android/filament/Camera.java",
        "filament-android/src/main/java/com/google/android/filament/ColorGrading.java",
        "filament-android/src/main/java/com/google/android/filament/Colors.java",
        "filament-android/src/main/java/com/google/android/filament/Engine.java",
        "filament-android/src/main/java/com/google/android/filament/Entity.java",
        "filament-android/src/main/java/com/google/android/filament/EntityInstance.java",
        "filament-android/src/main/java/com/google/android/filament/EntityManager.java",
        "filament-android/src/main/java/com/google/android/filament/Fence.java",
        "filament-android/src/main/java/com/google/android/filament/Filament.java",
        "filament-android/src/main/java/com/google/android/filament/IndexBuffer.java",
        "filament-android/src/main/java/com/google/android/filament/IndirectLight.java",
        "filament-android/src/main/java/com/google/android/filament/LightManager.java",
        "filament-android/src/main/java/com/google/android/filament/Material.java",
        "filament-android/src/main/java/com/google/android/filament/MaterialInstance.java",
        "filament-android/src/main/java/com/google/android/filament/MathUtils.java",
        "filament-android/src/main/java/com/google/android/filament/MorphTargetBuffer.java",
        "filament-android/src/main/java/com/google/android/filament/NativeSurface.java",
        "filament-android/src/main/java/com/google/android/filament/NioUtils.java",
        "filament-android/src/main/java/com/google/android/filament/Platform.java",
        "filament-android/src/main/java/com/google/android/filament/RenderTarget.java",
        "filament-android/src/main/java/com/google/android/filament/RenderableManager.java",
        "filament-android/src/main/java/com/google/android/filament/Renderer.java",
        "filament-android/src/main/java/com/google/android/filament/Scene.java",
        "filament-android/src/main/java/com/google/android/filament/SkinningBuffer.java",
        "filament-android/src/main/java/com/google/android/filament/Skybox.java",
        "filament-android/src/main/java/com/google/android/filament/Stream.java",
        "filament-android/src/main/java/com/google/android/filament/SurfaceOrientation.java",
        "filament-android/src/main/java/com/google/android/filament/SwapChain.java",
        "filament-android/src/main/java/com/google/android/filament/SwapChainFlags.java",
        "filament-android/src/main/java/com/google/android/filament/Texture.java",
        "filament-android/src/main/java/com/google/android/filament/TextureSampler.java",
        "filament-android/src/main/java/com/google/android/filament/ToneMapper.java",
        "filament-android/src/main/java/com/google/android/filament/TransformManager.java",
        "filament-android/src/main/java/com/google/android/filament/VertexBuffer.java",
        "filament-android/src/main/java/com/google/android/filament/View.java",
        "filament-android/src/main/java/com/google/android/filament/Viewport.java",
        "filament-android/src/main/java/com/google/android/filament/android/DisplayHelper.java",
        "filament-android/src/main/java/com/google/android/filament/android/TextureHelper.java",
        "filament-android/src/main/java/com/google/android/filament/android/UiHelper.java",
        "filament-android/src/main/java/com/google/android/filament/proguard/UsedByNative.java",
        "filament-android/src/main/java/com/google/android/filament/proguard/UsedByReflection.java",
    ],
    custom_package = "com.google.android.filament",
    javacopts = ["-XepAllErrorsAsWarnings"],
    manifest = "filament-android/src/main/AndroidManifest.xml",
    proguard_specs = [
        ":filament-android/proguard-rules.pro",
    ],
    resource_files = ["filament-android/src/main/res/values/strings.xml"],
    deps = [
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_appcompat_appcompat",
    ],
)

android_library(
    name = "ui_helper",
    srcs = [
        "filament-android/src/main/java/com/google/android/filament/SwapChainFlags.java",
        "filament-android/src/main/java/com/google/android/filament/android/UiHelper.java",
    ],
    custom_package = "com.google.android.filament",
    javacopts = ["-XepAllErrorsAsWarnings"],
    manifest = "filament-android/src/main/AndroidManifest.xml",
    deps = [
        "@maven//:androidx_annotation_annotation",
    ],
)

android_library(
    name = "filament_android",
    srcs = [],
    exports = [
        ":filament-android-jni",
        ":filament_android_no_jni",
    ],
)

android_library(
    name = "filament_texture_utils_android",
    srcs = [],
    exports = [
        ":filament-texture-utils-android-jni",
        ":filament_texture_utils_android_no_jni",
    ],
)

alias(
    name = "filament-android",
    actual = ":filament_android",
)
