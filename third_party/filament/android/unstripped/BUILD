load("@build_bazel_rules_android//android:rules.bzl", "android_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

android_library(
    name = "filament_android",
    srcs = [],
    exports = [
        ":filament-android-jni",
        "@third_party//filament/android:filament_android_no_jni",
    ],
)
