load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "log",
    srcs = ["log.cc"],
    hdrs = [
        "log.h",
        "log_sink.h",
    ],
    alwayslink = 1,
)

cc_library(
    name = "jxr_log",
    srcs = ["jxr_log.cc"],
    hdrs = [
        "jxr_log_sink.h",
        "log.h",
    ],
    deps = [
        ":log",
        "//common:jni_utils",
    ],
    alwayslink = 1,
)
