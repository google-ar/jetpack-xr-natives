load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load(
    "@third_party//filament:filament.bzl",
    "copy_files",
    "filament_copts",
    "filament_defines",
    "if_android",
    "if_android_perfetto",
    "if_apple",
    "if_linux",
    "if_linux_aarch64",
    "if_linux_or_android",
    "if_linux_x86_64",
    "if_macos",
    "if_macos_aarch64",
    "if_macos_x86_64",
    "if_vulkan",
    "if_wasm",
    "if_windows",
    "uberarchive_args",
)

licenses(["notice"])

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

exports_files([
    "gltfio/materials/base.mat.in",
    "gltfio/materials/base.spec.in",
])

cc_library(
    name = "math_hdrs",
    srcs = ["math/src/dummy.cpp"],
    hdrs = glob(["math/include/**"]),
    copts = filament_copts(),
    defines = filament_defines(),
    alwayslink = 1,
)

cc_library(
    name = "math",
    copts = filament_copts(),
    defines = filament_defines(),
    linkstatic = 1,
    deps = [":math_hdrs"],
    alwayslink = 1,
)

cc_library(
    name = "math_io",
    srcs = glob(["mathio/src/**"]),
    hdrs = glob(["mathio/include/**"]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["mathio/include"],
    linkstatic = 1,
    deps = [":math_hdrs"],
    alwayslink = 1,
)

cc_library(
    name = "utils_hdrs",
    hdrs = [
        "utils/include/utils/Allocator.h",
        "utils/include/utils/AsyncJobQueue.h",
        "utils/include/utils/BinaryTreeArray.h",
        "utils/include/utils/BitmaskEnum.h",
        "utils/include/utils/CString.h",
        "utils/include/utils/CallStack.h",
        "utils/include/utils/Condition.h",
        "utils/include/utils/CountDownLatch.h",
        "utils/include/utils/Entity.h",
        "utils/include/utils/EntityInstance.h",
        "utils/include/utils/EntityManager.h",
        "utils/include/utils/FixedCapacityVector.h",
        "utils/include/utils/FixedCircularBuffer.h",
        "utils/include/utils/Hash.h",
        "utils/include/utils/ImmutableCString.h",
        "utils/include/utils/InternPool.h",
        "utils/include/utils/Invocable.h",
        "utils/include/utils/JobSystem.h",
        "utils/include/utils/Log.h",
        "utils/include/utils/Logger.h",
        "utils/include/utils/MonotonicRingMap.h",
        "utils/include/utils/Mutex.h",
        "utils/include/utils/NameComponentManager.h",
        "utils/include/utils/Panic.h",
        "utils/include/utils/Path.h",
        "utils/include/utils/PrivateImplementation.h",
        "utils/include/utils/PrivateImplementation-impl.h",
        "utils/include/utils/Profiler.h",
        "utils/include/utils/QuadTree.h",
        "utils/include/utils/Range.h",
        "utils/include/utils/RangeMap.h",
        "utils/include/utils/RefCountedMap.h",
        "utils/include/utils/SingleInstanceComponentManager.h",
        "utils/include/utils/Slice.h",
        "utils/include/utils/StaticString.h",
        "utils/include/utils/Status.h",
        "utils/include/utils/StructureOfArrays.h",
        "utils/include/utils/ThermalManager.h",
        "utils/include/utils/ThreadUtils.h",
        "utils/include/utils/WorkStealingDequeue.h",
        "utils/include/utils/algorithm.h",
        "utils/include/utils/android/PerformanceHintManager.h",
        "utils/include/utils/api_level.h",
        "utils/include/utils/architecture.h",
        "utils/include/utils/ashmem.h",
        "utils/include/utils/bitset.h",
        "utils/include/utils/compiler.h",
        "utils/include/utils/compressed_pair.h",
        "utils/include/utils/debug.h",
        "utils/include/utils/memalign.h",
        "utils/include/utils/ostream.h",
        "utils/include/utils/sstream.h",
        "utils/include/utils/string.h",
        "utils/include/utils/trap.h",
        "utils/include/utils/unwindows.h",
        "utils/include/utils/vector.h",
    ] + if_windows([
        "utils/include/utils/win32/stdtypes.h",
    ]) + if_android(
        [
            "utils/include/utils/android/ThermalManager.h",
            "utils/include/utils/linux/Mutex.h",
            "utils/include/utils/linux/Condition.h",
        ],
        [
            "utils/include/utils/generic/ThermalManager.h",
            "utils/include/utils/generic/Mutex.h",
            "utils/include/utils/generic/Condition.h",
        ],
    ),
    copts = filament_copts(),
    defines = filament_defines(),
    textual_hdrs = [
        "utils/include/utils/Systrace.h",
        "utils/include/utils/Zip2Iterator.h",
    ] + if_android([
        "utils/include/utils/android/Systrace.h",
    ]) + if_apple([
        "utils/include/utils/darwin/Systrace.h",
    ]),
    deps = [
        "@com_google_absl//absl/base:log_severity",
        "@com_google_impress//core/common:log",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "utils_private_hdrs",
    hdrs = [
        "utils/include/private/utils/Tracing.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["utils/include"],
    textual_hdrs = [] + if_android_perfetto([
        "utils/include/private/utils/android/Tracing.h",
    ]) + if_apple([
        "utils/include/private/utils/darwin/Tracing.h",
    ]),
    deps = [
        ":utils_hdrs",
    ] + if_android_perfetto([
        "@perfetto//:libperfetto_client_experimental",
    ]),
    alwayslink = 1,
)

# cc_library(
#     name = "utils_private",
#     srcs = [
#     ] + if_android([
#         "utils/src/android/Tracing.cpp",
#     ]) + if_apple([
#         "utils/src/darwin/Tracing.cpp",
#     ]),
#     copts = filament_copts(),
#     defines = filament_defines(),
#     linkstatic = 1,
#     deps = [
#         ":utils_private_hdrs",
#         "@perfetto//:libperfetto_client_experimental",
#     ],
# )

objc_library(
    name = "utils_path",
    srcs = ["utils/src/darwin/Path.mm"],
    hdrs = ["utils/include/utils/Path.h"],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["utils/include"],
    textual_hdrs = ["utils/include/utils/compiler.h"],
)

cc_library(
    name = "utils",
    srcs = [
        "utils/include/utils/CyclicBarrier.h",
        "utils/include/utils/NameComponentManager.h",
        "utils/src/Allocator.cpp",
        "utils/src/AsyncJobQueue.cpp",
        "utils/src/CString.cpp",
        "utils/src/CallStack.cpp",
        "utils/src/CountDownLatch.cpp",
        "utils/src/CyclicBarrier.cpp",
        "utils/src/EntityManager.cpp",
        "utils/src/EntityManagerImpl.h",
        "utils/src/FixedCapacityVectorBase.cpp",
        "utils/src/ImmutableCString.cpp",
        "utils/src/Invocable.cpp",
        "utils/src/JobSystem.cpp",
        "utils/src/Log.cpp",
        "utils/src/NameComponentManager.cpp",
        "utils/src/Panic.cpp",
        "utils/src/Path.cpp",
        "utils/src/Profiler.cpp",
        "utils/src/Status.cpp",
        "utils/src/ThreadUtils.cpp",
        "utils/src/api_level.cpp",
        "utils/src/architecture.cpp",
        "utils/src/ashmem.cpp",
        "utils/src/debug.cpp",
        "utils/src/linux/Path.cpp",
        "utils/src/ostream.cpp",
        "utils/src/ostream_.h",
        "utils/src/sstream.cpp",
        "utils/src/string.cpp",
    ] + if_linux_or_android([
        "utils/src/linux/futex.h",
    ]) + if_android([
        "utils/src/android/PerformanceHintManager.cpp",
        "utils/src/android/Systrace.cpp",
        "utils/src/android/ThermalManager.cpp",
        "utils/src/linux/Condition.cpp",
        "utils/src/linux/Mutex.cpp",
    ]) + if_windows([
        "utils/src/win32/Path.cpp",
    ]) + if_apple([
        "utils/src/darwin/Systrace.cpp",
    ]) + if_wasm([
        "utils/src/web/Path.cpp",
    ]) + if_android_perfetto([
        "utils/src/android/Tracing.cpp",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    linkopts = if_linux_or_android([
        "-ldl",
    ]),
    linkstatic = 1,
    deps = [
        ":utils_hdrs",
        ":utils_private_hdrs",
        "@robin_map",
    ] + if_apple([":utils_path"]) + if_android_perfetto([
        "@perfetto//:libperfetto_client_experimental",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "filabridge_hdrs",
    hdrs = glob([
        "filabridge/include/**/*.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "filabridge/include",
    ],
    linkstatic = 1,
    deps = [
        ":math_hdrs",
        ":utils_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

# Filament tool/runtime common code
cc_library(
    name = "filabridge",
    srcs = glob([
        "filabridge/src/**/*.cpp",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    linkstatic = 1,
    textual_hdrs = glob([
        "filabridge/src/**/*.h",
    ]),
    deps = [
        ":filabridge_hdrs",
        ":utils_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

cc_library(
    name = "image_hdrs",
    hdrs = [
        "image/include/image/ColorTransform.h",
        "image/include/image/ImageOps.h",
        "image/include/image/ImageSampler.h",
        "image/include/image/Ktx1Bundle.h",
        "image/include/image/LinearImage.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "image/include",
    ],
    linkstatic = 1,
    deps = [
        ":filabridge_hdrs",
        ":math_hdrs",
        ":utils_hdrs",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "image",
    srcs = [
        "image/src/ImageOps.cpp",
        "image/src/ImageSampler.cpp",
        "image/src/Ktx1Bundle.cpp",
        "image/src/LinearImage.cpp",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    linkstatic = 1,
    deps = [
        ":image_hdrs",
        ":math_hdrs",
        ":utils",
        ":utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ktxreader",
    srcs = [
        "ktxreader/src/Ktx1Reader.cpp",
        "ktxreader/src/Ktx2Reader.cpp",
    ],
    hdrs = [
        "ktxreader/include/ktxreader/Ktx1Reader.h",
        "ktxreader/include/ktxreader/Ktx2Reader.h",
    ],
    # (broken link): library depended by a linkstatic=0 cc_library cannot use
    # -fvisibility=hidden.
    copts = filament_copts() + ["-fvisibility=default"],
    defines = filament_defines(),
    includes = [
        "ktxreader/include",
    ],
    linkstatic = 1,
    deps = [
        ":image_hdrs",
        ":utils_hdrs",
        "@basis_universal",
        "@basis_universal//:basisu_headers_lib",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ktxreader_hdrs",
    hdrs = [
        "ktxreader/include/ktxreader/Ktx1Reader.h",
        "ktxreader/include/ktxreader/Ktx2Reader.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "ktxreader/include",
    ],
    deps = [
        ":image_hdrs",
        ":utils_hdrs",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

# Filament image serialization library.
cc_library(
    name = "imageio",
    srcs = [
        "imageio/src/HDRDecoder.cpp",
        "imageio/src/ImageDecoder.cpp",
        "imageio/src/ImageDiffer.cpp",
        "imageio/src/ImageEncoder.cpp",
    ],
    hdrs = [
        "imageio/include/imageio/HDRDecoder.h",
        "imageio/include/imageio/ImageDecoder.h",
        "imageio/include/imageio/ImageDiffer.h",
        "imageio/include/imageio/ImageEncoder.h",
    ],
    copts = filament_copts() + [
        "-fexceptions",
    ],
    defines = filament_defines(),
    includes = [
        "imageio/include",
    ],
    linkopts = select({
        "@platforms//os:linux": [],
        "@platforms//os:android": [],
        "//conditions:default": [],
    }),
    textual_hdrs = glob([
        "imageio/src/**/*.h",
    ]),
    deps = [
        ":filabridge_hdrs",
        ":image",
        ":image_hdrs",
        ":math_hdrs",
        ":utils_hdrs",
        "@png",
        "@tinyexr",
    ],
    alwayslink = 1,
)

cc_library(
    name = "imageio_basis_encoder",
    srcs = [
        "imageio/src/BasisEncoder.cpp",
    ],
    hdrs = [
        "imageio/include/imageio/BasisEncoder.h",
    ],
    copts = filament_copts() + [
        "-fexceptions",
    ],
    defines = filament_defines(),
    includes = [
        "imageio/include",
    ],
    linkopts = select({
        "@platforms//os:linux": [],
        "@platforms//os:android": [],
        "//conditions:default": [],
    }),
    textual_hdrs = glob([
        "imageio/src/**/*.h",
    ]),
    deps = [
        ":image_hdrs",
        ":utils_hdrs",
        "@basis_universal",
        "@basis_universal//:basisu_headers_lib",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ibl_hdrs",
    hdrs = [
        "ibl/include/ibl/Cubemap.h",
        "ibl/include/ibl/CubemapIBL.h",
        "ibl/include/ibl/CubemapSH.h",
        "ibl/include/ibl/CubemapUtils.h",
        "ibl/include/ibl/Image.h",
        "ibl/include/ibl/utilities.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "ibl/include",
    ],
    deps = [
        ":math_hdrs",
        ":utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ibl",
    srcs = [
        "ibl/src/Cubemap.cpp",
        "ibl/src/CubemapIBL.cpp",
        "ibl/src/CubemapSH.cpp",
        "ibl/src/CubemapUtils.cpp",
        "ibl/src/CubemapUtilsImpl.h",
        "ibl/src/Image.cpp",
    ],
    copts = filament_copts() + select({
        # TODO: Work around clang hanging in the SLP
        # vectorizer.  The impact should be small as macos x86_64 is
        # being phased out ((broken link)), and this dependency
        # currently only affects one package
        # (//third_party/arcore/ar).
        "@third_party//filament:macos_x86_64": [
            "-fno-slp-vectorize",
        ],
        "//conditions:default": [],
    }),
    defines = filament_defines(),
    linkstatic = 1,
    deps = [
        ":ibl_hdrs",
        ":math_hdrs",
        ":utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "geometry",
    srcs = [
        "geometry/src/MikktspaceImpl.cpp",
        "geometry/src/MikktspaceImpl.h",
        "geometry/src/SurfaceOrientation.cpp",
        "geometry/src/TangentSpaceMesh.cpp",
        "geometry/src/TangentSpaceMeshInternal.h",
        "geometry/src/Transcoder.cpp",
    ],
    hdrs = [
        "geometry/include/geometry/SurfaceOrientation.h",
        "geometry/include/geometry/TangentSpaceMesh.h",
        "geometry/include/geometry/Transcoder.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "geometry/include",
    ],
    linkstatic = 1,
    deps = [
        ":math_hdrs",
        ":utils",
        ":utils_hdrs",
        "@meshoptimizer",
        "@mikktspace",
    ],
    alwayslink = 1,
)

# This header cannot be independently compiled, so it needs to be a textual_hdrs.
cc_library(
    name = "filamat_builtinResource",
    linkstatic = 1,
    textual_hdrs = [
        "filamat/src/sca/builtinResource.h",
    ],
    alwayslink = 1,
)

# Filament data serialization library.
[
    cc_library(
        name = lib_name,
        srcs = glob([
            "filaflat/src/**/*.cpp",
            "filaflat/src/**/*.h",
        ]),
        hdrs = glob([
            "filaflat/include/**/*.h",
        ]),
        copts = filament_copts(),
        defines = filament_defines() + defines,
        includes = [
            "filaflat/include",
        ],
        linkstatic = 1,
        deps = [
            ":filabridge_hdrs",
            ":utils",
            ":utils_hdrs",
            "@robin_map",
            "@third_party//filament/filament/backend:driver_enums",
        ] + deps,
        alwayslink = 1,
    )
    for lib_name, defines, deps in [
        (
            "filaflat",
            [],
            [],
        ),
        (
            "filaflat_vulkan",
            ["FILAMENT_DRIVER_SUPPORTS_VULKAN"],
            ["@smol_v//:smol-v"],
        ),
    ]
]

# Filament material builder library.
cc_library(
    name = "filamat",
    srcs = glob([
        "filamat/src/**/*.cpp",
    ]),
    hdrs = glob([
        "filamat/include/**/*.h",
    ]),
    copts = filament_copts() + [
        "-Ithird_party/glslang/glslang/Include",
        "-Ithird_party/glslang/glslang/MachineIndependent",
        "-Ithird_party/glslang/glslang/Public",
        "-Ithird_party/glslang/SPIRV",
        "-Wno-uninitialized",
        "-fexceptions",
    ],
    defines = filament_defines(),
    includes = [
        "filamat/include",
        "filamat/src",  # TODO this is needed by matedit for eiff/ headers
    ],
    linkstatic = 1,
    textual_hdrs = glob([
        "filamat/src/**/*.h",
    ]),
    deps = [
        ":filabridge",
        ":filabridge_hdrs",
        ":math_hdrs",
        ":utils",
        ":utils_hdrs",
        "@glslang",
        "@glslang//:SPIRV",
        "@smol_v//:smol-v",
        "@spirv_cross//:spirv_cross_lib",
        "@spirv_tools",
        "@spirv_tools//:spirv_tools_opt",
        "@third_party//filament/filament:generated_shaders",
        "@third_party//filament/filament/backend:backend_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

cc_library(
    name = "matp",
    srcs = glob([
        "filament-matp/src/**/*.cpp",
        "filament-matp/src/**/*.h",
    ]),
    hdrs = glob([
        "filament-matp/include/filament-matp/**/*.h",
    ]),
    linkstatic = 1,
    strip_include_prefix = "filament-matp/include",
    deps = [
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:filamat",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generatePrefilterMipmap",
    srcs = [
        "generatePrefilterMipmap/src/generatePrefilterMipmap.cpp",
    ],
    hdrs = [
        "generatePrefilterMipmap/include/filament-generatePrefilterMipmap/generatePrefilterMipmap.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "generatePrefilterMipmap/include",
    ],
    linkstatic = 1,
    deps = [
        ":math_hdrs",
        ":utils_hdrs",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:ibl_hdrs",
    ],
    alwayslink = 1,
)

# Filament GL Extension wrangling library.
cc_library(
    name = "bluegl_hdrs",
    srcs = [],
    hdrs = [
        "bluegl/include/bluegl/BlueGL.h",
        "bluegl/include/bluegl/BlueGLDefines.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "bluegl/include",
    ],
    textual_hdrs = [
        "bluegl/src/private_BlueGL.h",
        "bluegl/include/GL/glcorearb.h",
        "bluegl/include/GL/glext.h",
        "bluegl/include/GL/wglext.h",
    ],
    alwayslink = 1,
)

[
    cc_library(
        name = lib_name,
        srcs = [
            "bluegl/src/BlueGL.cpp",
        ] + if_linux_aarch64([
            "bluegl/src/BlueGLCoreLinuxAArch64Impl.S",
        ]) + if_linux_x86_64([
            "bluegl/src/BlueGLCoreLinuxImpl.S",
        ]) + if_windows([
            "bluegl/src/BlueGLWindows.cpp",
            "bluegl/src/BlueGLCoreWindowsImpl.S",
        ]) + if_macos_aarch64([
            "bluegl/src/BlueGLDarwin.cpp",
            "bluegl/src/BlueGLCoreDarwinAArch64Impl.S",
        ]) + if_macos_x86_64([
            "bluegl/src/BlueGLDarwin.cpp",
            "bluegl/src/BlueGLCoreDarwinImpl.S",
        ]) + extra_srcs,
        copts = filament_copts() + select({
            "//conditions:default": [
                "-Wa,--noexecstack",
            ],
        }),
        defines = filament_defines(),
        includes = [
            "bluegl/include",
        ],
        linkstatic = 1,
        deps = [
            ":bluegl_hdrs",
        ] + extra_deps,
        alwayslink = 1,
    )
    for lib_name, extra_srcs, extra_deps in [
        (
            "bluegl",
            if_linux([
                "bluegl/src/BlueGLLinux.cpp",
            ]),
            [],
        ),
        (
            "bluegl_osmesa",
            ["bluegl/src/BlueGLOSMesa.cpp"],
            ["//third_party/mesa:OSMesa_header_thirdparty"],
        ),
    ]
]

bluevk_textual_base_hdrs = [
    "bluevk/include/vk_video/vulkan_video_codec_h264std.h",
    "bluevk/include/vk_video/vulkan_video_codec_h264std_decode.h",
    "bluevk/include/vk_video/vulkan_video_codec_h264std_encode.h",
    "bluevk/include/vk_video/vulkan_video_codec_h265std.h",
    "bluevk/include/vk_video/vulkan_video_codec_h265std_decode.h",
    "bluevk/include/vk_video/vulkan_video_codec_h265std_encode.h",
    "bluevk/include/vk_video/vulkan_video_codecs_common.h",
    "bluevk/include/vulkan/vk_platform.h",
    "bluevk/include/vulkan/vulkan.h",
    "bluevk/include/vulkan/vulkan_ggp.h",
    "bluevk/include/vulkan/vulkan_android.h",
    "bluevk/include/vulkan/vulkan_beta.h",
    "bluevk/include/vulkan/vulkan_core.h",
    "bluevk/include/vulkan/vulkan_vi.h",
    "bluevk/include/vulkan/vulkan_wayland.h",
    "bluevk/include/vulkan/vulkan_xcb.h",
    "bluevk/include/vulkan/vulkan_xlib.h",
    "bluevk/include/vulkan/vulkan_xlib_xrandr.h",
]

cc_library(
    name = "bluevk_hdrs",
    hdrs = [
        "bluevk/include/bluevk/BlueVK.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "bluevk/include",
    ],
    textual_hdrs = bluevk_textual_base_hdrs + if_macos([
        "bluevk/include/vulkan/vulkan_macos.h",
    ]) + if_windows([
        "bluevk/include/vulkan/vulkan_win32.h",
    ]),
    deps = [
        ":utils_hdrs",
    ],
    alwayslink = 1,
)

# BlueVK library, Filament Vulkan wrangling library.
[
    cc_library(
        name = lib_name,
        srcs = [
            "bluevk/src/BlueVK.cpp",
        ] + if_linux_or_android([
            "bluevk/src/BlueVKLinuxAndroid.cpp",
        ]) + if_windows([
            "bluevk/src/BlueVKWindows.cpp",
        ]) + if_macos([
            "bluevk/src/BlueVKDarwin.cpp",
        ]),
        hdrs = [
            "bluevk/include/bluevk/BlueVK.h",
        ],
        copts = filament_copts() + select({
            "//conditions:default": [
                "-Wa,--noexecstack",
            ],
        }),
        defines = filament_defines() + extra_defines,
        includes = [
            "bluevk/include",
        ],
        linkopts = linkopts,
        linkstatic = 1,
        textual_hdrs = bluevk_textual_base_hdrs + if_macos([
            "bluevk/include/vulkan/vulkan_macos.h",
        ]) + if_windows([
            "bluevk/include/vulkan/vulkan_win32.h",
        ]),
        deps = [
            ":utils_hdrs",
        ],
        alwayslink = 1,
    )
    for lib_name, extra_defines, linkopts in [
        (
            "bluevk",
            if_linux(["FILAMENT_SUPPORTS_XLIB"]),
            [],
        ),
        (
            "bluevk_swiftshader",
            ["FILAMENT_VKLIBRARY_PATH=\\\"libvk_swiftshader.so\\\""],
            [],
        ),
        (
            "bluevk_static",
            ["FILAMENT_VKLIBRARY_PATH=nullptr"],
            ["-Wl,--export-dynamic-symbol=vkGetInstanceProcAddr"],
        ),
        ("bluevk_headless", [], []),
    ]
]

# This generates uiBlit and uiBlitExternal materials for all platforms. But
# uiBlitExternal should be optimized out on non-Android devices. Hopefully?
genrule(
    name = "generate_filagui_materials",
    srcs = [
        "filagui/src/materials/uiBlit.mat",
        "filagui/src/materials/uiBlitExternal.mat",
    ],
    outs = [
        "generated/resources/filagui_resources.h",
        "generated/resources/filagui_resources.c",
        "generated/resources/uiBlit.filamat",
        "generated/resources/uiBlitExternal.filamat",
    ],
    cmd = "MATC=$(location @third_party//filament/tools:matc) && " +
          "RESGEN=$(location @third_party//filament/tools:resgen) && " +
          "DEPLOY=`dirname $(location generated/resources/uiBlit.filamat)` && " +
          "$$MATC -a all -p all -o $$DEPLOY/uiBlit.filamat $(location filagui/src/materials/uiBlit.mat) && " +
          "$$MATC -a all -p mobile -o $$DEPLOY/uiBlitExternal.filamat $(location filagui/src/materials/uiBlitExternal.mat) && " +
          "$$RESGEN -x $$DEPLOY -c -p filagui_resources $$DEPLOY/uiBlit.filamat $$DEPLOY/uiBlitExternal.filamat",
    tools = [
        "@third_party//filament/tools:matc",
        "@third_party//filament/tools:resgen",
    ],
)

cc_library(
    name = "generated_filagui_materials",
    srcs = [
        "generated/resources/filagui_resources.c",
    ],
    hdrs = [],
    copts = filament_copts(),
    data = [],
    includes = ["generated"],
    linkstatic = 1,
    textual_hdrs = [
        "generated/resources/filagui_resources.h",
    ],
    alwayslink = 1,
)

# Filament GUI helper library.
cc_library(
    name = "filagui",
    srcs = glob([
        "filagui/src/**/*.cpp",
        "filagui/src/**/*.h",
    ]),
    hdrs = glob([
        "filagui/include/**/*.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "filagui/include",
    ],
    linkstatic = 1,
    textual_hdrs = ["generated/resources/filagui_resources.h"],
    deps = [
        ":filabridge",
        ":filamat",
        ":generated_filagui_materials",
        ":math_hdrs",
        ":utils_hdrs",
        "@dear_imgui",
        "@robin_map",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

# Filament ubershader archives library.
cc_library(
    name = "uberz",
    srcs = glob([
        "uberz/src/*.cpp",
    ]),
    hdrs = glob([
        "uberz/include/uberz/*.h",
    ]),
    copts = filament_copts() + [
        "-Ithird_party/glslang/glslang/Include",
        "-Ithird_party/glslang/glslang/MachineIndependent",
        "-Ithird_party/glslang/glslang/Public",
        "-Ithird_party/glslang/SPIRV",
        "-Wno-uninitialized",
    ],
    defines = filament_defines(),
    includes = [
        "uberz/include",
    ],
    linkstatic = 1,
    deps = [
        ":filabridge",
        ":filabridge_hdrs",
        ":utils",
        ":utils_hdrs",
        "@robin_map",
        "@zstdlib",
    ],
    alwayslink = 1,
)

# TODO: this step can be removed after adding an explicit input feature to uberz
copy_files(
    name = "uberarchive_core_inputs",
    srcs = [
        ":gltfio/materials/base.mat.in",
        ":gltfio/materials/base.spec.in",
    ],
    outs = [
        "generated/resources/core.mat",
        "generated/resources/core.spec",
    ],
)

genrule(
    name = "uberarchive_core",
    srcs = [
        "generated/resources/core.mat",
        "generated/resources/core.spec",
    ],
    outs = ["generated/resources/core.uberz"],
    cmd = "MATC=$(location @third_party//filament/tools:matc) && " +
          "UBERZ=$(location @third_party//filament/tools:uberz) && " +
          "DEPLOY=`dirname $(location generated/resources/core.uberz)` && " +
          "$$MATC " + uberarchive_args("lit", "opaque") + " -a all -p all -o $$DEPLOY/core.filamat $$DEPLOY/core.mat && " +
          "$$UBERZ " + uberarchive_args("lit", "opaque") + " -o $$DEPLOY/core.uberz $$DEPLOY/core",
    tools = [
        "@third_party//filament/tools:matc",
        "@third_party//filament/tools:uberz",
    ],
)

# Filament glTF loader library.
cc_library(
    name = "gltfio",
    srcs = glob(
        [
            "gltfio/src/*.cpp",
            "gltfio/src/*.h",
            "gltfio/src/extended/*.cpp",
            "gltfio/src/extended/*.h",
        ],
        exclude = [
            "gltfio/src/Ktx2Provider.cpp",
        ],
    ),
    hdrs = glob([
        "gltfio/include/gltfio/*.h",
    ]),
    defines = filament_defines() + [
        "GLTFIO_DRACO_SUPPORTED=0",
        "FILAMENT_WASM_THREADS",
    ],
    includes = [
        "gltfio/include",
    ],
    linkstatic = 1,
    deps = [
        ":filabridge_hdrs",
        ":filamat",
        ":geometry",
        ":math_hdrs",
        ":uberz",
        ":utils_hdrs",
        ":utils_private_hdrs",
        "//third_party/cgltf",
        "@meshoptimizer",
        "@robin_map",
        "@stblib//:stb_image",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@zstdlib",
    ],
    alwayslink = 1,
)

genrule(
    name = "generate_matdbg_resources",
    srcs = [
        "matdbg/web/index.html",
        "matdbg/web/app.js",
        "matdbg/web/api.js",
    ],
    outs = [
        "generated/resources/matdbg_resources.h",
        "generated/resources/matdbg_resources.c",
    ],
    cmd = "RESGEN=$(location @third_party//filament/tools:resgen) && " +
          "DEPLOY=`dirname $(location generated/resources/matdbg_resources.h)` && " +
          "WEB_DIR=`dirname $(location matdbg/web/index.html)` && " +
          "$$RESGEN -t -x $$DEPLOY -c -p matdbg_resources " +
          "$$WEB_DIR/index.html $$WEB_DIR/app.js $$WEB_DIR/api.js",
    tools = [
        "@third_party//filament/tools:resgen",
    ],
)

cc_library(
    name = "generated_matdbg_resources",
    srcs = [
        "generated/resources/matdbg_resources.c",
    ],
    hdrs = [],
    copts = filament_copts(),
    data = [],
    includes = ["generated"],
    linkstatic = 1,
    textual_hdrs = [
        "generated/resources/matdbg_resources.h",
    ],
    deps = [":generate_matdbg_resources"],
    alwayslink = 1,
)

cc_library(
    name = "matdbg",
    srcs = [
        "matdbg/src/ApiHandler.cpp",
        "matdbg/src/CommonWriter.cpp",
        "matdbg/src/DebugServer.cpp",
        "matdbg/src/JsonWriter.cpp",
        "matdbg/src/ShaderExtractor.cpp",
        "matdbg/src/ShaderInfo.cpp",
        "matdbg/src/ShaderReplacer.cpp",
        "matdbg/src/SourceFormatter.cpp",
        "matdbg/src/TextWriter.cpp",
    ],
    hdrs = [
        "matdbg/include/matdbg/DebugServer.h",
        "matdbg/include/matdbg/JsonWriter.h",
        "matdbg/include/matdbg/ShaderExtractor.h",
        "matdbg/include/matdbg/ShaderInfo.h",
        "matdbg/include/matdbg/ShaderReplacer.h",
        "matdbg/include/matdbg/TextWriter.h",
    ],
    copts = filament_copts() + [
        "-Ithird_party/glslang/glslang/Include",
        "-Ithird_party/glslang/glslang/MachineIndependent",
        "-Ithird_party/glslang/glslang/Public",
        "-Ithird_party/glslang/SPIRV",
        "-Ithird_party/filament/libs/filamat/src",
    ],
    defines = filament_defines(),
    includes = [
        "matdbg/include",
    ],
    linkstatic = 1,
    textual_hdrs = [
        "matdbg/src/ApiHandler.h",
        "matdbg/src/CommonWriter.h",
        "matdbg/src/SourceFormatter.h",
        "filamat/src/eiff/ChunkContainer.h",
        "generated/resources/matdbg_resources.h",
    ],
    deps = [
        ":filabridge_hdrs",
        ":filamat",
        ":generated_matdbg_resources",
        ":utils_hdrs",
        "@civetweb//:civetserver",
        "@glslang",
        "@glslang//:SPIRV",
        "@robin_map",
        "@spirv_cross//:spirv_cross_lib",
        "@spirv_tools",
        "@spirv_tools//:spirv_tools_opt",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ] + if_vulkan(
        [":filaflat_vulkan"],
        [":filaflat"],
    ),
    alwayslink = 1,
)

genrule(
    name = "generate_fgviewer_resources",
    srcs = [
        "fgviewer/web/index.html",
        "fgviewer/web/app.js",
        "fgviewer/web/api.js",
    ],
    outs = [
        "generated/resources/fgviewer_resources.h",
        "generated/resources/fgviewer_resources.c",
    ],
    cmd = "RESGEN=$(location @third_party//filament/tools:resgen) && " +
          "DEPLOY=`dirname $(location generated/resources/fgviewer_resources.h)` && " +
          "WEB_DIR=`dirname $(location fgviewer/web/index.html)` && " +
          "$$RESGEN -t -x $$DEPLOY -c -p fgviewer_resources " +
          "$$WEB_DIR/index.html $$WEB_DIR/app.js $$WEB_DIR/api.js",
    tools = [
        "@third_party//filament/tools:resgen",
    ],
)

cc_library(
    name = "generated_fgviewer_resources",
    srcs = [
        "generated/resources/fgviewer_resources.c",
    ],
    hdrs = [],
    copts = filament_copts(),
    data = [],
    includes = ["generated"],
    linkstatic = 1,
    textual_hdrs = [
        "generated/resources/fgviewer_resources.h",
    ],
    deps = [":generate_fgviewer_resources"],
    alwayslink = 1,
)

cc_library(
    name = "fgviewer",
    srcs = [
        "fgviewer/src/ApiHandler.cpp",
        "fgviewer/src/DebugServer.cpp",
        "fgviewer/src/FrameGraphInfo.cpp",
        "fgviewer/src/JsonWriter.cpp",
    ],
    hdrs = [
        "fgviewer/include/fgviewer/DebugServer.h",
        "fgviewer/include/fgviewer/FrameGraphInfo.h",
        "fgviewer/include/fgviewer/JsonWriter.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "fgviewer/include",
    ],
    linkstatic = 1,
    textual_hdrs = [
        "fgviewer/src/ApiHandler.h",
        "generated/resources/fgviewer_resources.h",
    ],
    deps = [
        ":generated_fgviewer_resources",
        ":utils_hdrs",
        "@civetweb//:civetserver",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

# TODO: Don't blindly pass "-a all -p all" to matc.
genrule(
    name = "generate_iblprefilter_materials",
    srcs = [
        "iblprefilter/src/materials/equirectToCube.mat",
        "iblprefilter/src/materials/generateKernel.mat",
        "iblprefilter/src/materials/iblprefilter.mat",
    ],
    outs = [
        "generated/resources/iblprefilter_materials.h",
        "generated/resources/iblprefilter_materials.c",
        "generated/resources/equirectToCube.filamat",
        "generated/resources/generateKernel.filamat",
        "generated/resources/iblprefilter.filamat",
    ],
    cmd = "MATC=$(location @third_party//filament/tools:matc) && " +
          "RESGEN=$(location @third_party//filament/tools:resgen) && " +
          "DEPLOY=`dirname $(location generated/resources/equirectToCube.filamat)` && " +
          "$$MATC -a all -p all -o $$DEPLOY/equirectToCube.filamat $(location iblprefilter/src/materials/equirectToCube.mat) && " +
          "$$MATC -a all -p all -o $$DEPLOY/generateKernel.filamat $(location iblprefilter/src/materials/generateKernel.mat) && " +
          "$$MATC -a all -p all -o $$DEPLOY/iblprefilter.filamat $(location iblprefilter/src/materials/iblprefilter.mat) && " +
          "$$RESGEN -x $$DEPLOY -c -p iblprefilter_materials " +
          "$$DEPLOY/equirectToCube.filamat " +
          "$$DEPLOY/generateKernel.filamat " +
          "$$DEPLOY/iblprefilter.filamat",
    tools = [
        "@third_party//filament/tools:matc",
        "@third_party//filament/tools:resgen",
    ],
)

cc_library(
    name = "generated_iblprefilter_materials",
    srcs = [
        "generated/resources/iblprefilter_materials.c",
    ],
    hdrs = [],
    copts = filament_copts(),
    data = [],
    includes = ["generated"],
    linkstatic = 1,
    textual_hdrs = [
        "generated/resources/iblprefilter_materials.h",
    ],
    alwayslink = 1,
)

cc_library(
    name = "iblprefilter",
    srcs = [
        "iblprefilter/src/IBLPrefilterContext.cpp",
    ],
    hdrs = [
        "iblprefilter/include/filament-iblprefilter/IBLPrefilterContext.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "iblprefilter/include",
    ],
    linkstatic = 1,
    textual_hdrs = ["generated/resources/iblprefilter_materials.h"],
    deps = [
        ":filabridge_hdrs",
        ":generated_iblprefilter_materials",
        ":math_hdrs",
        ":utils_hdrs",
        ":utils_private_hdrs",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)
