load(
    "@third_party//filament:filament.bzl",
    "filament_copts",
    "filament_defines",
    "filament_generate_materials",
    "if_android",
    "if_apple",
    "if_fgviewer",
    "if_ios_simulator",
    "if_matdbg",
    "if_mobile_materials",
    "if_multiview",
)

licenses(["notice"])

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

genrule(
    name = "dfg",
    outs = ["generated/data/dfg.inc"],
    cmd = "$(location @third_party//filament/tools:cmgen) " +
          if_mobile_materials("--size=64 ", "--size=128 ") +
          "--ibl-dfg-multiscatter --ibl-dfg-cloth --ibl-dfg=$@",
    tools = ["@third_party//filament/tools:cmgen"],
)

genrule(
    name = "generate_shaders",
    srcs = ["@third_party//filament:shader_srcs"],
    outs = [
        "generated/shaders.bin",
        "generated/shaders.h",
        "generated/shaders.c",
        "generated/shaders.S",
        "generated/shaders.apple.S",
    ],
    cmd = "DEPLOY=`dirname $(location generated/shaders.bin)` && " +
          "$(location @third_party//filament/tools:resgen) " +
          "-kcp shaders --text --deploy=$$DEPLOY $(SRCS)",
    tools = [
        "@third_party//filament/tools:resgen",
    ],
)

cc_library(
    name = "generated_shaders",
    srcs = [
        "generated/shaders.c",
    ],
    hdrs = [],
    copts = filament_copts(False),
    data = [],
    includes = ["generated"],
    linkstatic = 1,
    textual_hdrs = [
        "generated/shaders.h",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generated_data",
    copts = filament_copts(),
    linkstatic = 1,
    textual_hdrs = [
        "generated/data/dfg.inc",
    ],
    alwayslink = 1,
)

[
    filament_generate_materials(
        api = api,
        enable_fl0 = enable_fl0,
        enable_multiview = enable_multiview,
        material_api = material_api,
        metal_precompile_mode = metal_precompile_mode,
        platform = platform,
    )
    for platform, api, material_api, enable_fl0, enable_multiview, metal_precompile_mode in [
        ("mobile", "opengl", "opengl", True, False, None),
        ("mobile", "opengl", "opengl", False, True, None),
        ("mobile", "opengl", "opengl", False, False, None),
        ("mobile", "vulkan", "vulkan", False, False, None),
        ("mobile", "vulkan", "vulkan", False, True, None),
        ("mobile", "metal", "metal", False, False, None),
        ("mobile", "metal", "metal", False, False, "sim"),
        ("mobile", "metal", "metal", False, False, "device"),
        ("desktop", "opengl", "all", False, False, None),
        ("desktop", "vulkan", "all", False, False, None),
    ]
]

[
    cc_library(
        name = "generated_%s_%s_%s_%s_%s_materials" % (platform, api, fl0, multiview, precompile),
        srcs = [
            "%s/%s/%s/%s/%s/generated/resources/materials.c" % (platform, api, fl0, multiview, precompile),
        ],
        hdrs = [],
        copts = filament_copts(False),
        data = [],
        includes = [
            "%s/%s/%s/%s/%s" % (platform, api, fl0, multiview, precompile),
            "%s/%s/%s/%s/%s/generated" % (platform, api, fl0, multiview, precompile),
        ],
        linkstatic = 1,
        textual_hdrs = [
            "%s/%s/%s/%s/%s/generated/resources/materials.h" % (platform, api, fl0, multiview, precompile),
        ],
        alwayslink = 1,
    )
    for platform, api, fl0, multiview, precompile in [
        ("mobile", "opengl", "yesFL0", "noMV", "noMPC"),
        ("mobile", "opengl", "noFL0", "yesMV", "noMPC"),
        ("mobile", "opengl", "noFL0", "noMV", "noMPC"),
        ("mobile", "metal", "noFL0", "noMV", "noMPC"),
        ("mobile", "metal", "noFL0", "noMV", "yesMPC"),
        ("mobile", "metal", "noFL0", "noMV", "yesMPCSim"),
        ("mobile", "vulkan", "noFL0", "noMV", "noMPC"),
        ("mobile", "vulkan", "noFL0", "yesMV", "noMPC"),
        ("desktop", "opengl", "noFL0", "noMV", "noMPC"),
        ("desktop", "vulkan", "noFL0", "noMV", "noMPC"),
    ]
]

cc_library(
    name = "filament_hdrs",
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
    ],
    linkstatic = 0,
    textual_hdrs = glob([
        "include/filament/*.h",
    ]),
    deps = [
        "@third_party//filament/filament/backend:backend_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:platform",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

# Does *not* include the JNI bindings;
# for that, see "filament-jni"
[
    cc_library(
        name = "filament_%s" % driver,
        srcs = glob([
            "src/*.cpp",
            "src/components/*.cpp",
            "src/details/*.cpp",
            "src/fg/*.cpp",
            "src/ds/*.cpp",
        ]),
        copts = filament_copts() + [
            "-Wno-bool-operation",  # TODO: Remove this.
        ],
        data = extra_data,
        defines = filament_defines() +
                  if_mobile_materials(
                      ["FILAMENT_DFG_LUT_SIZE=64"],
                      ["FILAMENT_DFG_LUT_SIZE=128"],
                  ) + extra_defines,
        includes = [
            "src",
            "src/ds",
        ],
        linkopts = extra_linkopts,
        linkstatic = linkstatic,
        textual_hdrs = glob([
            "src/*.h",
            "src/components/*.h",
            "src/details/*.h",
            "src/fg/*.h",
            "src/fg/details/*.h",
            "src/ds/*.h",
            "src/materials/fsr/*.h",
        ]),
        deps = [
            ":filament_hdrs",
            ":generated_data",
            "@robin_map",
            "@third_party//filament/filament/backend:backend_hdrs",
            "@third_party//filament/filament/backend:backend_opengl_private_hdrs",
            "@third_party//filament/filament/backend:pixel_buffer_descriptor",
            "@third_party//filament/libs:filabridge",
            "@third_party//filament/libs:filabridge_hdrs",
            "@third_party//filament/libs:geometry",
            "@third_party//filament/libs:ibl",
            "@third_party//filament/libs:ibl_hdrs",
            "@third_party//filament/libs:image",
            "@third_party//filament/libs:image_hdrs",
            "@third_party//filament/libs:math_hdrs",
            "@third_party//filament/libs:utils",
            "@third_party//filament/libs:utils_hdrs",
        ] + if_matdbg(["@third_party//filament/libs:matdbg"]) + if_fgviewer(["@third_party//filament/libs:fgviewer"]) + extra_deps,
        alwayslink = 1,
    )
    for driver, extra_deps, extra_data, extra_linkopts, extra_defines, linkstatic, alwayslink in [
        (
            "noop",
            [
                ":generated_desktop_opengl_noFL0_noMV_noMPC_materials",
                "@third_party//filament/libs:filaflat",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to link to OpenGL 4/5 on desktop.
            "opengl_desktop",
            [
                ":generated_desktop_opengl_noFL0_noMV_noMPC_materials",
                "@third_party//filament/libs:filaflat",
            ] + if_apple(
                [
                    "@third_party//filament/filament/backend:backend_opengl_objc",
                ],
                [
                    "@third_party//filament/filament/backend:backend_opengl",
                ],
            ),
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to link to OpenGL ES 3.1 on android and iOS.
            "opengl_mobile",
            [
                "@third_party//filament/libs:filaflat",
            ] + if_apple(
                [
                    "@third_party//filament/filament/backend:backend_opengl_objc",
                ],
                [
                    "@third_party//filament/filament/backend:backend_opengl",
                ],
            ) + if_multiview(
                [
                    ":generated_mobile_opengl_noFL0_yesMV_noMPC_materials",
                ],
                [
                    ":generated_mobile_opengl_noFL0_noMV_noMPC_materials",
                ],
            ),
            [],
            [],
            if_multiview(
                ["FILAMENT_ENABLE_MULTIVIEW=1"],
            ),
            False,
            False,
        ),
        (
            # The main filament library, configured to link to OpenGL ES 3.1 on
            # Android with Feature Level 0 enabled.
            "opengl_android_feature_level_0",
            [
                "@third_party//filament/filament/backend:backend_opengl",
                "@third_party//filament/libs:filaflat",
                ":generated_mobile_opengl_yesFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            ["FILAMENT_ENABLE_FEATURE_LEVEL_0=1"],
            False,
            False,
        ),
        (
            # The main filament library, always configured to link to GLES 3.1
            # (for WASM and iOS Simulator)
            "gles3",
            [
                "@third_party//filament/filament/backend:backend_opengl",
                "@third_party//filament/libs:filaflat",
                ":generated_mobile_opengl_noFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # Software rasterizer for OpenGL using Mesa's osmesa library.
            "opengl_osmesa",
            [
                "@third_party//filament/filament/backend:backend_opengl_osmesa",
                "@third_party//filament/libs:filaflat",
                ":generated_desktop_opengl_noFL0_noMV_noMPC_materials",
                "//third_party/mesa:OSMesa",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to link to Metal, with no shader precompilation
            "metal",
            [
                "@third_party//filament/filament/backend:backend_metal",
                "@third_party//filament/libs:filaflat",
                ":generated_mobile_metal_noFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            [],
            False,
            False,
        ),
        (
            # The main filament library, configured to link to Metal, with shader precompilation enabled
            "metal_with_precompiled_shaders",
            [
                "@third_party//filament/filament/backend:backend_metal",
                "@third_party//filament/libs:filaflat",
            ] + if_ios_simulator(
                [
                    ":generated_mobile_metal_noFL0_noMV_yesMPCSim_materials",
                ],
                otherwise = [
                    ":generated_mobile_metal_noFL0_noMV_yesMPC_materials",
                ],
            ),
            [],
            [],
            [],
            False,
            False,
        ),
        (
            # The main filament library, always configured to use Vulkan+Swiftshader
            "vulkan_swiftshader",
            [
                "@third_party//filament/filament/backend:backend_vulkan_swiftshader",
                "@third_party//filament/libs:filaflat_vulkan",
                ":generated_desktop_vulkan_noFL0_noMV_noMPC_materials",
            ],
            ["//third_party/swiftshader/src:libvk_swiftshader.so"],
            ["-Wl,-rpath=third_party/swiftshader/src"],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to use Vulkan on Linux
            "vulkan_for_linux",
            [
                "@third_party//filament/filament/backend:backend_vulkan_for_linux",
                "@third_party//filament/libs:filaflat_vulkan",
                ":generated_desktop_vulkan_noFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to use Vulkan on Linux
            "vulkan_for_linux_headless",
            [
                "@third_party//filament/filament/backend:backend_vulkan_for_linux_headless",
                "@third_party//filament/libs:filaflat_vulkan",
                ":generated_desktop_vulkan_noFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to use Vulkan on Linux, with statically linked Vulkan loader
            "vulkan_for_linux_headless_static",
            [
                "@third_party//filament/filament/backend:backend_vulkan_for_linux_headless_static",
                "@third_party//filament/libs:filaflat_vulkan",
                ":generated_desktop_vulkan_noFL0_noMV_noMPC_materials",
            ],
            [],
            [],
            [],
            True,
            True,
        ),
        (
            # The main filament library, configured to link to Vulkan on Android
            "vulkan_android",
            [
                "@third_party//filament/libs:filaflat_vulkan",
            ] + if_android(
                [
                    "@third_party//filament/filament/backend:backend_vulkan_android",
                ],
            ) + if_multiview(
                [
                    ":generated_mobile_vulkan_noFL0_yesMV_noMPC_materials",
                ],
                [
                    ":generated_mobile_vulkan_noFL0_noMV_noMPC_materials",
                ],
            ),
            [],
            [],
            if_multiview(
                ["FILAMENT_ENABLE_MULTIVIEW=1"],
            ),
            False,
            False,
        ),
    ]
]

cc_library(
    name = "filament_sandboxed_vulkan_swiftshader",
    data = [
        "//third_party/swiftshader/src:libvk_swiftshader.so",
        "//third_party/vulkan_loader:libvulkan.so.1",
    ],
    linkopts = ["-Wl,-rpath=google3/third_party/swiftshader/src:google3/third_party/vulkan_loader:third_party/vulkan_loader"],
    deps = [
        ":filament_vulkan_swiftshader",
    ],
    alwayslink = 1,
)
