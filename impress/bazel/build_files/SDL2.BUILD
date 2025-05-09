# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# Simple DirectMedia Layer is a cross-platform development library
# designed to provide low level access to audio, keyboard, mouse, joystick, and
# graphics hardware via OpenGL and Direct3D. It is used by video playback
# software, emulators, and popular games including Valve's award winning catalog
# and many Humble Bundle games.
#

package(
    features = [
        "-parse_headers",
        "-layering_check",
    ],
)

licenses(["notice"])

exports_files(["LICENSE"])

sdl_includes = ["include"]

SDL_DEFAULT_SRC_EXCLUDES = [
    # Link directly against the math library (-lm), see sdl_linkopts.
    "src/libm/*.c",
    # Test files are compiled as part of the ":SDL2_test" target.
    "src/test/*.c",
    # Unfortunately, thread/generic does not contain any guards to
    # prevent collisions against thread/pthread.
    "src/thread/generic/*.c",
    # Don't include Windows support.
    "src/render/direct3d*/**",
    "src/render/SDL_d3d*",
    "src/haptic/windows/**",
    # Remove support for QNX
    "src/video/qnx/**",
]

sdl_sources = select({
    "@com_google_impress//third_party/filament:macos": glob(
        include = [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = SDL_DEFAULT_SRC_EXCLUDES + ["src/core/linux/**"],
    ),
    "@com_google_impress//third_party/filament:wasm": glob(
        include = [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = SDL_DEFAULT_SRC_EXCLUDES + [
            "src/core/**",
            "src/thread/**",
            "src/video/**",
        ],
    ) + glob([
        "src/thread/*",
        "src/thread/generic/*",
        "src/video/*.c",
        "src/video/*.h",
        "src/video/dummy/**",
        "src/video/emscripten/**",
        "src/video/yuv2rgb/*.c",
        "src/video/yuv2rgb/*.h",
    ]),
    "@com_google_impress//third_party/filament:windows": glob(
        include = [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = [
            "src/test/*.c",
            "src/video/qnx/**",
            "src/core/linux/**",
            "src/thread/**",
            "src/core/unix/**",
        ],
    ) + glob([
        "src/thread/*",
        "src/thread/windows/*",
        "src/thread/generic/SDL_syscond.*",
    ]),
    "//conditions:default": glob(
        include = [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = SDL_DEFAULT_SRC_EXCLUDES,
    ),
})

sdl_headers = [
    "include/SDL.h",
    "include/SDL_assert.h",
    "include/SDL_atomic.h",
    "include/SDL_audio.h",
    "include/SDL_bits.h",
    "include/SDL_blendmode.h",
    "include/SDL_clipboard.h",
    "include/SDL_config.h",
    "include/SDL_config_android.h",
    "include/SDL_config_linux.h",
    "include/SDL_config_macosx.h",
    "include/SDL_config_windows.h",
    "include/SDL_config_minimal.h",
    "include/SDL_cpuinfo.h",
    "include/SDL_egl.h",
    "include/SDL_endian.h",
    "include/SDL_error.h",
    "include/SDL_events.h",
    "include/SDL_filesystem.h",
    "include/SDL_gamecontroller.h",
    "include/SDL_gesture.h",
    "include/SDL_haptic.h",
    "include/SDL_hints.h",
    "include/SDL_joystick.h",
    "include/SDL_keyboard.h",
    "include/SDL_keycode.h",
    "include/SDL_loadso.h",
    "include/SDL_log.h",
    "include/SDL_main.h",
    "include/SDL_messagebox.h",
    "include/SDL_mouse.h",
    "include/SDL_mutex.h",
    "include/SDL_name.h",
    "include/SDL_opengl.h",
    "include/SDL_opengles.h",
    "include/SDL_opengles2.h",
    "include/SDL_opengles2_gl2.h",
    "include/SDL_opengles2_gl2ext.h",
    "include/SDL_opengles2_gl2platform.h",
    "include/SDL_opengles2_khrplatform.h",
    "include/SDL_opengl_glext.h",
    "include/SDL_pixels.h",
    "include/SDL_platform.h",
    "include/SDL_power.h",
    "include/SDL_quit.h",
    "include/SDL_rect.h",
    "include/SDL_render.h",
    "include/SDL_revision.h",
    "include/SDL_rwops.h",
    "include/SDL_scancode.h",
    "include/SDL_shape.h",
    "include/SDL_stdinc.h",
    "include/SDL_surface.h",
    "include/SDL_system.h",
    "include/SDL_syswm.h",
    "include/SDL_thread.h",
    "include/SDL_timer.h",
    "include/SDL_touch.h",
    "include/SDL_version.h",
    "include/SDL_video.h",
    "include/SDL_vulkan.h",
]

# Due to https://github.com/bazelbuild/bazel/issues/680 *.c files can't be
# included when using bazel, so instead use textual_hdrs.
sdl_textual_hdrs = [
    "include/begin_code.h",
    "include/close_code.h",
] + select({
    "@com_google_impress//third_party/filament:macos": ["src/thread/generic/SDL_syssem.c"],
    "//conditions:default": [],
})

sdl_copts = select({
    "@com_google_impress//third_party/filament:android": [
        "-DGL_GLEXT_PROTOTYPES",
        "-Wno-incompatible-pointer-types",
        "-Wno-string-conversion",
        "-pthread",
    ],
    "@com_google_impress//third_party/filament:windows": [
        "-Wno-empty-body",
        "-Wno-knr-promoted-parameter",
        "-Wno-pragma-pack",
    ],
    "@com_google_impress//third_party/filament:wasm": [],
    "//conditions:default": [
        "-Wno-string-conversion",
        "-pthread",
    ],
})

sdl_features = select({
    "@com_google_impress//third_party/filament:windows": ["gdi"],
    "//conditions:default": [],
})

sdl_linkopts = select({
    "@com_google_impress//third_party/filament:android": [
        "-lm",
        "-ldl",
    ],
    "@com_google_impress//third_party/filament:macos": [
        "-lm",
        "-ldl",
    ],
    "@com_google_impress//third_party/filament:windows": [
        "advapi32.lib",
        "gdi32.lib",
        "imm32.lib",
        "ole32.lib",
        "oleaut32.lib",
        "opengl32.lib",
        "shell32.lib",
        "user32.lib",
        "version.lib",
        "winmm.lib",
    ],
    "//conditions:default": [
        "-lm",
        "-ldl",
        "-lrt",
    ],
})

sdl_deps = select({
    "@com_google_impress//third_party/filament:android": [
        "@third_party//gl",
    ],
    "@com_google_impress//third_party/filament:macos": [
        ":osx_lib",
    ],
    "@com_google_impress//third_party/filament:windows": [
    ],
    "//conditions:default": [
        #"@third_party//gl:GLES2_headers",
        #"@third_party//gl:GLX_headers",
        #"@third_party//gl:OpenGL_headers",
        #"//third_party/Xorg:includes",
        #"//third_party/alsa_lib:alsa_headers",
        #"//third_party/libxcb:includes",
    ],
})

objc_library(
    name = "osx_lib",
    srcs = select({
        "@com_google_impress//third_party/filament:macos": glob([
            "src/**/*.h",
            "include/*.h",
        ]),
        "//conditions:default": [],
    }),
    copts = sdl_copts,
    features = sdl_features,
    includes = sdl_includes,
    non_arc_srcs = select({
        "@com_google_impress//third_party/filament:macos": glob([
            "src/audio/coreaudio/*.m",
            "src/file/cocoa/*.m",
            "src/filesystem/cocoa/*.m",
            "src/video/cocoa/*.m",
        ]),
        "//conditions:default": [],
    }),
    deps = [
        "//third_party/apple_frameworks:AudioToolbox",
        "//third_party/apple_frameworks:Carbon",
        "//third_party/apple_frameworks:Cocoa",
        "//third_party/apple_frameworks:CoreAudio",
        "//third_party/apple_frameworks:CoreVideo",
        "//third_party/apple_frameworks:ForceFeedback",
        "//third_party/apple_frameworks:IOKit",
    ] + select({
        "@com_google_impress//third_party/filament:macos": ["//third_party/apple_frameworks:AudioUnit"],
        "//conditions:default": [],
    }),
)

# Internal headers required for custom video drivers.
cc_library(
    name = "SDL2_internal_hdrs",
    hdrs = [
        "include/begin_code.h",
        "include/close_code.h",
        "src/SDL_internal.h",
        "src/dynapi/SDL_dynapi.h",
        "src/dynapi/SDL_dynapi_overrides.h",
        "src/video/SDL_egl_c.h",
        "src/video/SDL_sysvideo.h",
        "src/video/SDL_vulkan_internal.h",
        "src/video/khronos/vulkan/vk_platform.h",
        "src/video/khronos/vulkan/vulkan.h",
    ] + sdl_headers,
    includes = sdl_includes,
    deps = [
        "//third_party/libxcb:includes",
    ],
)

cc_library(
    name = "SDL2",
    srcs = sdl_sources,
    hdrs = sdl_headers,
    copts = sdl_copts,
    features = sdl_features,
    includes = sdl_includes,
    linkopts = sdl_linkopts,
    linkstatic = 1,
    textual_hdrs = sdl_textual_hdrs,
    visibility = ["//visibility:public"],
    deps = sdl_deps,
    include_prefix = "SDL2",
)

# Static linking SDL results in JNI_OnLoad() being exclusively hooked by
# SDL which prevents applications from performing initialization on this
# entry point.  The library build has been changed to rename JNI_OnLoad()
# to SDL_JNI_OnLoad() so that applications have the option of providing
# their own initialization prior to calling SDL.
# See SDL/src/core/android/SDL_android.c
cc_library(
    name = "SDL2_jni_onload_renamed",
    srcs = sdl_sources,
    hdrs = sdl_headers,
    copts = sdl_copts + ["-DJNI_OnLoad=SDL_JNI_OnLoad"],
    features = sdl_features,
    includes = sdl_includes,
    linkopts = sdl_linkopts,
    linkstatic = 1,
    textual_hdrs = sdl_textual_hdrs,
    visibility = ["//visibility:public"],
    deps = sdl_deps,
)
