load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load(
    "@third_party//filament:filament.bzl",
    "filament_copts",
    "filament_defines",
    "if_android",
    "if_apple",
    "if_desktop",
    "if_ios",
    "if_linux",
    "if_macos",
    "if_profiling_mode",
    "if_wasm",
    "if_windows",
)

licenses(["notice"])

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -fno-rtti
    ],
)

cc_library(
    name = "driver_enums",
    hdrs = [
        "include/backend/DescriptorSetOffsetArray.h",
        "include/backend/DriverApiForward.h",
        "include/backend/DriverEnums.h",
        "include/backend/Platform.h",
        "include/backend/PresentCallable.h",
    ],
    includes = [
        "include",
    ],
    deps = [
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "platform",
    hdrs = [
        "include/backend/Platform.h",
    ],
    includes = [
        "include",
    ],
    deps = [
        ":driver_enums",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "backend_utils",
    srcs = [
        "src/BackendUtils.cpp",
    ] + if_android([
        "src/BackendUtilsAndroid.cpp",
    ]),
    hdrs = [
        "include/private/backend/BackendUtils.h",
        "src/DataReshaper.h",
    ] + if_android([
        "include/private/backend/BackendUtilsAndroid.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
    ],
    linkstatic = True,
    deps = [
        ":driver_enums",
        ":pixel_buffer_descriptor",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "pixel_buffer_descriptor",
    hdrs = [
        "include/backend/BufferDescriptor.h",
        "include/backend/PixelBufferDescriptor.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
    ],
    linkstatic = True,
    deps = [
        ":driver_enums",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "virtual_machine_env",
    srcs = [
        "src/VirtualMachineEnv.cpp",
    ],
    hdrs = [
        "include/private/backend/VirtualMachineEnv.h",
    ],
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
    ],
    linkstatic = True,
    deps = [
        "@third_party//filament/libs:utils_hdrs",
    ] + if_linux([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

backend_srcs = [
    "include/private/backend/CircularBuffer.h",
    "include/private/backend/CommandBufferQueue.h",
    "include/private/backend/CommandStream.h",
    "include/private/backend/Driver.h",
    "include/private/backend/DriverAPI.inc",
    "include/private/backend/DriverApi.h",
    "include/private/backend/HandleAllocator.h",
    "include/private/backend/PlatformFactory.h",
    "src/BlobCacheKey.cpp",
    "src/BlobCacheKey.h",
    "src/Callable.cpp",
    "src/CallbackManager.cpp",
    "src/CallbackManager.h",
    "src/CircularBuffer.cpp",
    "src/CommandBufferQueue.cpp",
    "src/CommandStream.cpp",
    "src/CommandStreamDispatcher.h",
    "src/CompilerThreadPool.cpp",
    "src/CompilerThreadPool.h",
    "src/Driver.cpp",
    "src/DriverBase.h",
    "src/Handle.cpp",
    "src/HandleAllocator.cpp",
    "src/JobQueue.cpp",
    "src/JobQueue.h",
    "src/ostream.cpp",
    "src/Platform.cpp",
    "src/PlatformFactory.cpp",
    "src/Program.cpp",
    "src/SystraceProfile.h",
    "src/noop/NoopDriver.cpp",
    "src/noop/PlatformNoop.cpp",
] + if_android([
    "src/AndroidFrameCallback.h",
    "src/AndroidFrameCallback.cpp",
    "src/AndroidNdk.cpp",
    "src/AndroidNativeWindow.cpp",
    "src/AndroidNativeWindow.h",
    "src/AndroidSwapChainHelper.cpp",
    "src/AndroidSwapChainHelper.h",
])

backend_opengl_srcs = backend_srcs + [
    "src/opengl/OpenGLBlobCache.cpp",
    "src/opengl/OpenGLBlobCache.h",
    "src/opengl/GLMemoryMappedBuffer.cpp",
    "src/opengl/GLMemoryMappedBuffer.h",
    "src/opengl/GLUtils.cpp",
    "src/opengl/GLUtils.h",
    "src/opengl/OpenGLContext.cpp",
    "src/opengl/OpenGLDriver.cpp",
    "src/opengl/OpenGLPlatform.cpp",
    "src/opengl/OpenGLProgram.cpp",
    "src/opengl/OpenGLDriverFactory.h",
    "src/opengl/OpenGLProgram.h",
    "src/opengl/OpenGLTimerQuery.cpp",
    "src/opengl/OpenGLTimerQuery.h",
    "src/opengl/ShaderCompilerService.cpp",
    "src/opengl/ShaderCompilerService.h",
    "src/opengl/gl_headers.cpp",
    "src/opengl/gl_headers.h",
    "src/opengl/BindingMap.h",
    "src/opengl/GLDescriptorSet.h",
    "src/opengl/GLDescriptorSet.cpp",
    "src/opengl/GLDescriptorSetLayout.h",
] + if_android([
    "src/opengl/platforms/ExternalStreamManagerAndroid.cpp",
    "src/opengl/platforms/ExternalStreamManagerAndroid.h",
    "src/opengl/platforms/PlatformEGL.cpp",
    "src/opengl/platforms/PlatformEGLAndroid.cpp",
]) + if_windows([
    "src/opengl/platforms/PlatformWGL.cpp",
]) + if_wasm([
    "src/opengl/platforms/PlatformWebGL.cpp",
])

backend_metal_srcs = backend_srcs + [
    "src/metal/MetalBlitter.mm",
    "src/metal/MetalBuffer.mm",
    "src/metal/MetalBufferPool.mm",
    "src/metal/MetalContext.mm",
    "src/metal/MetalDriver.mm",
    "src/metal/MetalEnums.mm",
    "src/metal/MetalExternalImage.mm",
    "src/metal/MetalHandles.mm",
    "src/metal/MetalResourceTracker.cpp",
    "src/metal/MetalShaderCompiler.mm",
    "src/metal/MetalState.mm",
    "src/metal/MetalTimerQuery.mm",
    "src/metal/MetalUtils.mm",
    "src/metal/PlatformMetal.mm",
]

backend_vulkan_only_srcs = [
    "src/vulkan/memory/Resource.cpp",
    "src/vulkan/memory/Resource.h",
    "src/vulkan/memory/ResourceManager.cpp",
    "src/vulkan/memory/ResourceManager.h",
    "src/vulkan/memory/ResourcePointer.h",
    "src/vulkan/platform/VulkanPlatform.cpp",
    "src/vulkan/platform/VulkanPlatformSwapChainImpl.cpp",
    "src/vulkan/platform/VulkanPlatformSwapChainImpl.h",
    "src/vulkan/VulkanAsyncHandles.cpp",
    "src/vulkan/VulkanAsyncHandles.h",
    "src/vulkan/VulkanBlitter.cpp",
    "src/vulkan/VulkanBlitter.h",
    "src/vulkan/VulkanBuffer.h",
    "src/vulkan/VulkanBufferCache.h",
    "src/vulkan/VulkanBufferCache.cpp",
    "src/vulkan/VulkanBufferProxy.h",
    "src/vulkan/VulkanBufferProxy.cpp",
    "src/vulkan/VulkanCommands.cpp",
    "src/vulkan/VulkanCommands.h",
    "src/vulkan/VulkanConstants.h",
    "src/vulkan/VulkanContext.cpp",
    "src/vulkan/VulkanContext.h",
    "src/vulkan/VulkanDescriptorSetCache.cpp",
    "src/vulkan/VulkanDescriptorSetCache.h",
    "src/vulkan/VulkanDescriptorSetLayoutCache.cpp",
    "src/vulkan/VulkanDescriptorSetLayoutCache.h",
    "src/vulkan/VulkanDriver.cpp",
    "src/vulkan/VulkanDriverFactory.h",
    "src/vulkan/VulkanDriver.h",
    "src/vulkan/VulkanExternalImageManager.cpp",
    "src/vulkan/VulkanExternalImageManager.h",
    "src/vulkan/VulkanStreamedImageManager.cpp",
    "src/vulkan/VulkanStreamedImageManager.h",
    "src/vulkan/VulkanFboCache.cpp",
    "src/vulkan/VulkanFboCache.h",
    "src/vulkan/VulkanHandles.cpp",
    "src/vulkan/VulkanHandles.h",
    "src/vulkan/VulkanMemory.cpp",
    "src/vulkan/VulkanMemory.h",
    "src/vulkan/VulkanPipelineCache.cpp",
    "src/vulkan/VulkanPipelineCache.h",
    "src/vulkan/VulkanPipelineLayoutCache.cpp",
    "src/vulkan/VulkanPipelineLayoutCache.h",
    "src/vulkan/VulkanQueryManager.cpp",
    "src/vulkan/VulkanQueryManager.h",
    "src/vulkan/VulkanSamplerCache.cpp",
    "src/vulkan/VulkanSamplerCache.h",
    "src/vulkan/VulkanSemaphore.cpp",
    "src/vulkan/VulkanSemaphore.h",
    "src/vulkan/VulkanSemaphoreManager.cpp",
    "src/vulkan/VulkanSemaphoreManager.h",
    "src/vulkan/VulkanStagePool.cpp",
    "src/vulkan/VulkanStagePool.h",
    "src/vulkan/VulkanSwapChain.cpp",
    "src/vulkan/VulkanSwapChain.h",
    "src/vulkan/VulkanReadPixels.h",
    "src/vulkan/VulkanReadPixels.cpp",
    "src/vulkan/VulkanTexture.cpp",
    "src/vulkan/VulkanTexture.h",
    "src/vulkan/VulkanYcbcrConversionCache.cpp",
    "src/vulkan/VulkanYcbcrConversionCache.h",
    "src/vulkan/utils/Conversion.cpp",
    "src/vulkan/utils/Conversion.h",
    "src/vulkan/utils/Definitions.h",
    "src/vulkan/utils/Helper.h",
    "src/vulkan/utils/Image.h",
    "src/vulkan/utils/Image.cpp",
    "src/vulkan/utils/Spirv.h",
    "src/vulkan/utils/Spirv.cpp",
    "src/vulkan/utils/StaticVector.h",
] + if_android(
    ["src/vulkan/platform/VulkanPlatformAndroid.cpp"],
) + if_linux(
    ["src/vulkan/platform/VulkanPlatformLinux.cpp"],
) + if_windows(
    ["src/vulkan/platform/VulkanPlatformWindows.cpp"],
)

backend_vulkan_srcs = backend_srcs + backend_vulkan_only_srcs

backend_opengl_objc_srcs = backend_opengl_srcs + if_ios(
    [
        "src/opengl/platforms/CocoaTouchExternalImage.h",
        "src/opengl/platforms/CocoaTouchExternalImage.mm",
        "src/opengl/platforms/PlatformCocoaTouchGL.mm",
    ],
    [
        "src/opengl/platforms/CocoaExternalImage.h",
        "src/opengl/platforms/CocoaExternalImage.mm",
        "src/opengl/platforms/PlatformCocoaGL.mm",
    ],
)

backend_textual_hdrs = [
    "include/private/backend/CircularBuffer.h",
    "include/private/backend/CommandBufferQueue.h",
    "include/private/backend/CommandStream.h",
    "include/private/backend/Dispatcher.h",
    "include/private/backend/Driver.h",
    "include/private/backend/DriverAPI.inc",
    "include/private/backend/DriverApi.h",
    "include/private/backend/HandleAllocator.h",
    "include/private/backend/PlatformFactory.h",
    "src/CommandStreamDispatcher.h",
    "src/DataReshaper.h",
    "src/DriverBase.h",
    "src/noop/NoopDriver.h",
    "src/noop/PlatformNoop.h",
]

backend_opengl_textual_hdrs = backend_textual_hdrs + [
    "src/opengl/GLBufferObject.h",
    "src/opengl/GLTexture.h",
    "src/opengl/NullGLES.h",
    "src/opengl/OpenGLDriver.h",
    "src/opengl/OpenGLDriverBase.h",
    "src/opengl/OpenGLContext.h",
    "src/opengl/OpenGLTimerQuery.h",
]

backend_metal_textual_hdrs = backend_textual_hdrs + [
    "include/backend/platforms/PlatformMetal-ObjC.h",
    "src/metal/MetalBlitter.h",
    "src/metal/MetalBuffer.h",
    "src/metal/MetalBufferPool.h",
    "src/metal/MetalContext.h",
    "src/metal/MetalDriver.h",
    "src/metal/MetalDriverFactory.h",
    "src/metal/MetalEnums.h",
    "src/metal/MetalErrorQueue.h",
    "src/metal/MetalFlags.h",
    "src/metal/MetalExternalImage.h",
    "src/metal/MetalHandles.h",
    "src/metal/MetalResourceTracker.h",
    "src/metal/MetalShaderCompiler.h",
    "src/metal/MetalState.h",
    "src/metal/MetalTimerQuery.h",
    "src/metal/MetalUtils.h",
]

backend_hdrs_hdrs = [
    "include/backend/AcquiredImage.h",
    "include/backend/CallbackHandler.h",
    "include/backend/DriverApiForward.h",
    "include/backend/Handle.h",
    "include/backend/PipelineState.h",
    "include/backend/Platform.h",
    "include/backend/Program.h",
    "include/backend/SamplerDescriptor.h",
    "include/backend/TargetBufferInfo.h",
    "include/backend/platforms/OpenGLPlatform.h",
]

backend_hdrs_deps = [
    ":driver_enums",
    ":pixel_buffer_descriptor",
    "@third_party//filament/libs:math_hdrs",
    "@third_party//filament/libs:utils_hdrs",
]

cc_library(
    name = "backend_hdrs",
    hdrs = backend_hdrs_hdrs + if_ios([
        "include/backend/platforms/PlatformCocoaTouchGL.h",
    ]) + if_macos([
        "include/backend/platforms/PlatformCocoaGL.h",
    ]) + if_android([
        "include/backend/platforms/PlatformEGL.h",
        "include/backend/platforms/PlatformEGLAndroid.h",
        "include/backend/platforms/AndroidNdk.h",
    ]) + if_windows([
        "include/backend/platforms/PlatformWGL.h",
    ]) + if_wasm([
        "include/backend/platforms/PlatformWebGL.h",
    ]) + if_linux([
        "include/backend/platforms/PlatformGLX.h",
    ]) + if_apple([
        "include/backend/platforms/PlatformMetal.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
        "src",
    ],
    linkstatic = 1,
    textual_hdrs = if_apple(
        backend_metal_textual_hdrs,
        backend_opengl_textual_hdrs,
    ),
    deps = backend_hdrs_deps + if_desktop([
        "@third_party//filament/libs:bluegl_hdrs",
    ]) + if_linux([
    ]),
    alwayslink = 1,
)

cc_library(
    name = "backend_vk_hdrs",
    hdrs = backend_hdrs_hdrs + [
        "include/backend/platforms/VulkanPlatform.h",
    ] + if_android([
        "include/backend/platforms/VulkanPlatformAndroid.h",
        "include/backend/platforms/AndroidNdk.h",
    ]) + if_windows([
        "include/backend/platforms/VulkanPlatformWindows.h",
    ]) + if_linux([
        "include/backend/platforms/VulkanPlatformLinux.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = [
        "include",
        "src",
    ],
    linkstatic = 1,
    textual_hdrs = backend_textual_hdrs,
    deps = backend_hdrs_deps + [
        "@third_party//filament/libs:bluevk_hdrs",
    ],
    alwayslink = 1,
)

backend_defines = filament_defines() + if_profiling_mode(
    ["FILAMENT_FORCE_PROFILING_MODE"],
)

backend_opengl_defines = backend_defines + [
    "FILAMENT_SUPPORTS_OPENGL=1",
]

backend_vulkan_defines = [
    "FILAMENT_DRIVER_SUPPORTS_VULKAN",
    # Needed, otherwise Filament sets this to 1, which is incompatible with Google3
    "VMA_DYNAMIC_VULKAN_FUNCTIONS=0",
]

backend_vulkan_deps = [
    ":backend_vk_hdrs",
    "@smol_v//:smol-v",
    "@spirv_headers//:spirv_cpp_headers",
    "@vulkan_memory_allocator",
]

backend_deps = [
    ":backend_hdrs",
    ":backend_utils",
    ":driver_enums",
    ":pixel_buffer_descriptor",
    "@third_party//filament/libs:math_hdrs",
    "@third_party//filament/libs:utils",
    "@third_party//filament/libs:utils_hdrs",
    "@third_party//filament/libs:utils_private_hdrs",
    "@robin_map",
]

backend_opengl_deps = backend_deps + if_desktop([
    "@third_party//filament/libs:bluegl_hdrs",
]) + if_linux([
]) + if_android([
    ":virtual_machine_env",
])

cc_library(
    name = "backend_noop",
    srcs = backend_srcs,
    copts = filament_copts(),
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_textual_hdrs,
    deps = backend_deps,
    alwayslink = 1,
)

# Filament backend library for non-mac OpenGL
cc_library(
    name = "backend_opengl_default",
    srcs = backend_opengl_srcs + if_linux([
        "src/opengl/platforms/PlatformGLX.cpp",
    ]),
    copts = filament_copts(),
    defines = backend_opengl_defines + if_linux(
        ["FILAMENT_SUPPORTS_X11"],
    ),
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = backend_opengl_deps + if_android(
        [],
        [
            "@third_party//filament/libs:bluegl",
        ],
    ),
    alwayslink = 1,
)

cc_library(
    name = "backend_gl_vulkan",
    srcs = backend_opengl_srcs + backend_vulkan_only_srcs,
    copts = filament_copts(),
    defines = backend_opengl_defines + backend_vulkan_defines,
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = backend_opengl_deps + backend_vulkan_deps + [
        "@third_party//filament/libs:bluevk",
    ],
    alwayslink = 1,
)

cc_library(
    name = "backend_opengl_osmesa",
    srcs = backend_opengl_srcs + [
        "src/opengl/platforms/PlatformOSMesa.cpp",
    ],
    hdrs = [
        "include/backend/platforms/PlatformOSMesa.h",
    ],
    copts = filament_copts(),
    defines = backend_opengl_defines + [
        "FILAMENT_SUPPORTS_OSMESA",
    ],
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = backend_opengl_deps + [
        "//third_party/mesa:OSMesa_header_thirdparty",
        "@third_party//filament/libs:bluegl_osmesa",
    ],
    alwayslink = 1,
)

cc_library(
    name = "egl_backend_hdrs",
    hdrs = backend_hdrs_hdrs + if_linux([
        "include/backend/platforms/PlatformEGL.h",
        "include/backend/platforms/PlatformEGLHeadless.h",
    ]),
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["include"],
    linkstatic = 1,
    textual_hdrs = if_apple(
        backend_metal_textual_hdrs,
        backend_opengl_textual_hdrs,
    ),
    deps = backend_hdrs_deps + [
        ":backend_hdrs",
        ":backend_opengl_private_hdrs",
        "@third_party//filament/libs:bluegl_hdrs",
        "@third_party//gl",
    ] + if_linux([
    ]),
    alwayslink = 1,
)

egl_backend_deps = [
    ":egl_backend_hdrs",
    ":backend_utils",
    ":driver_enums",
    ":pixel_buffer_descriptor",
    "@third_party//filament/libs:math_hdrs",
    "@third_party//filament/libs:utils",
    "@third_party//filament/libs:utils_hdrs",
    "@third_party//filament/libs:utils_private_hdrs",
    "@robin_map",
]

cc_library(
    name = "backend_opengl_egl_headless",
    srcs = backend_opengl_srcs + [
        "src/opengl/platforms/PlatformEGL.cpp",
        "src/opengl/platforms/PlatformEGLHeadless.cpp",
    ],
    hdrs = [
        "include/backend/platforms/PlatformEGL.h",
        "include/backend/platforms/PlatformEGLHeadless.h",
    ],
    copts = filament_copts(),
    defines = backend_opengl_defines + [
        "FILAMENT_SUPPORTS_EGL_ON_LINUX=1",
    ],
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = egl_backend_deps + [
        "@third_party//filament/libs:bluegl_hdrs",
        "@third_party//filament/libs:bluegl_linux_egl",
        "@third_party//gl",
        "@third_party//gl/native:EGL",
        "@third_party//gl/native:GL",
    ],
    alwayslink = 1,
)

cc_library(
    name = "backend_opengl_egl_headless_fl0",
    srcs = backend_opengl_srcs + [
        "src/opengl/platforms/PlatformEGL.cpp",
        "src/opengl/platforms/PlatformEGLHeadless.cpp",
    ],
    hdrs = [
        "include/backend/platforms/PlatformEGL.h",
        "include/backend/platforms/PlatformEGLHeadless.h",
    ],
    copts = filament_copts(),
    defines = backend_opengl_defines + [
        "FILAMENT_SUPPORTS_EGL_ON_LINUX=1",
        "FILAMENT_ENABLE_FEATURE_LEVEL_0=1",
    ],
    includes = ["src"],
    linkstatic = True,
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = egl_backend_deps + [
        "//third_party/angle:EGL",
        "//third_party/angle:GLESv2",
        "//third_party/angle:swiftshader_icd",
        "@third_party//filament/libs:bluegl_hdrs",
        "@third_party//filament/libs:bluegl_linux_egl",
        "@third_party//gl",
    ],
    alwayslink = 1,
)

# Vulkan backends
[
    cc_library(
        name = "%s" % lib_name,
        srcs = backend_vulkan_srcs,
        copts = filament_copts(),
        defines = backend_defines + backend_vulkan_defines + extra_defines,
        includes = [
            "src",
        ],
        linkstatic = True,
        textual_hdrs = backend_textual_hdrs,
        deps = deps + backend_deps + backend_vulkan_deps,
        alwayslink = 1,
    )
    for lib_name, extra_defines, deps in [
        (
            "backend_vulkan_for_linux",
            [
                "FILAMENT_SUPPORTS_X11",
                "FILAMENT_SUPPORTS_XLIB",
            ],
            [
                "@third_party//filament/libs:bluevk",  # This should come early in the dep list.
            ],
        ),
        (
            "backend_vulkan_for_linux_headless",
            [
                "FILAMENT_SUPPORTS_X11",
            ],
            [
                "@third_party//filament/libs:bluevk_headless",  # This should come early in the dep list.
            ],
        ),
        (
            "backend_vulkan_for_linux_headless_static",
            [
                "FILAMENT_SUPPORTS_X11",
            ],
            [
                "@third_party//filament/libs:bluevk_static",  # This should come early in the dep list.
            ],
        ),
        # Filament backend library for non-mac vulkan swiftshader
        (
            "backend_vulkan_swiftshader",
            [
                "FILAMENT_USE_SWIFTSHADER",
                "FILAMENT_SUPPORTS_X11",
            ],
            [
                "@third_party//filament/libs:bluevk_swiftshader",  # This should come early in the dep list.
            ],
        ),
        # Filament backend library for Android
        (
            "backend_vulkan_android",
            [],
            [
                "@third_party//filament/libs:bluevk",  # This should come early in the dep list.
            ],
        ),
    ]
]

# Filament driver for OpenGL on Cocoa (iOS or macOS)
objc_library(
    name = "backend_opengl_objc",
    srcs = backend_opengl_objc_srcs,
    copts = filament_copts(),
    defines = filament_defines() + ["FILAMENT_SUPPORTS_OPENGL=1"],
    features = ["-layering_check"],
    includes = [
        "src",
        "src/opengl",
    ],
    textual_hdrs = backend_opengl_textual_hdrs,
    deps = backend_deps + [
        "//third_party/apple_frameworks:CoreGraphics",
        "//third_party/apple_frameworks:CoreVideo",
    ] + if_ios(
        [],
        [
            "@third_party//filament/libs:bluegl",
        ],
    ),
)

# Filament driver for Metal + MacOS.
objc_library(
    name = "backend_metal_objc",
    srcs = backend_metal_srcs,
    copts = filament_copts(),
    defines = filament_defines() + ["FILAMENT_SUPPORTS_METAL=1"],
    features = ["-layering_check"],
    includes = [
        "src",
    ],
    textual_hdrs = backend_metal_textual_hdrs,
    deps = backend_deps + [
        "@third_party//filament/filament:filament_hdrs",
    ],
)

cc_library(
    name = "backend_opengl_private_hdrs",
    copts = filament_copts(),
    defines = filament_defines(),
    includes = ["src"],
    linkstatic = 1,
    textual_hdrs = [
        "src/opengl/OpenGLDriver.h",
        "src/opengl/OpenGLDriverBase.h",
        "src/opengl/OpenGLBlobCache.h",
        "src/opengl/OpenGLContext.h",
        "src/opengl/ShaderCompilerService.h",
        "src/BlobCacheKey.h",
        "src/CallbackManager.h",
        "src/CompilerThreadPool.h",
        "src/DriverBase.h",
        "src/JobQueue.h",
        "include/backend/DescriptorSetOffsetArray.h",
        "include/backend/DriverEnums.h",
        "include/private/backend/BackendUtils.h",
        "include/private/backend/Driver.h",
        "include/private/backend/CommandStream.h",
        "include/private/backend/CommandBufferQueue.h",
        "include/private/backend/DriverApi.h",
        "include/private/backend/HandleAllocator.h",
        "include/private/backend/CircularBuffer.h",
        "include/private/backend/PlatformFactory.h",
        "include/private/backend/DriverAPI.inc",
        "src/opengl/GLMemoryMappedBuffer.h",
        "src/opengl/GLUtils.h",
        "src/opengl/GLDescriptorSet.h",
        "src/opengl/GLDescriptorSetLayout.h",
        "src/opengl/gl_headers.h",
        "src/opengl/NullGLES.h",
    ] + if_android([
        "include/private/backend/BackendUtilsAndroid.h",
    ]),
    deps = [
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

alias(
    name = "backend_opengl",
    actual = if_apple(":backend_opengl_objc", ":backend_opengl_default"),
)

alias(
    name = "backend_metal",
    actual = ":backend_metal_objc",
)
