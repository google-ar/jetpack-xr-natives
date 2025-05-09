# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Contains macros for loading repositories that have yet to be converted to Bazel modules.
"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def non_module_dependencies():
    """Repositories that have yet to be converted to Bazel modules"""

    # TODO: Replace with bazel_dep and git_override.
    # Currently, it appears that git_override does not support patches.
    http_archive(
        name = "com_google_sandboxed_api",
        patch_args = ["-p1"],
        # Add package to TOC entry in embedded resources.
        patches = ["//:bazel/patches/com_google_sandboxed_api.patch"],
        sha256 = "dcecf04c3a26f1dbaab84d893904b7131c50c9baf88c896d7ad8bac078705f67",
        strip_prefix = "google-sandboxed-api-f06ee44",
        type = ".tar.gz",
        url = "https://api.github.com/repos/google/sandboxed-api/tarball/f06ee44f248fd08ccd42817f97c54b711cb3dec1",
    )

    # TODO: Investigate why we need these different glog versions.
    http_archive(
        name = "com_github_glog_glog",
        sha256 = "58c9b3b6aaa4dd8b836c0fd8f65d0f941441fb95e27212c5eeb9979cfd3592ab",
        strip_prefix = "glog-0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6",
        urls = [
            "https://github.com/google/glog/archive/0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6.zip",
        ],
    )

    # TODO: Investigate why we need these different glog versions.
    http_archive(
        name = "com_github_glog_glog_no_gflags",
        build_file = "//:bazel/build_files/glog_no_gflags.BUILD",
        sha256 = "58c9b3b6aaa4dd8b836c0fd8f65d0f941441fb95e27212c5eeb9979cfd3592ab",
        strip_prefix = "glog-0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6",
        urls = [
            "https://github.com/google/glog/archive/0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6.zip",
        ],
    )

    # TODO: Investigate why we need these different gflag versions.
    http_archive(
        name = "com_github_gflags_gflags",
        patch_args = ["-p1"],
        # Remove lpthread linkopt.
        patches = ["//:bazel/patches/gflags_lpthread.patch"],
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
        strip_prefix = "gflags-2.2.2",
        urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    )

    # Mediapipe is not in the Bazel Central Registry as of 12/2024.
    # TODOInvestigate removing mediapipe dependency.
    http_archive(
        name = "mediapipe",
        patch_args = ["-p1"],
        patches = [
            "//:bazel/patches/mediapipe.patch",
        ],
        sha256 = "19efaa3b402d4bb318e89aa07a2c88773a4c85c1c4482dc36ca805d7c3bc6e5b",
        strip_prefix = "mediapipe-0.10.18",
        type = ".tar.gz",
        url = "https://github.com/google/mediapipe/archive/refs/tags/v0.10.18.tar.gz",
    )

    # TODO Investigate how we can migrate to the equivalent bazel_dep.
    http_archive(
        name = "com_github_google_flatbuffers",
        sha256 = "acc26a825e3202f753256abee4dd71e9ab40759c2ba5adb6b8ee6bb25e195371",
        strip_prefix = "google-flatbuffers-fb9afba",
        type = ".tar.gz",
        url = "https://api.github.com/repos/google/flatbuffers/tarball/fb9afbafc7dfe226b9db54d4923bfb8839635274",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "smol_v",
        build_file = "//:bazel/build_files/smol_v.BUILD",
        sha256 = "7c9df84a33adde9134f9ff6a7c31a602ff17e5e8de452b1bddd699b33138b740",
        strip_prefix = "aras-p-smol-v-4b52c16",
        type = ".tar.gz",
        url = "https://api.github.com/repos/aras-p/smol-v/tarball/4b52c165c13763051a18e80ffbc2ee436314ceb2",
    )

    # TODOInvestigate how to switch to bazel_dep.
    # We need to patch our build file but the bazel module already has a patch adding a build file:
    # https://github.com/bazelbuild/bazel-central-registry/blob/main/modules/zlib/1.3.1.bcr.3/patches/add_build_file.patch
    http_archive(
        name = "zstdlib",
        build_file = "//:bazel/build_files/zstdlib.BUILD",
        sha256 = "53f4696f3cec8703f12d3402707a6aaf7eb92d43c90d61e1d32454bda5da7b9c",
        strip_prefix = "zstd-1.5.2",
        url = "https://github.com/facebook/zstd/archive/v1.5.2.zip",
    )

    # TODOinvestigate how to switch to bazel_dep.
    # We need to patch our build file but the bazel module already has a patch adding a build file:
    # https://github.com/bazelbuild/bazel-central-registry/blob/main/modules/robin-map/1.3.0/patches/add_build_file.patch
    http_archive(
        name = "robin_map",
        build_file = "//:bazel/build_files/robin_map.BUILD",
        sha256 = "a8424ad3b0affd4c57ed26f0f3d8a29604f0e1f2ef2089f497f614b1c94c7236",
        strip_prefix = "robin-map-1.3.0",
        url = "https://github.com/Tessil/robin-map/archive/v1.3.0/robin-map-1.3.0.tar.gz",
    )

    # TODO Migrate to Bazel module ((broken link))
    http_archive(
        name = "civetweb",
        build_file = "//:bazel/build_files/civetweb.BUILD",
        sha256 = "c0ed531d8ab7a56dc3457eefdbb5568419446181303fd99b6bd9c94532cf85d5",
        strip_prefix = "civetweb-1.9.1",
        url = "https://github.com/civetweb/civetweb/archive/refs/tags/v1.9.1.zip",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "glslang",
        patch_args = ["-p1"],
        # Remove -lpthread linkopt.
        patches = ["//:bazel/patches/glslang.patch"],
        sha256 = "eaa6ee797cfc392311bd2d1f49fa74fdb8ba3f79c787959024e4d08e91d22f57",
        strip_prefix = "KhronosGroup-glslang-12bb860",
        type = ".tar.gz",
        url = "https://api.github.com/repos/KhronosGroup/glslang/tarball/12bb8602dd2ef450e064959eb44ae117a80f01c9",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "spirv_cross",
        build_file = "//:bazel/build_files/spirv_cross.BUILD",
        sha256 = "dd656a51ba4c229c1a0bb220b7470723e8fd4b68abb7f2cf2ca4027df824f4a0",
        strip_prefix = "SPIRV-Cross-vulkan-sdk-1.3.268.0",
        url = "https://github.com/KhronosGroup/SPIRV-Cross/archive/refs/tags/vulkan-sdk-1.3.268.0.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "spirv_tools",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/spirv_tools.patch"],
        sha256 = "a156215a2d7c6c5b267933ed691877a9a66f07d75970da33ce9ad627a71389d7",
        strip_prefix = "SPIRV-Tools-2022.4",
        url = "https://github.com/KhronosGroup/SPIRV-Tools/archive/refs/tags/v2022.4.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "spirv_headers",
        sha256 = "1870d2ffcf695383a510e950821dfef3397ec85bb0a8bf55ba254f0f4f5998e7",
        strip_prefix = "KhronosGroup-SPIRV-Headers-c214f6f",
        type = ".tar.gz",
        url = "https://api.github.com/repos/KhronosGroup/SPIRV-Headers/tarball/c214f6f2d1a7253bb0e9f195c2dc5b0659dc99ef",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "basis_universal",
        build_file = "//:bazel/build_files/basis_universal.BUILD",
        sha256 = "b89563aa5879eed20f56b9cfa03b52848e759531fd5a1d51a8f63c846f96c2ac",
        strip_prefix = "basis_universal-1.16.3",
        url = "https://github.com/BinomialLLC/basis_universal/archive/refs/tags/1.16.3.tar.gz",
    )

    # TODO Migrate to the equivalent bazel_dep.
    # We need to patch our build file but the bazel module already has a patch adding a build file:
    # https://github.com/bazelbuild/bazel-central-registry/blob/main/modules/zlib/1.3.1.bcr.3/patches/add_build_file.patch
    http_archive(
        name = "zlib_imp",
        build_file = "//:bazel/build_files/zlib.BUILD",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/zlib.patch"],
        #repo_mapping = {"@com_github_protocolbuffers_protobuf": "@com_google_protobuf"},
        sha256 = "1525952a0a567581792613a9723333d7f8cc20b87a81f920fb8bc7e3f2251428",
        strip_prefix = "zlib-1.2.13",
        url = "https://github.com/madler/zlib/archive/refs/tags/v1.2.13.tar.gz",
    )

    # TODO Migrate to the equivalent bazel_dep (in my attempt to do so, I was getting a "png/png.h not found" error)
    http_archive(
        name = "png",
        build_file = "//:bazel/build_files/png.BUILD",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/png.patch"],
        sha256 = "d4160037fa5d09fa7cff555037f2a7f2fefc99ca01e21723b19bfcda33015234",
        strip_prefix = "libpng-1.6.38",
        url = "https://github.com/glennrp/libpng/archive/v1.6.38.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "tinyexr",
        build_file = "//:bazel/build_files/tinyexr.BUILD",
        sha256 = "a0553cfa5566a12932942afa086f00d38b2affcc1f8d704fb566f612bd2202b6",
        strip_prefix = "syoyo-tinyexr-76dad42",
        type = ".tar.gz",
        url = "https://api.github.com/repos/syoyo/tinyexr/tarball/76dad4250c0bf2ef45865a13189ac606ccbb9529",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "stblib",
        build_file = "//:bazel/build_files/stblib.BUILD",
        patch_args = [
            "-p1",
        ],
        patches = [
            "//:bazel/patches/stblib.patch",
        ],
        sha256 = "13a99ad430e930907f5611325ec384168a958bf7610e63e60e2fd8e7b7379610",
        strip_prefix = "stb-b42009b3b9d4ca35bc703f5310eedc74f584be58",
        url = "https://github.com/nothings/stb/archive/b42009b3b9d4ca35bc703f5310eedc74f584be58.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "com_google_zetasql",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/zetasql.patch"],
        sha256 = "58510f44dc815648039fd4e45fc40da3d9f0a746cf483e6bbc5171e984eb7e79",
        strip_prefix = "zetasql-2024.03.1",
        url = "https://github.com/google/zetasql/archive/refs/tags/2024.03.1.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "boost_beast",
        build_file = "//:bazel/build_files/boost_beast.BUILD",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/boost_beast.patch"],
        sha256 = "c4f5232c7cbc8b2b5196d0661c2d0b3dd75bd1056a12510ef6b57736a207f01e",
        strip_prefix = "boostorg-beast-b986b3b",
        type = ".tar.gz",
        url = "https://api.github.com/repos/boostorg/beast/tarball/b986b3b1b0dbd96e4426d29afa3da8f77c3b4da8",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "dear_imgui",
        build_file = "//:bazel/build_files/dear_imgui.BUILD",
        sha256 = "04943919721e874ac75a2f45e6eb6c0224395034667bf508923388afda5a50bf",
        strip_prefix = "imgui-1.90.9",
        url = "https://github.com/ocornut/imgui/archive/refs/tags/v1.90.9.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "implot",
        build_file = "//:bazel/build_files/implot.BUILD",
        sha256 = "4c20f22fbfbe4ad055f3d344581918d62cde72070b233dad75419a4334f82146",
        strip_prefix = "implot-0.15",
        url = "https://github.com/epezent/implot/archive/refs/tags/v0.15.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "meshoptimizer",
        build_file = "//:bazel/build_files/meshoptimizer.BUILD",
        sha256 = "f5bc07d7322e6292fe0afce03462b5c394d111386236f926fdc44d2aff3b854b",
        strip_prefix = "meshoptimizer-0.18",
        url = "https://github.com/zeux/meshoptimizer/archive/refs/tags/v0.18.tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "mikktspace",
        build_file = "//:bazel/build_files/mikktspace.BUILD",
        sha256 = "a612bad7f6345c0c2405ca78a921f43fd2001e4d93cc9c9d8fe2ff089c54be1a",
        strip_prefix = "mmikk-MikkTSpace-3e895b4",
        type = ".tar.gz",
        url = "https://api.github.com/repos/mmikk/MikkTSpace/tarball/3e895b49d05ea07e4c2133156cfa94369e19e409",
    )

    # TODOInvestigate how to switch to bazel_dep.
    # There's a mismatch between Bazel Central Registry BUILD file and our working BUILD file:
    # https://github.com/bazelbuild/bazel-central-registry/blob/main/modules/libzip/1.10.1/patches/add-build-file.patch
    http_archive(
        name = "zip",
        build_file = "//:bazel/build_files/zip.BUILD",
        patch_args = ["-p1"],
        patches = ["//:bazel/patches/zip.patch"],
        sha256 = "adbd787055b2d788fc505f4f8ac87c284173e6f0d17920db37c6424722451000",
        strip_prefix = "brooksmoses-zip-cfe3c1b",
        type = ".tar.gz",
        url = "https://api.github.com/repos/brooksmoses/zip/tarball/cfe3c1b7c036da1c082991cff2ec028f8816b865",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "SDL2",
        build_file = "//:bazel/build_files/SDL2.BUILD",
        patch_args = ["-p1"],
        patches = [
            "@third_party//:bazel/patches/SDL2.patch",
            "@third_party//:bazel/patches/SDL2_config.patch",
        ],
        sha256 = "a0b77c453274401dd88325ceeae8ce2b41402514081e677209057936e30f44a6",
        strip_prefix = "libsdl-org-SDL-e9fc66a",
        type = ".tar.gz",
        url = "https://api.github.com/repos/libsdl-org/SDL/tarball/e9fc66a038304be0b892b83c16d6dcf5ee36f388",
    )

    # TODO Replace with bazel_dep and git_override.
    # Currently, it appears that git_override does not support patches, which we need to add a custom BUILD file.
    http_archive(
        name = "bullet",
        build_file = "//:bazel/build_files/bullet.BUILD",
        url = "https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz",
        strip_prefix = "bullet3-3.25",
        sha256 = "c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77",
        type = ".tar.gz",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "draco",
        build_file = "//:bazel/build_files/draco.BUILD",
        patch_args = ["-p1"],
        patches = [
            "//:bazel/patches/draco.patch",
        ],
        url = "https://github.com/google/draco/archive/refs/tags/1.5.6.tar.gz",
        strip_prefix = "draco-1.5.6",
        sha256 = "0280888e5b8e4c4fb93bf40e65e4e8a1ba316a0456f308164fb5c2b2b0c282d6",
    )

    # Not in Bazel Central Registry as of 12/2024 and no MODULE.bazel in git repo.
    http_archive(
        name = "icu",
        url = "https://github.com/unicode-org/icu/archive/refs/tags/release-74-2.tar.gz",
        strip_prefix = "icu-release-74-2",
        sha256 = "27b8650a94df6f945cb3b686be3be320c2a32edf3ece2981672bf98bb3baa9e1",
    )

    # Not in Bazel Central Registry as of 12/2024.
    http_archive(
        name = "vulkan_memory_allocator",
        build_file = "//:bazel/build_files/vk_mem_alloc.BUILD",
        url = "https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator/archive/refs/tags/v3.1.0.tar.gz",
        strip_prefix = "VulkanMemoryAllocator-3.1.0",
        sha256 = "ae134ecc37c55634f108e926f85d5d887b670360e77cd107affaf3a9539595f2",
    )

    # TODOreplace with bazel_dep and git_override.
    http_archive(
        name = "hedron_compile_commands",
        sha256 = "6a0cacf3d71b406e6e2ac72a3e2648cb6b718e96d2ba3b0b8e488d90b725ef65",
        strip_prefix = "bazel-compile-commands-extractor-4f28899",
        type = ".tar.gz",
        url = "https://api.github.com/repos/hedronvision/bazel-compile-commands-extractor/tarball/4f28899228fb3ad0126897876f147ca15026151e",
    )

    # Not in Bazel Central Registry as of 12/2024 and no MODULE.bazel in git repo.
    new_git_repository(
        name = "tinyfiledialogs",
        build_file = "//:bazel/build_files/tinyfiledialogs.BUILD",
        commit = "d33dde9323a44b949902039cbcb63219a046ad8e",
        remote = "https://git.code.sf.net/p/tinyfiledialogs/code",
    )

def _non_module_repos_impl(_ctx):
    """Wrapper for converting non_module_dependencies to a module_extension."""
    non_module_dependencies()

# Create a module_extension that can be used to access the non_module_dependencies in the MODULE.bazel file.
non_module_repos = module_extension(
    implementation = _non_module_repos_impl,
)
