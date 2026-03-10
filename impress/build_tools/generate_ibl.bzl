# Copyright 2024 Google LLC
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

"""Generate cubemap mipmap levels and spherical harmonics from an environment map image"""

load("@bazel_skylib//lib:paths.bzl", "paths")

def _generate_ibl_impl(ctx):
    source_image_name = paths.split_extension(ctx.file.source_image.basename)[0]

    # Verify the size of the source image is a power of two.
    size = 1
    for _ in range(12):
        if size == ctx.attr.size:
            break
        if size > ctx.attr.size:
            fail("Size must be a power of two.")
        size = size * 2

    # Run cmgen on the source image
    command = " ".join(
        [
            ctx.executable.cmgen.path,
            "--quiet",
            "--format=rgb32f",
            ctx.file.source_image.path,
            "--size=%d" % (size),
            "--ibl-samples=%d" % (ctx.attr.ibl_samples),
            "--extract-blur=%f" % float(ctx.attr.blur_roughness),
        ],
    )

    if (ctx.attr.include_skybox_cubemap):
        command += " --extract=."

    if (not ctx.attr.exclude_lighting_cubemap):
        command += " --ibl-ld=."

    if (not ctx.attr.exclude_spherical_harmonics):
        command += " --sh-irradiance --sh-shader --sh-output=%s/sh.txt" % source_image_name

    if (ctx.attr.deterministic_output):
        # Change the timestamps on the generated files to a deterministic value, since zip includes
        # the file timestamps in the archive.
        command = " ".join(
            [
                command,
                "&&",
                "find",
                source_image_name,
                "-exec touch -t 200001010000 {} +",
            ],
        )

    # Zip the output files
    command = " ".join(
        [
            command,
            "&&",
            "cd",
            ctx.executable.zip.dirname,
            "&&",
            "zip",
            "-Xrq",
            "../../../../../" + ctx.outputs.output_path.path,
            "../../../../../" + source_image_name,
        ],
    )

    ctx.actions.run_shell(
        inputs = [ctx.file.source_image],
        outputs = [ctx.outputs.output_path],
        mnemonic = "ImpressIblGen",
        command = command,
        tools = [ctx.executable.cmgen, ctx.executable.zip],
    )

    return [
        DefaultInfo(
            files = depset([ctx.outputs.output_path]),
        ),
    ]

generate_ibl_rule = rule(
    implementation = _generate_ibl_impl,
    attrs = {
        "source_image": attr.label(allow_single_file = True, mandatory = True),
        "cmgen": attr.label(
            default = "@third_party//filament:cmgen",
            executable = True,
            cfg = "exec",
        ),
        "zip": attr.label(
            default = Label("@zip//:zip"),
            executable = True,
            cfg = "exec",
        ),
        "size": attr.int(default = 256, doc = "Size of base level cubemap faces.  Must be a power of two."),
        "ibl_samples": attr.int(default = 1024, doc = "Number of samples to use for IBL integrations."),
        "blur_roughness": attr.string(default = "0.05", doc = "Blurs the skybox cubemap before saving the faces using the roughness blur."),
        "output_path": attr.output(),
        "include_skybox_cubemap": attr.bool(default = False, doc = "Include skybox cubemap face images."),
        "exclude_lighting_cubemap": attr.bool(default = False, doc = "Exclude lighting cubemap face images."),
        "exclude_spherical_harmonics": attr.bool(default = False, doc = "Exclude spherical harmonics information"),
        "deterministic_output": attr.bool(default = True, doc = "Ensure deterministic output."),
    },
)
