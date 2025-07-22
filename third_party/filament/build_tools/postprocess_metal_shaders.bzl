"""Rule to postprocess a compiled material using matedit, converting all MSL shaders to LLVM bitcode."""

load("//third_party/bazel_rules/apple_support/lib:apple_support.bzl", "apple_support")
load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@third_party//filament:filament.bzl", "by_backend")

def _platform_is_simulator(ctx):
    return (ctx.fragments.apple.single_arch_platform.name_in_plist == "iPhoneSimulator")

def _get_minimum_ios_version(ctx):
    if ctx.attr.minimum_ios_version:
        return ctx.attr.minimum_ios_version

    return ctx.attr._xcode_config[apple_common.XcodeVersionConfig].minimum_os_for_platform_type(
        apple_common.platform_type.ios,
    )

def _get_minimum_macos_version(ctx):
    if ctx.attr.minimum_macos_version:
        return ctx.attr.minimum_macos_version

    return ctx.attr._xcode_config[apple_common.XcodeVersionConfig].minimum_os_for_platform_type(
        apple_common.platform_type.macos,
    )

def _emit_matedit_action(ctx):
    # The argument list starts with matedit args.
    args = ctx.actions.args()
    args.add("-i", ctx.file.input_material)
    args.add("-o", ctx.outputs.output_material)
    if ctx.attr.preserve_text_shaders:
        args.add("-p")
    args.add("-t", "metal")
    args.add("external-compile", "--")

    # Then, add path to the precompile tool, and its args.
    args.add(ctx.executable.script_tool.path)

    if ctx.attr.allow_warnings:
        args.add("--xctool-allow-stderr-output-on-success")

    minimum_ios_version = _get_minimum_ios_version(ctx)
    if minimum_ios_version:
        args.add("--min-ios-version=%s" % minimum_ios_version)

    minimum_macos_version = _get_minimum_macos_version(ctx)
    if minimum_macos_version:
        args.add("--min-macos-version=%s" % minimum_macos_version)

    if _platform_is_simulator(ctx):
        args.add("--mobile-is-simulator")

    if ctx.attr.emit_line_tables:
        args.add("--emit-line-tables")

    if ctx.attr.record_sources:
        args.add("--record-sources")

    if ctx.attr.script_verbose:
        args.add("--verbosity", "1")

    if ctx.attr.fast_math:
        args.add("--fast-math")

    apple_support.run(
        actions = ctx.actions,
        apple_fragment = ctx.fragments.apple,
        xcode_config = ctx.attr._xcode_config[apple_common.XcodeVersionConfig],
        executable = ctx.executable.matedit_tool,
        arguments = [args],
        tools = [ctx.executable.matedit_tool, ctx.executable.script_tool],
        inputs = depset(direct = [ctx.file.input_material]),
        outputs = [ctx.outputs.output_material],
        exec_group = "macos",
    )

def _postprocess_metal_shaders_impl(ctx):
    # The postprocess_metal_shaders() macro below will run before select() objects can be evaluated,
    # meaning that it won't be able to know which API is going to be passed to matc. So, instances
    # of this rule may be generated for materials that end up being OpenGL or Vulkan only.
    #
    # By the time the impl actually gets evaluated, the by_backend selects() should be resolved.
    # If the backend wasn't Metal, then the input material isn't expected to have any Metal shaders
    # in it, so skip generating the matedit action and just create a symlink pointing at the input.
    #
    # (Technically, matedit could run over a .cmat containing OpenGL/Vulkan shaders, and should
    # produce output identical to the original. But we want to avoid the expense of finding
    # a MacOS build host to run that action unless it's absolutely needed.)
    if not ctx.attr.backend_is_metal:
        ctx.actions.symlink(
            output = ctx.outputs.output_material,
            target_file = ctx.file.input_material,
        )
    else:
        _emit_matedit_action(ctx)

    return [
        DefaultInfo(
            files = depset([ctx.outputs.output_material]),
        ),
    ]

_postprocess_metal_shaders_rule = rule(
    attrs = dicts.add(
        apple_support.action_required_attrs(),
        {
            "input_material": attr.label(allow_single_file = True),
            "output_material": attr.output(),
            "backend_is_metal": attr.bool(),
            "minimum_ios_version": attr.string(),
            "minimum_macos_version": attr.string(),
            "emit_line_tables": attr.bool(),
            "record_sources": attr.bool(),
            "fast_math": attr.bool(),
            "preserve_text_shaders": attr.bool(),
            "allow_warnings": attr.bool(default = True),
            "matedit_tool": attr.label(
                default = Label("@third_party//filament:matedit"),
                executable = True,
                cfg = config.exec("macos"),
            ),
            "script_tool": attr.label(
                default = Label("@third_party//filament/build_tools:compile_metal_shader"),
                executable = True,
                cfg = config.exec("macos"),
            ),
            "script_verbose": attr.bool(),
        },
    ),
    fragments = ["apple"],
    exec_groups = {
        "default": exec_group(),
        "macos": exec_group(
            exec_compatible_with = ["@platforms//os:macos"],
        ),
    },
    implementation = _postprocess_metal_shaders_impl,
)

def _backend_is_metal_or_default(backend_is_metal):
    if backend_is_metal == None:  # Using == instead of "is" is a Starlark-ism. (broken link)
        return by_backend(
            metal = True,
            opengl = False,
            vulkan = False,
        )

    return backend_is_metal

def postprocess_metal_shaders(
        name,
        input_material,
        output_material,
        minimum_ios_version = None,
        minimum_macos_version = None,
        backend_is_metal = None,
        **kwargs):
    """Postprocesses a Filament compiled material (.cmat) to compile all Metal shaders inside it.

    By default, a compiled material (produced by `matc`) stores any Metal shaders as plain-text MSL
    source code, which get compiled on-device at runtime.

    If the targeted backend is Metal, this rule uses `matedit` to extract all MSL shaders from the
    input .cmat, compiles each one to LLVM bitcode and packages it as an Apple .metallib, and
    outputs a new .cmat file containing those metallibs and no MSL source. (This operation requires
    a Mac build host with Xcode.)

    If the targeted backend is not Metal, this rule just emits a symlink pointing at the input file.

    Args:
        name: The build target name
        input_material: A label that yields the input .cmat to be postprocessed.
        output_material: The output filename.
        minimum_ios_version: Optional. A minimum iOS version to target for mobile shaders.
          If not specified, uses the minimum version from the build environment's Xcode config.
        minimum_macos_version: Optional. A minimum MacOS version to target for desktop shaders.
          If not specified, uses the minimum version from the build environment's Xcode config.
        backend_is_metal: Optional. If left to the default value of None, the Filament backend will
          be determined at rule evaluation time, and the rule only runs the matedit tool if the
          backend is Metal. Pass a boolean or select() object to override this behavior -- passing
          True will emit the matedit action on all materials regardless of backend, while False
          will make this rule always emit symlinks (an effective no-op).
        **kwargs: Passed along to the rule. Meant for common rule attributes (visibility, etc.)
          but can also be used to toggle extra options for debugging. See rule definition above.
    """
    _postprocess_metal_shaders_rule(
        name = name,
        input_material = input_material,
        output_material = output_material,
        minimum_ios_version = minimum_ios_version,
        minimum_macos_version = minimum_macos_version,
        backend_is_metal = _backend_is_metal_or_default(backend_is_metal),
        **kwargs
    )
