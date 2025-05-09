r"""Tool to compile a MSL shader in a Filament material to LLVM bitcode.

Arguments:
    compile_metal_shader [input file] [output file] [type] [platform]

This tool is not expected to be invoked directly; it is intended to be a child
process spawned by matedit's `external-script` mode, e.g.:

    matedit -i material.cmat -o precomp.cmat -t metal external-script -- \
         compile_metal_shader --min_ios_version=15.0 --mobile-is-simulator

It expects to get four positional arguments from matedit, plus any flags
afterwards:
  * An input shader filename (MSL source code in a .metal file)
  * An output shader filename (typically something ending in .metallib)
  * Shader type (one of "vertex", "fragment", or "compute")
  * Shader platform (one of "mobile" or "desktop")

It will shell out to the Xcode metal/metallib command line tools (using xcrun)
to compile MSL source code to LLVM bitcode, and put it in a Metallib package.

We assume the shader source code was produced by Filament's `matc` and thus
will not have any #includes or dependencies other than the Metal stdlib.
"""

from collections.abc import Sequence
import os
import subprocess
import sys
import tempfile

from absl import app
from absl import flags
from absl import logging


# The metal_library rule used by Rocket and the iGMM legacy map renderer used
# -ffast-math; Filament's on-device MSL compiler does -fno-fast-math. Let's
# default to no-fast-math for now, for parity with on-device compiling.
_METAL_FAST_MATH_FLAG = flags.DEFINE_bool(
    "fast-math",
    False,
    "Pass -ffast-math to the MSL compiler.",
)

_METAL_RECORD_SOURCES_FLAG = flags.DEFINE_bool(
    "record-sources",
    False,
    "Embed sources and driver options into the output for debugging.",
)

_METAL_EMIT_LINE_TABLES_ONLY_FLAG = flags.DEFINE_bool(
    "emit-line-tables",
    False,
    "Emit line tables in the output for debugging.",
)

_METAL_MIN_IOS_VERSION_FLAG = flags.DEFINE_string(
    "min-ios-version",
    None,
    "Set minimum iOS deployment target for mobile shaders.",
)

_METAL_MIN_MACOS_VERSION_FLAG = flags.DEFINE_string(
    "min-macos-version",
    None,
    "Set minimum MacOS deployment target for desktop shaders.",
)

_METAL_MOBILE_TARGETS_SIMULATOR_FLAG = flags.DEFINE_bool(
    "mobile-is-simulator",
    False,
    "Explicitly target iOS Simulator for mobile shaders.",
)

_METAL_DIAGNOSTICS_FILE_FLAG = flags.DEFINE_string(
    "diagnostics",
    None,
    "Write diagnostics from the MSL compiler to the specified file.",
)

_XCTOOL_PROCESS_TIMEOUT_FLAG = flags.DEFINE_integer(
    "xctool-process-timeout",
    300,
    "Maximum time (in seconds) to wait for an Xcode child process to finish."
    " Set to zero to wait forever.",
    lower_bound=0,
)

_XCTOOL_STDIO_ENCODING_FLAG = flags.DEFINE_string(
    "xctool-stdio-encoding",
    None,
    "String encoding to expect for stdout/stderr from Xcode child processes."
    " Leave unset to use the default locale.",
)

_XCTOOL_ALLOW_STDERR_OUTPUT_ON_SUCCESS_FLAG = flags.DEFINE_bool(
    "xctool-allow-stderr-output-on-success",
    False,
    "By default, if an Xcode child process has an exit code of 0 but also"
    " outputs to stderr, the exit code will be set to 99. This is useful for"
    " catching warnings. Set true to disable this behavior.",
)

_XCTOOL_SUCCESSFUL_WITH_STDERR_OUTPUT_EXIT_CODE = 99


def _run_tool_with_xcrun(
    tool_cmdline: Sequence[str], sdk: str | None
) -> tuple[int, str, str]:
  """Spawns a child process to run a tool with xcrun, and blocks until it exits.

  Args:
    tool_cmdline: A binary name and command line args to be passed after xcrun.
    sdk: Optionally, an explicit SDK to pass to xcrun with --sdk. If not set, no
      explicit SDK will be set, and xcrun is expected to check the SDKROOT
      environment variable, falling back to most recent SDK if that isn't set.

  Returns:
    A tuple of (exitcode, stdout, stderr).
  """
  assert len(tool_cmdline) >= 1

  tool_name = tool_cmdline[0]
  xcrun_cmdline = ["/usr/bin/xcrun"]
  if sdk:
    xcrun_cmdline += ["--sdk", sdk]
  xcrun_cmdline += tool_cmdline

  timeout_seconds = _XCTOOL_PROCESS_TIMEOUT_FLAG.value
  if timeout_seconds <= 0:
    timeout_seconds = None

  logging.debug("Spawning child process: '%s'", " ".join(xcrun_cmdline))
  child_process = None
  try:
    child_process = subprocess.Popen(
        xcrun_cmdline,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        encoding=_XCTOOL_STDIO_ENCODING_FLAG.value,
    )

    if child_process is None:
      raise RuntimeError("Child process was not created")

    stdout, stderr = child_process.communicate(timeout=timeout_seconds)
    if stdout is None:
      stdout = ""
    if stderr is None:
      stderr = ""

    exitcode = child_process.returncode
    if exitcode is None:
      raise RuntimeError("Child process was still alive after communicate()")
    exitcode = int(exitcode)

    if exitcode != 0:
      logging.error('Tool "%s" failed with exit code %d.', tool_name, exitcode)
    if exitcode == 0 and stderr:
      # Tool exited with exit code 0 but outputted something to stderr; probably
      # warnings. Depending on flag, either fail or continue.
      if _XCTOOL_ALLOW_STDERR_OUTPUT_ON_SUCCESS_FLAG.value:
        logging.warning(
            'Tool "%s" succeeded with warnings; continuing.', tool_name
        )
      else:
        logging.error(
            'Tool "%s" succeeded with warnings; treating as a failure.',
            tool_name,
        )
        exitcode = _XCTOOL_SUCCESSFUL_WITH_STDERR_OUTPUT_EXIT_CODE

    if stdout:
      logging.debug('Tool "%s" stdout output:\n%s', tool_name, stdout)
    if stderr:
      if exitcode == 0:
        logging.warning('Tool "%s" stderr output:\n%s', tool_name, stderr)
      else:
        logging.error('Tool "%s" stderr output:\n%s', tool_name, stderr)

    return (exitcode, str(stdout), str(stderr))

  except:
    logging.error(
        "Python exception raised while waiting on child process: %s",
        str(sys.exc_info()[0]),
    )
    # Send SIGKILL to the child process, then re-raise the exception.
    if child_process is not None:
      child_process.kill()
    raise


def _get_xcrun_sdk_from_shader_platform(
    shader_platform: str | None,
) -> str | None:
  """Returns the appropriate xcrun SDK for the given shader_platform."""
  if shader_platform is None:
    return None
  elif shader_platform == "mobile":
    if _METAL_MOBILE_TARGETS_SIMULATOR_FLAG.value:
      return "iphonesimulator"
    else:
      return "iphoneos"
  elif shader_platform == "desktop":
    return "macosx"
  else:
    logging.fatal("Unexpected shader platform: %s", shader_platform)


def _get_target_copt_from_shader_platform(
    shader_platform: str | None,
) -> str | None:
  """Returns the appropriate target string for the given shader_platform and min-os-version flags."""
  if shader_platform is None:
    return None
  elif shader_platform == "mobile":
    minimum_ios_version = _METAL_MIN_IOS_VERSION_FLAG.value
    if not minimum_ios_version:
      return None
    os_tag = "air64-apple-ios%s" % minimum_ios_version
    if _METAL_MOBILE_TARGETS_SIMULATOR_FLAG.value:
      os_tag += "-simulator"
    return os_tag
  elif shader_platform == "desktop":
    minimum_macos_version = _METAL_MIN_MACOS_VERSION_FLAG.value
    if not minimum_macos_version:
      return None

    return "air64-apple-macosx%s" % minimum_macos_version
  else:
    logging.fatal("Unexpected shader platform: %s", shader_platform)

def _get_metal_compiler_copts(
    shader_platform: str | None,
) -> Sequence[str]:
  """Builds a copts list for the MSL compiler based on flags and shader_platform."""
  copts = [
      "-Os",
      "-arch",
      "air64",
      "-emit-llvm",
      "-c",
  ]

  target_copt = _get_target_copt_from_shader_platform(shader_platform)
  if target_copt:
    copts += ["-target", target_copt]

  if _METAL_FAST_MATH_FLAG.value:
    copts += ["-ffast-math"]
  else:
    copts += ["-fno-fast-math"]

  if _METAL_EMIT_LINE_TABLES_ONLY_FLAG.value:
    copts += ["-gline-tables-only"]

  if _METAL_RECORD_SOURCES_FLAG.value:
    copts += ["-frecord-sources"]

  diagnostics_filename = _METAL_DIAGNOSTICS_FILE_FLAG.value
  if diagnostics_filename:
    copts += ["-serialize-diagnostics", diagnostics_filename]

  return copts


def _run_metal_compiler(
    input_msl_filename: str,
    output_air_filename: str,
    shader_platform: str | None,
    extra_copts: Sequence[str] | None = None,
) -> int:
  """Shells out to `xcrun metal` to compile an MSL to AIR.

  Args:
    input_msl_filename: The file containing the input shader source. (This
      filename needs to end in .metal for proper operation.)
    output_air_filename: The file that should receive the MetalIR output.
    shader_platform: Optional. The shader platform ("mobile" or "desktop").
    extra_copts: Optional. Any additional options to pass to clang.

  Returns:
    The child process' exit code.
  """
  if not os.path.exists(input_msl_filename):
    logging.fatal("Input MSL file does not exist: %s", input_msl_filename)

  # Check the input name -- clang cares about it. This run will probably fail
  # with a warning about '"linker" input not used'.
  if os.path.splitext(input_msl_filename)[1] != ".metal":
    logging.warning(
        "Input MSL file should end in .metal: %s", input_msl_filename
    )

  cmdline = ["metal"]
  cmdline += _get_metal_compiler_copts(shader_platform)
  if extra_copts:
    cmdline += extra_copts

  cmdline += ["-o", output_air_filename, input_msl_filename]

  exitcode, _, _ = _run_tool_with_xcrun(
      tool_cmdline=cmdline,
      sdk=_get_xcrun_sdk_from_shader_platform(shader_platform),
  )
  return exitcode


def _run_metal_linker(
    input_air_filename: str,
    output_metallib_filename: str,
    shader_platform: str | None,
    extra_linkopts: Sequence[str] | None = None,
) -> int:
  """Shells out to `xcrun metallib` to link an AIR object file into a library.

  Args:
    input_air_filename: The file containing MetalIR object code.
    output_metallib_filename: The file that should receive the MetalLib output.
    shader_platform: Optional. The shader platform ("mobile" or "desktop").
    extra_linkopts: Optional. Any additional options to pass to clang.

  Returns:
    The child process' exit code.
  """
  if not os.path.exists(input_air_filename):
    logging.fatal("Input AIR file does not exist: %s", input_air_filename)

  cmdline = [
      "metallib",
      "-split-module",
  ]

  if extra_linkopts:
    cmdline += extra_linkopts

  cmdline += ["-o", output_metallib_filename, input_air_filename]

  exitcode, _, _ = _run_tool_with_xcrun(
      tool_cmdline=cmdline,
      sdk=_get_xcrun_sdk_from_shader_platform(shader_platform),
  )
  return exitcode


def _compile_metal_shader(
    input_msl_filename: str,
    output_metallib_filename: str,
    shader_platform: str | None,
) -> tuple[str | None, int]:
  """Compiles and links the specified MSL shader to a .metallib.

  Args:
    input_msl_filename: The file containing the input shader source. (This
      filename needs to end in .metal for proper operation.)
    output_metallib_filename: The file that should receive the MetalLib output.
    shader_platform: Optional. The shader platform ("mobile" or "desktop"), if
      known.

  Returns:
    A tuple of (tool_name_str, exit_code). If either of the tools failed, the
    first field will be the failing tool ("metal" or "metallib"), and the second
    will bethe child process' exit code. On success, returns (None, 0).
  """
  shader_filename = os.path.splitext(os.path.basename(input_msl_filename))[0]

  # The Metal compiler will delete the temporary output file on failure, which
  # means that the NamedTemporaryFile cleanup (upon exiting the `with` block)
  # will throw a FileNotFoundError. So, we need to wrap this whole thing in a
  # try-except to safely catch that error at the appropriate time.
  def _do_precompile_shader(
      input_msl_filename: str,
      output_metallib_filename: str,
      temporary_air_filename: str,
      shader_platform: str | None,
  ) -> tuple[str | None, int]:
    exitcode = _run_metal_compiler(
        input_msl_filename, temporary_air_filename, shader_platform
    )
    if exitcode != 0:
      return ("metal", exitcode)

    exitcode = _run_metal_linker(
        temporary_air_filename, output_metallib_filename, shader_platform
    )
    if exitcode != 0:
      return ("metallib", exitcode)

    return (None, 0)

  result = None
  file_not_found_is_safe_to_ignore = False
  try:
    with tempfile.NamedTemporaryFile(
        prefix=shader_filename, suffix=".air"
    ) as air_fd:
      temporary_air_filename = air_fd.name
      result = _do_precompile_shader(
          input_msl_filename,
          output_metallib_filename,
          temporary_air_filename,
          shader_platform,
      )
      # If an exception is thrown from the inner function, that should be
      # re-raised. If it gets thrown from the NamedTemporaryFile() cleanup,
      # on the other hand, we can ignore it after this line.
      file_not_found_is_safe_to_ignore = True

  except FileNotFoundError:
    if file_not_found_is_safe_to_ignore:
      pass
    else:
      raise

  if result[1] == 0:
    logging.debug(
        'Successfully compiled "%s". Size change: %d -> %d',
        os.path.basename(input_msl_filename),
        os.path.getsize(input_msl_filename),
        os.path.getsize(output_metallib_filename),
    )

  return result


def main(argv: Sequence[str]) -> None:
  # We expect to get exactly five unparsed args from matedit:
  #   0: binary (ignored)
  #   1: input file
  #   2: output file
  #   3: shader stage, one of "vertex"/"fragment"/"compute" (currently ignored)
  #   4: shader platform, one of "mobile"/"desktop"
  if len(argv) != 5:
    raise app.UsageError("Not enough command-line arguments.")

  _, exitcode = _compile_metal_shader(
      input_msl_filename=argv[1],
      output_metallib_filename=argv[2],
      shader_platform=argv[4],
  )
  sys.exit(exitcode)


if __name__ == "__main__":
  app.run(main)
