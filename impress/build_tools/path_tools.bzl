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

"""Utility functions for path manipulation in BUILD files."""

load("@bazel_skylib//lib:paths.bzl", "paths")

def externalize_path(ctx, path):
    """Fixes path to a external-workspace-relative path if built externally."""
    if ctx.label.workspace_name and ctx.label.workspace_name != ctx.workspace_name:
        path = paths.join("external", ctx.label.workspace_name, path)
    return path

def rlocation_path(ctx, file):
    """Fixes up Bazel's short_paths, which are relative (i.e. remove "../").

    See https://github.com/bazelbuild/bazel/issues/1462 for context.

    Args:
        ctx: The Blaze/bazel context.
        file: The File whose path should be extracted.
    Returns:
        The relative path of the file (with workspace name removed, if necessary).
    """
    if file.short_path.startswith("../"):
        # Remove
        path = file.short_path[3:]
        return path[len(ctx.label.workspace_name) + 1:]
    else:
        return file.short_path
