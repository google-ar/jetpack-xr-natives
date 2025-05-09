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

"""Background scenes helper functions for exporting data to protobufs"""

load("//core/resources:converter.bzl", "light_probe")

def _imp_synthetic_data(name, scene_data, frame_data):
    native.filegroup(
        name = name + "_scene_data_group",
        srcs = [
            scene_data,
        ],
    )
    native.filegroup(
        name = name + "_frame_data_group",
        srcs = [
            frame_data,
        ],
    )

    native.genrule(
        name = name,
        srcs = [":" + name + "_scene_data_group", ":" + name + "_frame_data_group"],
        outs = [name + ".pb"],
        cmd = "$(location //third_party/arcore/ar/video/tools:" +
              "csv_to_ar_derived_data_proto) -- " +
              " -static_data_path $(location " + ":" + name + "_scene_data_group" + ")" +
              " -frame_data_path $(location " + ":" + name + "_frame_data_group" + ")" +
              " -output_dir_path \"$@\"",
        tools = ["//third_party/arcore/ar/video/tools:" +
                 "csv_to_ar_derived_data_proto"],
    )

def _imp_postcapture_video_to_playback_data(name, postcapture_video):
    native.genrule(
        name = name,
        srcs = [postcapture_video],
        outs = [name + ".pb"],
        cmd = "$(location //third_party/arcore/ar/video/tools:" +
              "postcapture_to_ar_derived_data_proto) -- " +
              " -dataset_path $(location " + postcapture_video + ")" +
              " -output_proto_path \"$@\"",
        tools = ["//third_party/arcore/ar/video/tools:" +
                 "postcapture_to_ar_derived_data_proto"],
    )

def _imp_ar_playback_scene_data(
        name,
        playback_data,
        video_url = "",
        video_src = "",
        light_probe_url = "",
        light_probe_src = ""):
    if video_url and video_src:
        fail("Expected either video_url ({0}) or video_src ({1}), not both.".format(video_url, video_src))
    if light_probe_url and light_probe_src:
        fail("Expected either light_probe_url ({0}) or light_probe_src ({1}), not both.".format(light_probe_url, light_probe_src))

    srcs = [playback_data]

    video_url_arg = ""
    video_data_arg = ""
    video_data_src = ""
    video_file = ""

    # Creates a video url argument.
    if video_url:
        video_url_arg = "-video_url " + video_url
        # Creates an argument referencing the video data file group if video data
        # was provided.

    elif video_src:
        video_file = name + "_video_data"
        native.filegroup(
            name = video_file,
            srcs = [
                video_src,
            ],
        )
        video_data_arg = " -video_data_path $(location " + ":" + video_file + ")"
        video_data_src = ":" + video_file

    light_probe_url_arg = ""
    light_probe_data_src = ""
    light_probe_buffer_arg = ""
    light_probe_rule_name = ""

    # Creates an argument referencing an hdr url if hdr url was provided.
    if light_probe_url:
        light_probe_url_arg = "-light_probe_url " + light_probe_url
        # Creates an argument referencing hdr data if the data is available.

    elif light_probe_src:
        light_probe_rule_name = name + "_hdr_data"
        light_probe(
            name = light_probe_rule_name,
            intensity = "1",
            source_image = light_probe_src,
        )

        light_probe_buffer_arg = " -light_probe_path $(location " + ":" + light_probe_rule_name + ")"
        light_probe_data_src = ":" + light_probe_rule_name

    # Aggregates source data together.
    if video_data_src:
        srcs.append(video_data_src)
    if light_probe_data_src:
        srcs.append(light_probe_data_src)

    # Calls native command for packaging all the various data pieces into one
    # protobuf.
    export_name = name
    native.genrule(
        name = name,
        srcs = srcs,
        outs = [":" + export_name + ".pb"],
        cmd = "$(location @com_google_impress//core/ar/playback/" +
              "tools:ar_playback_scene_exporter) -- " +
              " -scene_data_path $(location " + ":" + playback_data + ")" +
              " " + video_url_arg +
              " " + video_data_arg +
              " " + light_probe_url_arg +
              " " + light_probe_buffer_arg +
              " -output_dir_path $(location :" + export_name + ".pb)",
        tools = [
            "@com_google_impress//core/ar/playback/tools:ar_playback_scene_exporter",
        ],
    )

def imp_ar_playback_scene_data(
        name,
        scene_data = "",
        frame_data = "",
        video_url = "",
        video_src = "",
        light_probe_url = "",
        light_probe_src = ""):
    """Automates exporting all background scenes data as imp_ar_playback_scene_data.proto.

    Example usage:
    imp_ar_playback_scene_data(
    name = "scene_export",
    frame_data = "cam_pos.csv",
    scene_data = "cam_intrinsics_plane.csv",

    # Exports either video_url or video_src but not both.
    video_url = "http://mydata/made_up/fake.mp4",
    video_src = "room_2_test_final.mp4",

    # Exports either light_probe_url or light_probe_src but not both.
    light_probe_url = "http://mydata/made_up/fake.hdr",
    light_probe_src = "room_2_test_final.hdr",
    )

    To reference the output use:
    srcs = [
    ...
    "//path/to/build/file/dir:scene_export",
    ...
    ]

    Args:
      name: The name of the generated imp_ar_playback_scene_data data asset. The output name will be this name + 'BackgroundScene.pb' or kSceneExportBackgroundScenePb when referenced as an asset definition.
      frame_data: camera pos .csv file
      scene_data: camera intrinsics and plane data .csv file
      video_url: a URL to supported video format file.
      video_src: a path to supported video format file.
      light_probe_url: a URL to .sfb light_probe data.
      light_probe_src: a path to .hdr or .exr file (the light probe source data).
    """

    if video_url and video_src:
        fail("Expected either video_url ({0}) or video_src ({1}), not both.".format(video_url, video_src))
    if light_probe_url and light_probe_src:
        fail("Expected either light_probe_url ({0}) or light_probe_src ({1}), not both.".format(light_probe_url, light_probe_src))

    # Builds synthetic AR data protobuf.
    synth_data = name + "_synthetic_data"
    _imp_synthetic_data(
        name = synth_data,
        scene_data = scene_data,
        frame_data = frame_data,
    )

    _imp_ar_playback_scene_data(
        name = name,
        playback_data = synth_data,
        video_url = video_url,
        video_src = video_src,
        light_probe_url = light_probe_url,
        light_probe_src = light_probe_src,
    )

def imp_ar_playback_data_from_postcapture_video(
        name,
        postcapture_video,
        video_url = "",
        video_src = "",
        light_probe_url = "",
        light_probe_src = ""):
    """Automates exporting all background scenes data as imp_ar_playback_scene_data.proto.

    Example usage:
    imp_ar_playback_scene_data(
    name = "scene_export",
    postcapture_video = "my_capture_video.mp4",
    # Exports either video_url or video_src but not both.
    video_url = "http://mydata/made_up/fake.mp4",
    video_src = "my_capture_video.mp4",

    # Exports either light_probe_url or light_probe_src but not both.
    light_probe_url = "http://mydata/made_up/fake.hdr",
    light_probe_src = "room_2_test_final.hdr",
    )

    To reference the output use:
    srcs = [
    ...
    "//path/to/build/file/dir:scene_export",
    ...
    ]

    Args:
      name: The name of the generated imp_ar_playback_scene_data data asset. The output name will be this name + 'BackgroundScene.pb' or kSceneExportBackgroundScenePb when referenced as an asset definition.
      postcapture_video: A postcapture generated video.
      video_url: a URL to supported video format file.
      video_src: a path to supported video format file.
      light_probe_url: a URL to .sfb light_probe data.
      light_probe_src: a path to .hdr or .exr file (the light probe source data).
    """

    # Generates playback data from a postcapture video.
    playback_data = name + "_playback_data"
    _imp_postcapture_video_to_playback_data(
        name = playback_data,
        postcapture_video = postcapture_video,
    )

    # Generates the impress playback proto.
    _imp_ar_playback_scene_data(
        name = name,
        playback_data = playback_data,
        video_url = video_url,
        video_src = video_src,
        light_probe_url = light_probe_url,
        light_probe_src = light_probe_src,
    )
