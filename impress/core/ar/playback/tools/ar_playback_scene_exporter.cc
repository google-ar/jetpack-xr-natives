// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// A tool to export all data required by an imp ar-playback-session into one
// protobuf.
//
// Arguments:
//
// scene_data_path - a path to the derived data proto
//
// video_url - A URL to a .mp4 or other supported video file
//
// video_data_path - A path to .mp4 or other supported video data to be embedded
// in the protobuf.
//
// lightprobe_url - A URL to a lightprobe.sfb file exported from the light_probe
// rule (see google3/third_party/arcore/ar/sceneform/converter/converter.bzl)
//
// lightprobe_data - a path to a .sfb light_probe (see
// google3/third_party/arcore/ar/sceneform/converter/converter.bzl)
//
// output_dir_path - which contains all the export data in one playback_scene
// proto.

#include <stdio.h>

#include <fstream>

#include "zetasql/base/commandlineflags.h"
#include "zetasql/base/init_google.h"
#include "zetasql/base/logging_extensions.h"
#include "file/base/file.h"
#include "file/base/filebytestream.h"
#include "file/base/filesystem.h"
#include "file/base/helpers.h"
#include "absl/flags/flag.h"
#include "core/ar/playback/playback_scene.pb.h"
#include "third_party/arcore/proto/video/ar_derived_data.pb.h"
#include "util/task/status.h"

ABSL_FLAG(std::string, scene_data_path, "", "Path to derived data protobuf.");
ABSL_FLAG(std::string, video_url, "", "Url of mp4/video file.");
ABSL_FLAG(std::string, video_data_path, "", "Path to mp4/video file.");
ABSL_FLAG(std::string, light_probe_url, "",
          "Url of .sfb HDR lighting (see the 'light_probe' rule at "
          "google3/third_party/arcore/ar/sceneform/converter/converter.bzl).");
ABSL_FLAG(std::string, light_probe_path, "",
          "Path to .sfb light_probe data file.");
ABSL_FLAG(std::string, output_dir_path, "", "Path to output proto.");

int main(int argc, char** argv) {
  absl::SetFlag(&FLAGS_alsologtostderr, true);
  InitGoogle(argv[0], &argc, &argv, true);
  ParseCommandLineFlags(&argc, &argv, true);

  ::imp::ArPlaybackScene playback_scene;

  // Assigns the scene data protobuf.
  const std::string scene_data_path = absl::GetFlag(FLAGS_scene_data_path);
  ::ar::video::ARSession ar_metadata;
  
  *playback_scene.mutable_scene_data() = ar_metadata;

  // Assigns the Video URL.
  const std::string video_url = absl::GetFlag(FLAGS_video_url);
  const std::string video_data_path = absl::GetFlag(FLAGS_video_data_path);
  if (!video_url.empty()) {
    playback_scene.set_video_url(video_url);
  }

  // Assigns the Video buffer data.
  if (!video_data_path.empty()) {
    std::string video_data;
    
    playback_scene.set_video_data(std::move(video_data));

    
  }

  // Assigns the HDR URL
  const std::string light_probe_url = absl::GetFlag(FLAGS_light_probe_url);
  const std::string light_probe_path = absl::GetFlag(FLAGS_light_probe_path);
  if (!light_probe_url.empty()) {
    playback_scene.set_light_probe_url(light_probe_url);
  }

  // Assigns the HDR buffer data.
  if (!light_probe_path.empty()) {
    std::string light_probe_data;
    
    playback_scene.set_light_probe_data(std::move(light_probe_data));

    
  }

  std::string out_path = absl::GetFlag(FLAGS_output_dir_path);
  std::ofstream out_file(out_path.c_str());

  std::string serialized;
  playback_scene.SerializeToString(&serialized);

  out_file << serialized;
  out_file.close();

  return 0;
}
