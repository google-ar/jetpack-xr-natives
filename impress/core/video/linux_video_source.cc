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

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/media/media_asset.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/resources/resource_manager.h"
#include "core/video/video_source.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "util/regexp/re2/re2.h"
#include "util/task/status_builder.h"
#include "mediapipe/framework/port/status_macros.h"
#include "video/common/subprocess/ffmpeg/ffmpeg.h"
#include "video/common/subprocess/ffmpeg/ffmpeg.pb.h"
#include "video/common/temp_filename_deleter.h"
#include "video/file/shared/pipe/pipe_reader.h"

namespace imp::video {
namespace {

// Memory size limit of ffmpeg codec. -1 means unlimited memory size.
constexpr int kMaxSize = -1;
constexpr absl::string_view kPixelFormatRgba = "rgba";

using ::video_subprocess::FfmpegParams;

// Video source implementation for video playback on Linux. Uses ffmpeg to fetch
// and pass raw pixel data in rgba format to Filament.
// NOTE: Audio playback within a video is not currently supported on Linux.
class LinuxVideoSource : public VideoSource {
 public:
  explicit LinuxVideoSource(BaseView* view) : VideoSource(view) {}
  ~LinuxVideoSource() override;
  Future<absl::Status> Load(const media::MediaAsset* media_asset) override;
  absl::Status LoadSync(const media::MediaAsset* media_asset) override;
  // Loads the asset at the given URL asynchronously.
  Future<absl::Status> Load(absl::string_view url);
  absl::Status Play() override;
  absl::Status Pause() override;
  absl::Status Stop() override;
  absl::Status SetPlaybackSpeed(float speed) override;
  absl::Status SeekTo(float seconds, SeekType seek_type) override;
  absl::Status SetLoopCount(int loop) override;
  absl::Status SetVolume(float volume) override;
  absl::StatusOr<absl::Duration> GetDuration() const override;
  absl::StatusOr<absl::Duration> GetPlaybackTime() const override;
  absl::StatusOr<int> GetLoopCount() const override;
  media::MediaSource::State GetState() const override;
  uint2 GetVideoSize() const override;
  MediaColorSpace GetColorSpace() const override;
  MediaStereoMode GetStereoMode() const override;
  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;
  void SetOnSeekCompleteCallback(std::function<void()> callback) override;
  void SetOnBufferingCallback(
      std::function<void(BufferingState)> callback) override;
  // This needs to be called every frame to update the video texture with the
  // next video frame.
  void UpdateVideoTexture(filament::Texture* texture,
                          absl::Duration frame_delta) override;
  absl::StatusOr<Texture*> CreateVideoTexture() override;
  absl::StatusOr<BorrowedTexturePtr> BorrowVideoTextureImpl(
      SmallSourceLocation loc) override;

 private:
  struct VideoMetadata {
    // Width of the loaded video.
    uint32_t width;
    // Height of the loaded video.
    uint32_t height;
    // Expected duration of a video frame.
    // This is calculated from fps value.
    absl::Duration frame_duration;
    // The stereo encoding type of the video.
    int stereo_mode;
  };

  absl::Status RunFfmpegForVideo(absl::string_view filename);
  absl::Status InitVideoMetadata();
  absl::Status InitFfmpegAndOutputFifo();
  // Creates and starts the named pipe and a new ffmpeg subprocess.
  absl::StatusOr<std::unique_ptr<video_subprocess::Ffmpeg>> StartFfmpeg(
      video_file::PipeReader* out_pipe);
  // Stops after finishing pending I/O if finish_pending is true; otherwise
  // cancels immediately and stops the ffmpeg subprocess.
  void StopIO(bool finish_pending = true);
  void ResetFrameTime();
  void UpdateFrameTime(absl::Duration delta_time);
  // Returns true if the frame should be updated to match expected video fps.
  bool ShouldUpdateFrame();
  // Called when playback is complete.
  void OnPlaybackComplete();

  media::MediaSource::State state_ = State::kReady;
  uint32_t loop_count_ = 0;
  uint32_t current_loop_ = 0;
  std::function<void()> on_playback_complete_callback_;
  // Information of currently loaded video.
  VideoMetadata video_metadata_;
  // Elapsed duration of current video frame.
  absl::Duration current_frame_elapsed_time_ = absl::ZeroDuration();
  absl::Duration current_playback_time_ = absl::ZeroDuration();
  std::unique_ptr<video_subprocess::Ffmpeg> ffmpeg_;
  FfmpegParams ffmpeg_params_;
  std::unique_ptr<video_file::PipeReader> output_fifo_;
  TempFilenameDeleter input_downloaded_url_;
  TempFilenameDeleter output_fifo_name_;
  std::string rgba_buffer_;
  float playback_speed_ = 1.0f;

  OwnedTexturePtr texture_;
};

LinuxVideoSource::~LinuxVideoSource() { StopIO(false); }

Future<absl::Status> LinuxVideoSource::Load(
    const media::MediaAsset* media_asset) {
  return Future<absl::Status>::Schedule(
      [this, media_asset]() {
        return Future<absl::Status>(LoadSync(media_asset));
      },
      Executor::Type::kBackground);
}

absl::Status LinuxVideoSource::LoadSync(const media::MediaAsset* media_asset) {
  input_downloaded_url_ = TempFilenameDeleter("-input-downloaded-url");
  auto save_status = SaveFile(input_downloaded_url_.get(),
                              media_asset->GetData(), media_asset->GetSize());
  if (!save_status.ok()) {
    return save_status;
  }

  ResetFrameTime();
  return RunFfmpegForVideo(input_downloaded_url_.get());
}

Future<absl::Status> LinuxVideoSource::Load(absl::string_view url) {
  // Ffmpeg cannot stream from remote assets, therefore we should download it
  // and then play it from the local file.
  if (resources::ResourceManager::IsRemoteUrl(url)) {
    return view_->GetAssetManager().LoadMedia(url).Then(
        [this](AssetPtr<MediaAsset> media) -> Future<absl::Status> {
          return Future<absl::Status>(LoadSync(media.Get()));
        },
        Executor::Type::kBackground);
  } else {
    ResetFrameTime();
    BufferAccess access;
    if (LoadBinary(url, &access).ok()) {
      return Future<absl::Status>::Schedule(
          [this, url]() -> absl::Status { return RunFfmpegForVideo(url); });
    } else {
      // If the file cannot be opened by fopen directly, we need to load it into
      // a location that is before ffmpeg can read from it.
      return view_->GetAssetManager().LoadMedia(url).Then(
          [this](AssetPtr<MediaAsset> media) -> absl::Status {
            return LoadSync(media.Get());
          });
    }
  }
}

absl::Status LinuxVideoSource::RunFfmpegForVideo(absl::string_view filename) {
  output_fifo_name_ = TempFilenameDeleter("-videopipe-output");

  FfmpegParams::Input* input = ffmpeg_params_.add_input();
  input->set_file_name(filename);

  FfmpegParams::Output* output = ffmpeg_params_.add_output();
  output->set_file_name(output_fifo_name_.get());
  FfmpegParams::Video* output_video = output->add_video();
  output_video->set_codec(FfmpegParams::CODEC_LIB_RAWVIDEO);

  // First, run ffmpeg to get yuv4mpeg output to fill VideoMetadata
  // TODO: Investigate other ways of extracting metadata to improve
  // this.
  output->mutable_container()->set_format(FfmpegParams::FORMAT_ID_YUV4MPEGPIPE);
  MP_RETURN_IF_ERROR(InitVideoMetadata());

  // Update params and run ffmpeg to get raw pixel data for each frame.
  output->mutable_container()->set_format(FfmpegParams::FORMAT_ID_RAWVIDEO);
  output_video->set_pix_fmt(kPixelFormatRgba);

  return InitFfmpegAndOutputFifo();
}

absl::Status LinuxVideoSource::InitFfmpegAndOutputFifo() {
  output_fifo_ =
      std::make_unique<video_file::PipeReader>(output_fifo_name_.get());
  auto ffmpeg_status_or = StartFfmpeg(output_fifo_.get());
  MP_ASSIGN_OR_RETURN(ffmpeg_, std::move(ffmpeg_status_or));
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::InitVideoMetadata() {
  auto yuv4mpeg_output =
      std::make_unique<video_file::PipeReader>(output_fifo_name_.get());

  absl::StatusOr<std::unique_ptr<video_subprocess::Ffmpeg>> ffmpeg_status_or =
      StartFfmpeg(yuv4mpeg_output.get());
  MP_RETURN_IF_ERROR(ffmpeg_status_or.status());

  std::string video_info;
  if (yuv4mpeg_output->ReadBytes(50, &video_info) < 0) {
    return absl::InternalError("Failed to fetch video metadata");
  }

  if (std::string width_result;
      RE2::PartialMatch(video_info, "W(\\d+)", &width_result)) {
    video_metadata_.width = std::stof(width_result);
  } else {
    return util::InternalErrorBuilder() << "Could not determine width of video "
                                           "from available yuv4mpeg data: "
                                        << video_info;
  }

  // TODO: Implement stereo mode retrieval for LinuxVideoSource.
  video_metadata_.stereo_mode = -1;

  if (std::string height_result;
      RE2::PartialMatch(video_info, "H(\\d+)", &height_result)) {
    video_metadata_.height = std::stof(height_result);
  } else {
    return absl::InternalError(absl::StrFormat(
        "Could not determine height of video from available yuv4mpeg data: %s",
        video_info));
  }

  if (std::string fps_numerator, fps_denominator; RE2::PartialMatch(
          video_info, "F(\\d+):(\\d+)", &fps_numerator, &fps_denominator)) {
    video_metadata_.frame_duration =
        absl::Seconds(std::stof(fps_denominator) / std::stof(fps_numerator));
  } else {
    return absl::InternalError(absl::StrFormat(
        "Could not determine fps of video from available yuv4mpeg data: %s",
        video_info));
  }

  ffmpeg_status_or.value()->Cancel();
  yuv4mpeg_output->Stop();
  yuv4mpeg_output->Delete();
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<video_subprocess::Ffmpeg>>
LinuxVideoSource::StartFfmpeg(video_file::PipeReader* out_pipe) {
  if (!out_pipe->Create()) {
    return absl::InternalError("Couldn't create a pipe");
  }

  video_subprocess::SubprocessConfig config;
  config.set_max_data_size(kMaxSize);
  config.set_max_file_size(kMaxSize);

  auto ffmpeg = std::make_unique<video_subprocess::Ffmpeg>();
  if (!ffmpeg->Init(config, ffmpeg_params_)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Failed to initialize ffmpeg: %s", ffmpeg_params_.DebugString()));
  }
  if (!ffmpeg->Start()) {
    return absl::InternalError(
        absl::StrFormat("Ffmpeg failed during Start() (exit code %f) : %s",
                        ffmpeg->ExitCode(), ffmpeg_params_.DebugString()));
  }

  if (!out_pipe->Open()) {
    return absl::InternalError("Couldn't open pipe to ffmpeg process");
  }
  return ffmpeg;
}

void LinuxVideoSource::StopIO(bool finish_pending) {
  state_ = State::kStopped;
  if (output_fifo_) {
    if (finish_pending) {
      output_fifo_->Stop();
    } else {
      output_fifo_->Cancel();
    }
  }
  if (ffmpeg_) {
    ffmpeg_->Cancel();
    ffmpeg_.reset();
  }
  if (output_fifo_) {
    output_fifo_->Delete();
    output_fifo_.reset();
  }
}

void LinuxVideoSource::ResetFrameTime() {
  current_playback_time_ = absl::ZeroDuration();
  current_frame_elapsed_time_ = absl::ZeroDuration();
}

void LinuxVideoSource::UpdateFrameTime(absl::Duration delta_time) {
  delta_time *= playback_speed_;
  current_playback_time_ += delta_time;
  current_frame_elapsed_time_ += delta_time;
}

bool LinuxVideoSource::ShouldUpdateFrame() {
  if (current_frame_elapsed_time_ >= video_metadata_.frame_duration) {
    current_frame_elapsed_time_ -= video_metadata_.frame_duration;
    return true;
  }
  return false;
}

absl::StatusOr<Texture*> LinuxVideoSource::CreateVideoTexture() {
  texture_ = view_->GetTextureFactory().CreateTexture(
      video_metadata_.width, video_metadata_.height,
      filament::Texture::InternalFormat::RGBA8);
  return &(*texture_);
}

absl::StatusOr<BorrowedTexturePtr> LinuxVideoSource::BorrowVideoTextureImpl(
    SmallSourceLocation loc) {
  if (!texture_) {
    texture_ = view_->GetTextureFactory().CreateTexture(
        video_metadata_.width, video_metadata_.height,
        filament::Texture::InternalFormat::RGBA8);
  }
  return texture_.Borrow(loc);
}

void LinuxVideoSource::UpdateVideoTexture(filament::Texture* texture,
                                          absl::Duration frame_delta) {
  if (state_ != State::kPlaying || !output_fifo_ || !output_fifo_->IsOpen()) {
    return;
  }

  if (texture->getFormat() != filament::Texture::InternalFormat::RGBA8) {
    IMP_LOG(imp::FATAL) << "Incorrect texture format";
  }

  UpdateFrameTime(frame_delta);
  // Check if video frame should be updated to match the video's expected fps.
  if (!ShouldUpdateFrame()) return;

  uint32_t rgba_buffer_size =
      video_metadata_.width * video_metadata_.height * 4;
  int64_t byte_status =
      output_fifo_->ReadBytes(rgba_buffer_size, &rgba_buffer_);
  if (byte_status < 0) {
    IMP_LOG(imp::WARNING) << "Error reading bytes in video update loop";
    return;
  } else if (byte_status == 0) {
    OnPlaybackComplete();
    return;
  }
  filament::backend::PixelDataFormat format = filament::Texture::Format::RGBA;
  filament::backend::PixelDataType type = filament::Texture::Type::UBYTE;
  auto pixel_buffer = filament::Texture::PixelBufferDescriptor(
      rgba_buffer_.c_str(), rgba_buffer_size, format, type,
      [](void* buffer, size_t size, void* user) {}, &rgba_buffer_);
  texture->setImage(*engine_, /*level=*/0, std::move(pixel_buffer));
}

void LinuxVideoSource::OnPlaybackComplete() {
  if (loop_count_ > current_loop_ || loop_count_ < 0) {
    auto init_status = InitFfmpegAndOutputFifo();
    if (init_status.ok()) {
      ResetFrameTime();
      current_loop_ += 1;
      return;
    }
    IMP_LOG(imp::WARNING) << "Stopping playback. Failed to loop video with status: "
                 << init_status;
  }
  state_ = VideoSource::State::kStopped;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

absl::Status LinuxVideoSource::Play() {
  if (!output_fifo_ || !output_fifo_->IsOpen()) {
    return absl::InternalError("No video file opened");
  }
  state_ = State::kPlaying;
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::Pause() {
  state_ = State::kReady;
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::Stop() {
  state_ = State::kStopped;
  StopIO();
  OnPlaybackComplete();
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::SetPlaybackSpeed(float speed) {
  playback_speed_ = speed;
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::SeekTo(float seconds, SeekType seek_type) {
  State prev_state = state_;
  state_ = State::kReady;
  ffmpeg_params_.mutable_input(0)->set_start_pos_sec(seconds);
  if (output_fifo_->IsOpen()) {
    output_fifo_->Stop();
  }
  auto init_status = InitFfmpegAndOutputFifo();
  if (init_status.ok()) {
    current_frame_elapsed_time_ = absl::ZeroDuration();
    current_playback_time_ = absl::Seconds(seconds);
  }
  state_ = prev_state;
  return init_status;
}

absl::Status LinuxVideoSource::SetLoopCount(int loop) {
  current_loop_ = 0;
  loop_count_ = loop;
  return absl::OkStatus();
}

absl::Status LinuxVideoSource::SetVolume(float volume) {
  return absl::UnimplementedError(
      "Audio is not supported in Linux video implementation");
}

absl::StatusOr<absl::Duration> LinuxVideoSource::GetDuration() const {
  // TODO: Investigate other ways of extracting metadata that
  // include getting duration information. The current approach (yuv4mpeg header
  // parsing) does not provide this information.
  IMP_LOG(imp::WARNING) << "GetDuration is not implemented for Linux video playback";
  // TODO: Refactor ar_session_native_playback_test to use a no-op
  // video player and update GetDuration to return absl::UnimplementedError.
  return absl::ZeroDuration();
}

absl::StatusOr<absl::Duration> LinuxVideoSource::GetPlaybackTime() const {
  return current_playback_time_;
}

absl::StatusOr<int> LinuxVideoSource::GetLoopCount() const {
  return loop_count_;
}

media::MediaSource::State LinuxVideoSource::GetState() const { return state_; }

uint2 LinuxVideoSource::GetVideoSize() const {
  return {video_metadata_.width, video_metadata_.height};
}

MediaColorSpace LinuxVideoSource::GetColorSpace() const {
  return MediaColorSpace();
}

MediaStereoMode LinuxVideoSource::GetStereoMode() const {
  return static_cast<MediaStereoMode>(video_metadata_.stereo_mode);
}

void LinuxVideoSource::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = std::move(callback);
}

void LinuxVideoSource::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  // Not implemented.
}

void LinuxVideoSource::SetOnBufferingCallback(
    std::function<void(BufferingState)> callback) {
  // Not implemented.
}

}  // namespace

Future<std::unique_ptr<VideoSource>> CreateVideoSource(
    BaseView& base_view, absl::string_view asset_url) {
  auto video_source = std::make_unique<LinuxVideoSource>(&base_view);
  // Because the closure may move the video_source unique_ptr before Load()
  // gets called we should call Load() directly on the pointer itself rather.
  LinuxVideoSource* src = video_source.get();
  return src->Load(asset_url).Then(
      [media_source = std::move(video_source)](absl::Status status) mutable
      -> absl::StatusOr<std::unique_ptr<VideoSource>> {
        if (!status.ok()) {
          return status;
        } else {
          return std::move(media_source);
        }
      });
}

}  // namespace imp::video
