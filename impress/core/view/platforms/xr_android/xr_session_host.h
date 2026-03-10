/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SESSION_HOST_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SESSION_HOST_H_

#include <jni.h>
#include <stdbool.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/invocable.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/monitor/default_monitor_summary.h"
#include "core/monitor/duration_measurement.h"
#include "core/monitor/monitor_summary.h"
#include "core/monitor/value_measurement.h"
#include "core/render/content_security_level.h"
#include "core/view/base_view.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_events.proto.imp.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/xr/openxr_events.h"
#include "java/com/google/ar/imp/view/xr/xr_setup_params.proto.imp.h"
#if IMP_MATERIAL_API(VULKAN) && IMP_PLATFORM(ANDROID)
#include "core/view/platforms/xr_android/xr_vulkan_platform.h"
#else
#include "core/view/platforms/xr_android/xr_opengl_platform.h"
#endif
#include "core/view/view_host.h"

namespace imp {

// Impress subclass of ViewHost specifically used for OpenXR integration.
//
// This class manages the initialization and lifecycle of OpenXR, performs
// rendering into each eye, and controls the frame loop when using OpenXR.
//
// TODO: Integrate tracking for OpenXR frame timing.
// TODO: Handle pausing correctly.
// TODO: Handle stopping correctly by ending the session.
class XrSessionHost : public ViewHost {
 public:
  enum class RunningState { kNotRunning, kRunning };
  // TODO: ShownState is only used internally and will be removed.
  enum class [[deprecated("Marked for removal, see (broken link)")]] ShownState {
    kShown,
    kHidden
  };

  // Metrics for performance monitoring.
  struct XrPerformanceState {
    std::unordered_map<std::string_view, MonitorSummary::CustomMetricHandle>
        metrics;
  };

  XrSessionHost(std::unique_ptr<imp::BaseView> view,
                com::google::ar::imp::view::xr::XrSetupParams xr_setup_params);
  ~XrSessionHost() override;

  // Sets up the host, which will create the filament Engine with an XrPlatform
  // instance, and ultimately lead to imp::View::Setup being called.
  absl::Status Setup(JNIEnv* env, JavaVM* vm, jobject context);

  // Performs the bulk of the work for initializing OpenXr by creating the
  // XrInstance & XrSession, then creating the XrSwapChain, and resizing the
  // Impress View.
  absl::Status onWindowAttached();

  // Advances a frame by blocking on xrWaitFrame and running the frame loop.
  // Rendering ultimately happens on Filament's rendering thread.
  absl::Status AdvanceFrame();

  // Returns the Xr display size. This is the width of both the left & right
  // eyes together, and is not the same as the size of the Impress View which is
  // just the width of one eye.
  //
  // This is the size of the underlying buffer that is actually rendered to.
  uint2 GetDisplaySize() const;

  // This is the size of the underlying buffer that is actually rendered to
  // when varjo foveated rendering is enabled.
  uint2 GetVarjoFoveationDisplaySize() const;

  // Returns the view sample count obtained by the XrViewConfigurationView
  // objects provided by OpenXR.
  uint32_t GetViewSampleCount() const;

  // Provides access to the underlying XrInstance object.
  XrInstance GetXrInstance() const;

  // Provides access to the underlying XrSession object.
  XrSession GetXrSession() const;

  // Provides access to the underlying XrSystemId.
  XrSystemId GetSystemId() const;

  // Provides access to the underlying XrSpace.
  XrSpace GetXrSpace() const;

  // Returns the predicated display time in XrTime units for the current frame.
  //
  // This should be the same as the time in imp::FrameTime, but this
  // is useful for accessing the time directly as the XrTime data type without
  // needing to do any conversion back and forth.
  XrTime GetPredictedDisplayTime() const;

  // Converts an XrResult into an absl::Status using the XrInstance.
  absl::Status ToStatus(XrResult) const;

  // Begins a frame. Can be called by either the Impress thread or the Filament
  // Render thread through XrSwapChain.
  absl::Status BeginFrame();

  // Ends a frame. Can be called by either the Impress thread or the Filament
  // Render thread through XrSwapChain.
  absl::Status EndFrame(XrSwapchain swapchain,
                        XrSwapchain depth_swapchain = XR_NULL_HANDLE);

  // Called right before frame work begins. Used in XrSessionHost to choose the
  // protected (or standard) swap chain after all scene work for this frame has
  // been completed, and we may or may not have protected content in view.
  absl::Status PreBeginRender();

  // Overrides normal rendering to render into both eyes.
  void PerformRender(filament::View* view,
                     ViewHost::RenderPassOptions options) override;

  // Renders the left and right eyes with a single Filament render call.
  void PerformEnhancedStereoscopicRender(filament::View* view,
                                         ViewHost::RenderPassOptions options);

  // Renders the left and right eyes with two separate Filament render calls.
  void PerformNaiveStereoscopicRender(filament::View* view);

  // Renders only the left eye.
  void PerformMonoRender(filament::View* view);

  // Returns the stereoscopic type of the session.
  filament::Engine::StereoscopicType GetStereoscopicType() const;

  bool IsInXr() const override;

  // Returns true if the session is multiview.
  bool IsMultiviewStereo() const;

  // Returns the number of logical eyes in the session. E.g., it returns 4 if
  // foveated rendering is enabled.
  uint32_t GetLogicalEyeCount() const;

  // Identifies thread type to system through xrSetAndroidApplicationThreadKHR.
  absl::Status SetThreadType(XrAndroidThreadTypeKHR threadType);

  // kShown allows layer data to be displayed and kHidden blocks the submission
  // of layer data in xrEndFrame.
  absl::Status SetShownState(ShownState state);

  // Returns a list of color spaces supported by the system.
  absl::StatusOr<std::vector<XrColorSpaceFB>> EnumerateColorSpaces();

  // Sets the color space for the session.
  absl::Status SetColorSpace(XrColorSpaceFB color_space);

  // Returns the fence fd representing a GPU fence.
  uint32_t GetFenceFd() const;

#if IMP_RUNTIME(DEV)
  std::unique_ptr<imp::editor::EditorPlugin> CreateEditorPlugin() override;
#endif

  // If true, depth images will be submitted and composited along with the
  // projection images of the swap chain as per
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_FB_composition_layer_depth_test
  // and
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_KHR_composition_layer_depth
  bool IsCompositionLayerDepthEnabled();

  // TODO: Safely delete/create the depth swapchain when depth
  // composition is disabled/enabled. This requires some coordination & timing.
  // For example, we likely want to wait for the frame to end, at which point we
  // can call glFlush before we call xrDestroySwapChain.
  void SetCompositionLayerDepthEnabled(bool composition_layer_depth_enabled);

  ContentSecurityLevel GetContentSecurityLevel();

  void SetContentSecurityLevel(ContentSecurityLevel security);

  bool IsXrFbFoveationEnabled() const;
  bool IsXrVarjoQuadViewsEnabled() const;
  bool IsXrVarjoFoveatedRenderingEnabled() const;
  bool IsXrFbColorSpaceEnabled() const;
  // Returns true if this frame should be rendered with varjo foveation. Should
  // only be called on filament's render thread.
  // NOTE: This is a performance heavy call. Should only be called once per
  // frame.
  bool ShouldRenderVarjoFoveationThisFrame();
  bool IsXrAndroidXOccupancyGridEnabled() const;
  bool IsXrAndroidXSpatialInteractionEnabled() const;
  bool IsXrAndroidDepthTextureEnabled() const;
  bool IsXrEyeGazeInteractionEnabled() const;
  bool IsXrAndroidSystemExtensionsEnabled() const;
  bool IsXrGlobalPassthroughDimmingExtensionsEnabled() const;
  bool IsXrEyeTrackingCalibrationEnabled() const;

  void SetFoveationLevel(XrFoveationLevelFB xr_foveation_level_fb);

  void SetEyeTrackingEnabled(bool enabled);

  XrFoveationLevelFB GetCurrentFoveationLevel();

  void SetEnvironmentBlendMode(
      XrEnvironmentBlendMode xr_environment_blend_mode);

  // Returns the number of samples per pixel for the color and depth textures.
  int32_t GetMsaaSampleCount() const;

  absl::Status SetDisplayState(XrHelpers::DisplayState new_state);

  filament::Engine::Config GetEngineConfig() override;

  MonitorSummary& GetXrTimingSummary() {
    return xr_timing_summary_.GetSummary();
  }

  XrPerformanceState& GetXrPerformanceState() { return xr_performance_state_; }

  // Sets a callback to be called after Impress is done generating GPU work,
  // right before xrEndFrame is called for the current frame.
  //
  // IMPORTANT: This only applies to the current frame. To receive a callback
  // every frame, this must be called every frame.
  //
  // WARNING: The callback is called from the filament rendering thread.
  void SetBeforeEndFrameCallback(Invocable<void()> callback);

  // Sets a callback to be called after xrEndFrame is called.
  //
  // IMPORTANT: This only applies to the current frame. To receive a callback
  // every frame, this must be called every frame.
  //
  // WARNING: The callback is called from the filament rendering thread.
  //
  // When the callback is invoked, the fence_fd argument holds a file
  // descriptor that represents a fence sync object signaling completion of
  // GPU work generated by xrEndFrame, or -1 if the fence sync object could
  // not be created. If fence_fd is not -1, the receiver of the callback
  // takes ownership of the file descriptor and is responsible for closing it.
  void SetAfterEndFrameCallback(Invocable<void(int /*fence_fd*/)> callback);

  // This notes that the CPU-side calls for rendering a frame have been
  // completed, and informs xr_session_host that it is safe to create a Sync
  // object that will signal when all rendering operations for the frame have
  // been completed by the GPU.
  //
  // This sync will be used by SetAfterEndFrameCallback, if used. If not, this
  // function does not need to be called.
  //
  // IMPORTANT: This applies to the current frame. This should be called at
  // the end of every frame if using SetAfterEndFrameCallback, if a fence is
  // desired in that call.
  //
  // IMPORTANT: It is expected that this is called on the Filament frontend
  // thread, not the backend thread.
  void MarkPostRenderAndCreateSync();

#if IMP_MATERIAL_API(VULKAN) && IMP_PLATFORM(ANDROID)
  using XrGraphicsBinding = XrGraphicsBindingVulkan2KHR;
  using XrGraphicsRequirements = XrGraphicsRequirementsVulkan2KHR;
  using XrGetGraphicsRequirements = PFN_xrGetVulkanGraphicsRequirements2KHR;
#else
  using XrGraphicsBinding = XrGraphicsBindingOpenGLESAndroidKHR;
  using XrGraphicsRequirements = XrGraphicsRequirementsOpenGLESKHR;
  using XrGetGraphicsRequirements = PFN_xrGetOpenGLESGraphicsRequirementsKHR;
#endif
  enum class LocateViewsStatus { kUnableToObtainPose, kObtainedPose };
  struct LocateViewsResult {
    std::vector<XrView> views;
    LocateViewsStatus status;
  };
  absl::StatusOr<LocateViewsResult> LocateViews(XrTime predictedDisplayTime);
  struct ViewInfo {
    std::vector<XrView> views;
    XrViewConfigurationType view_configuration_type;
  };
  absl::StatusOr<ViewInfo> GetLatestViews();

  // Gets a reference to the list of extensions to load.
  // Returns a reference so that an app can set the list of extensions to
  // load before the session host is created. Static so it can be called
  // before the session host is created.
  static absl::Span<const char* const>& GetExtensionsToLoad();

  // Gets a reference to a list of optional extensions to load, provided by an
  // app. Returns a reference so that an app can set the list of extensions to
  // load before the session host is created. Static so it can be called
  // before the session host is created.
  static absl::Span<const char* const>& GetOptionalExtensionsToLoad();

  std::optional<XrSystemProperties> GetSystemProperties() const;

  // Get all enabled extensions.
  RobinSet<std::string> GetEnabledExtensions() const;

  // Add OpenXr layer that should be submitted to XrEndFrame, plus a weight
  // Negative weights are drawn in front of the impress-rendered projection
  // layer, and positive weights are drawn behind.  A weight of 0 is
  // indeterminate compared to the projection layer.
  void AddCompositionLayer(XrCompositionLayerBaseHeader* layer, int weight);

  void RemoveCompositionLayer(XrCompositionLayerBaseHeader* layer);

 private:
  enum class LocateSpaceStatus { kUnableToObtainPose, kObtainedPose };

#if IMP_MATERIAL_API(VULKAN) && IMP_PLATFORM(ANDROID)
  using PlatformType = imp::XrVulkanPlatform;
#else
  using PlatformType = imp::XrOpenGLPlatform;
#endif

  struct LocateSpaceResult {
    XrSpaceLocation location;
    LocateSpaceStatus status;
  };

  struct FenceCallbackData {
    XrTime display_time;
    PlatformType* platform;
    filament::Engine* engine;
    filament::Sync* sync;
    std::unique_ptr<Invocable<void(int /*fence_file_descriptor*/)>>
        after_end_frame_callback;
  };

  // Used to enqueue frame information to the Filament Render thread to ensure
  // that the view poses & display times are rendered in the correct order.
  struct QueuedFrameInfo {
    XrTime display_time;
    std::vector<XrView> views;
    bool should_render_varjo_foveation;
    Invocable<void()> before_end_frame_callback;
    Invocable<void(int)> after_end_frame_callback;
    filament::Sync* sync;
  };

  absl::StatusOr<XrInstance> CreateInstance(JNIEnv* env, JavaVM* vm,
                                            jobject context);

  absl::StatusOr<XrSystemId> ObtainSystemId() const;

  absl::Status CheckGraphicsRequirements() const;

  absl::StatusOr<XrSession> CreateSession() const;

  absl::StatusOr<XrSpace> ObtainXrSpace(XrReferenceSpaceType space_type) const;

  absl::StatusOr<std::vector<XrViewConfigurationView>> ObtainViewConfigs()
      const;

  absl::StatusOr<std::vector<XrViewConfigurationView>>
  ObtainFoveatedViewConfigs() const;

  absl::Status PollEvents();

  void BroadcastReferenceSpaceChanges(XrTime predicted_display_time);

  std::optional<const XrEventDataBaseHeader*> GetNextEvent();

  absl::Status HandleSessionStateChanged(
      const XrEventDataSessionStateChanged& event);

  absl::Status BeginSession();

  absl::Status EndSession();

  absl::StatusOr<XrFrameState> WaitFrame();

  // Locates the view space in the reference base space at a given display time.
  // In the stereo set up, this view space is the centroid of two views' origin.
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#reference-spaces.
  absl::StatusOr<LocateSpaceResult> LocateViewInReference(
      XrTime predicted_display_time);

  absl::Status BeginAndDiscardFrame(XrTime predictedDisplayTime);

  // Sets the Camera Node transform to that of an XrView.
  void UpdateCameraFromXrView(const XrView& view);

  // Sets the Camera Node transform to that of an XrPose.
  void UpdateCameraFromXrPose(const XrPosef& pose);

  // Returns the view configs to use for the current frame on the Impress
  // thread.
  const std::vector<XrViewConfigurationView>* GetActiveViewConfigs() const;
  uint32_t GetViewWidth(const XrViewConfigurationView& view_config);
  uint32_t GetViewHeight(const XrViewConfigurationView& view_config);

  // Get the sample count and the display size from the view configs.
  uint2 CalculateDisplaySize(
      const std::vector<XrViewConfigurationView>& view_configs);

  void ResizeImpressView();

  // Verifies that the given list of extensions are supported by OpenXR.
  absl::Status EnsureSupportedExtensions(
      const std::vector<const char*>& required_extensions) const;

  // Takes a list of extensions and filters out any that are not supported.
  static std::vector<const char*> FilterUnsupportedExtensions(
      const absl::Span<const char* const>& extensions);

  // Tracks if the XrSession is currently running.
  // The session is considered to be running after a successful call to
  // xrBeginSession and before calling xrEndSession.
  RunningState state_ = RunningState::kNotRunning;

  // Tracks the current state of the XrSession.
  // This is different from the running state, which is not indicated by the
  // XrSessionState.
  XrSessionState session_state_ = XR_SESSION_STATE_UNKNOWN;

  // Set to true at the end of AdvanceFrame, but to false if the frame is
  // discarded.
  bool did_last_advance_frame_succeed_ = false;

  // If set to kHidden no frame data will be submitted when xrEndFrame is
  // called.
  ShownState shown_state_ = ShownState::kShown;

  std::unique_ptr<PlatformType> platform_;
  XrInstance instance_ = XR_NULL_HANDLE;
  XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
  XrSession session_ = XR_NULL_HANDLE;
  XrReferenceSpaceType reference_space_type_;
  XrSpace reference_space_ = XR_NULL_HANDLE;
  XrSpace view_space_ = XR_NULL_HANDLE;
  XrSpace render_gaze_space_ = XR_NULL_HANDLE;
  bool is_composition_layer_depth_enabled_ = false;
  bool is_enhanced_stereoscopic_rendering_enabled_ = false;
  bool is_enhanced_stereoscopic_rendering_initialized_ = false;
  bool use_max_swapchain_size_ = false;
  float swapchain_size_multiplier_ = 1.0f;
  XrFoveationLevelFB current_foveation_level_ = XR_FOVEATION_LEVEL_NONE_FB;
  XrEnvironmentBlendMode environment_blend_mode_ =
      XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  std::vector<XrCompositionLayerDepthInfoKHR> layer_depth_infos_;

  std::vector<XrViewConfigurationView> view_configs_;
  std::vector<XrViewConfigurationView> varjo_foveation_view_configs_;
  uint32_t view_sample_count_ = 0;
  uint2 display_size_;
  uint2 varjo_foveation_display_size_;
  std::vector<XrView> latest_views_;
  // The standard filament::SwapChain to use for rendering.
  const filament::SwapChain* swap_chain_standard_ = nullptr;
  // A protected filament::SwapChain to use for rendering DRM content.
  const filament::SwapChain* swap_chain_protected_ = nullptr;
  // Controls which swap chain to use for rendering the current frame.
  ContentSecurityLevel content_security_level_ = ContentSecurityLevel::kNone;

  XrEventDataBuffer event_data_buffer_;

  absl::Duration last_frame_time_ = absl::ZeroDuration();

  // Used to enqueue frame information to the Filament Render thread to ensure
  // that the view poses & display times are rendered in the correct order.
  absl::Mutex frame_queue_mutex_;
  std::queue<QueuedFrameInfo> frame_queue_ ABSL_GUARDED_BY(frame_queue_mutex_);

  // Stores the predicated display time in XrTime units for the current frame.
  //
  // This should be the same as the time in imp::FrameTime, but this
  // is useful for accessing the time directly as the XrTime data type without
  // needing to do any conversion back and forth.
  // TODO: Consider changing/removing as part of designing system
  // for Xr input.
  XrTime latest_predicted_display_time_ = 0;

  std::optional<XrViewStateChangedEvent::ViewState> previous_view_state_;
  // Measures the time between successful calls to AdvanceFrame
  std::optional<DurationMeasurement> xr_between_frame_timing_;
  // Scheduled frame rate provided by OpenXR
  std::optional<DurationMeasurement> xr_scheduled_timing_;
  // Time from BeginFrame to EndFrame inclusive on the filament thread.
  std::optional<DurationMeasurement> filament_thread_timing_;
  // Reports the current display state.
  std::unique_ptr<ValueMeasurement> display_state_statistics_;
  // Timestamp used in filament thread.
  absl::Time filament_begin_timestamp_ = absl::InfiniteFuture();
  // Most recent timing sample accessed in filament thread only
  std::optional<absl::Duration> filament_thread_duration_;
  // Timing sample from one frame ago.  Accessed in both threads and protected
  // by mutex
  std::optional<absl::Duration> prev_filament_thread_duration_
      ABSL_GUARDED_BY(frame_queue_mutex_);

  // Enables or blocks rendering without affecting simulation.
  XrHelpers::DisplayState display_state_ =
      XrHelpers::DisplayState::kDisplayEnabled;

  XrViewConfigurationType view_configuration_type_;
  bool is_varjo_foveated_rendering_enabled_ = false;
  int msaa_sample_count_ = 0;
  bool eye_tracking_enabled_ = false;
  bool ipd_eye_calibration_enabled_ = false;
  bool eye_tracking_calibration_enabled_ = false;
  // Whether the current frame should render with varjo foveation. Should only
  // be used on Impress thread.
  bool use_varjo_foveation_this_frame_ = false;
  bool should_resize_view_ = false;
  bool is_android_depth_texture_enabled_ = false;
  DurationMeasurement display_enabled_duration_;
  DefaultMonitorSummary xr_timing_summary_;
  XrPerformanceState xr_performance_state_;

  // Whether the color space extension is supported by the system.
  bool is_fb_color_space_enabled_ = false;

  // Whether the Android system extensions are enabled.
  bool is_android_system_extensions_enabled_ = false;

  // Whether the global passthrough dimming extensions are enabled.
  bool is_global_passthrough_dimming_extensions_enabled_ = false;

  // All enabled extensions
  RobinSet<std::string> enabled_extensions_;

  std::vector<OpenXrSpaceChangePendingEvent> pending_space_changes_;

  // OpenXr layers that should be submitted to XrEndFrame, plus their weights
  // Negative weights are drawn in front of the impress-rendered projection
  // layer, and positive weights are drawn behind.  A weight of 0 is
  // indeterminate compared to the projection layer.
  RobinMap<XrCompositionLayerBaseHeader*, int> composition_layers_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_SESSION_HOST_H_
