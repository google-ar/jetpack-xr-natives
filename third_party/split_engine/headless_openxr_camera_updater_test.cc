// Copyright 2026 Google LLC
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

#include "split_engine/headless_openxr_camera_updater.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "testing/view_fixture.h"
#include "testing/xr/openxr_mock.h"

namespace imp {
namespace split_engine {
namespace {

using ::imp::testing::OpenXrMock;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgPointee;

extern "C" {

XrResult __wrap_xrLocateViews(XrSession session,
                              const XrViewLocateInfo* locateInfo,
                              XrViewState* viewState,
                              uint32_t viewCapacityInput,
                              uint32_t* viewCountOutput, XrView* views) {
  if (imp::testing::OpenXrMock::current_mock())
    return imp::testing::OpenXrMock::current_mock()->xrLocateViews(
        session, locateInfo, viewState, viewCapacityInput, viewCountOutput,
        views);
  return XR_SUCCESS;
}
}

static XrResult MockInitializeLoader(const XrLoaderInitInfoBaseHeaderKHR*) {
  return XR_SUCCESS;
}

class HeadlessOpenXrCameraUpdaterTest : public imp::testing::ViewFixture {};

TEST_F(HeadlessOpenXrCameraUpdaterTest,
       InitializeUsesHeadlessModeSuccessfully) {
  OpenXrMock mock_openxr;

  EXPECT_CALL(mock_openxr, xrGetInstanceProcAddr(_, _, _))
      .WillRepeatedly([](XrInstance, const char* name,
                         PFN_xrVoidFunction* func) {
        if (strcmp(name, "xrInitializeLoaderKHR") == 0) {
          *func = reinterpret_cast<PFN_xrVoidFunction>(&MockInitializeLoader);
        } else {
          *func = nullptr;
          return XR_ERROR_FUNCTION_UNSUPPORTED;
        }
        return XR_SUCCESS;
      });

  EXPECT_CALL(mock_openxr, xrCreateInstance(_, _))
      .WillOnce(DoAll(SetArgPointee<1>(reinterpret_cast<XrInstance>(1)),
                      Return(XR_SUCCESS)));

  EXPECT_CALL(mock_openxr, xrGetSystem(_, _, _))
      .WillOnce(DoAll(SetArgPointee<2>(1), Return(XR_SUCCESS)));

  EXPECT_CALL(mock_openxr, xrCreateSession(_, _, _))
      .WillOnce(DoAll(SetArgPointee<2>(reinterpret_cast<XrSession>(1)),
                      Return(XR_SUCCESS)));

  EXPECT_CALL(mock_openxr, xrCreateReferenceSpace(_, _, _))
      .Times(2)
      .WillRepeatedly(DoAll(SetArgPointee<2>(reinterpret_cast<XrSpace>(1)),
                            Return(XR_SUCCESS)));

  EXPECT_CALL(mock_openxr, xrDestroySession(_)).WillOnce(Return(XR_SUCCESS));
  EXPECT_CALL(mock_openxr, xrDestroyInstance(_)).WillOnce(Return(XR_SUCCESS));

  absl::StatusOr<std::unique_ptr<HeadlessOpenXrCameraUpdater>> updater =
      HeadlessOpenXrCameraUpdater::Create(*view_);
  EXPECT_OK(updater.status());
}

}  // namespace
}  // namespace split_engine
}  // namespace imp
