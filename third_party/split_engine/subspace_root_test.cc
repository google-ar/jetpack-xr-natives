// Copyright 2025 Google LLC
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

#include "split_engine/subspace_root.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "testing/view_fixture.h"

namespace {
using ::imp::testing::ViewFixture;

class SubspaceRootTest : public ViewFixture {};

TEST_F(SubspaceRootTest, CreateSubspaceRootWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  EXPECT_TRUE(
      subspace_root->GetComponent<android_xr::SubspaceRoot::Tag>().IsValid());
  EXPECT_EQ(subspace_root_manager.GetNode(), subspace_root);
}

TEST_F(SubspaceRootTest,
       AnchorSubspaceRootToTaskSpaceWorksReturnsIdentityTransform) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kTaskSpace));
  EXPECT_EQ(subspace_root_manager.GetNode(), subspace_root);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetLocalTrs(), imp::mat4f{});
}

TEST_F(SubspaceRootTest, AnchorSubspaceRootToWorldSpaceWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  EXPECT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetLocalTrs(), imp::mat4f{});
}

TEST_F(SubspaceRootTest, AttachNodeToRootThenAnchorToWorldSpaceWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  EXPECT_EQ(subspace_root->GetChildren().size(), 1);
  EXPECT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
}

TEST_F(SubspaceRootTest, SetTransformWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kTaskSpace));
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  EXPECT_OK(subspace_root_manager.UpdateSubspaceTransform(
      imp::mat4f::translation(imp::float3{1, 2, 3})));
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f::translation(imp::float3{1, 2, 3}));
}

TEST_F(SubspaceRootTest, SetTransformThenAttachToWorldSpaceWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  MP_ASSERT_OK(subspace_root_manager.UpdateSubspaceTransform(
      imp::mat4f::translation(imp::float3{1, 2, 3})));
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f{});
}

TEST_F(SubspaceRootTest,
       SetTransformThenAttachToWorldSpaceAndBackToTaskSpaceWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  MP_ASSERT_OK(subspace_root_manager.UpdateSubspaceTransform(
      imp::mat4f::translation(imp::float3{1, 2, 3})));
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f{});
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kTaskSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f::translation(imp::float3{1, 2, 3}));
}

TEST_F(SubspaceRootTest,
       AttachToWorldSpaceAndBackToTaskSpaceWorksAndBackToWorldSpaceWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  MP_ASSERT_OK(subspace_root_manager.UpdateSubspaceTransform(
      imp::mat4f::translation(imp::float3{1, 2, 3})));
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f{});
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kTaskSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f::translation(imp::float3{1, 2, 3}));
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f{});
}

TEST_F(SubspaceRootTest, AttachToWorldSpaceThenSetTransformWorks) {
  imp::NodeHandle subspace_root = view_->CreateNode();
  android_xr::SubspaceRoot subspace_root_manager(*view_, subspace_root);
  subspace_root_manager.AttachToRoot(view_->CreateNode());
  MP_ASSERT_OK(subspace_root_manager.UpdateAnchor(
      android_xr::SubspaceRoot::AnchorType::kWorldSpace));
  MP_ASSERT_OK(subspace_root_manager.UpdateSubspaceTransform(
      imp::mat4f::translation(imp::float3{1, 2, 3})));
  ASSERT_EQ(subspace_root_manager.GetNode(), subspace_root->GetChildren()[0]);
  EXPECT_EQ(subspace_root_manager.GetNode()->GetChildren()[0]->GetWorldTrs(),
            imp::mat4f{});
}

}  // namespace
