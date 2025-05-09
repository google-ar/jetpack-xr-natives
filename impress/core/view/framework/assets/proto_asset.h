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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_PROTO_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_PROTO_ASSET_H_

#include <memory>
#include <optional>
#include <type_traits>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/proto/proto_reader.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

// Asset that wraps any protocol buffer using the Impress AssetManager.
//
// ProtoAsset supports both impress protos ((broken link))
// and standard protos.
template <typename T>
class ProtoAsset {
 public:
  static Future<std::unique_ptr<ProtoAsset<T>>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future, bool retain_resource);

  explicit ProtoAsset(T proto, std::optional<resources::Resource> resource);

  // Provides access  to the proto stored by the asset.
  const T& GetProto() const;

 private:
  T proto_;
  std::optional<resources::Resource> resource_;
};

template <typename T>
Future<std::unique_ptr<ProtoAsset<T>>> ProtoAsset<T>::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future, bool retain_resource) {
  return resource_future.Then(
      [retain_resource](resources::Resource resource)
          -> absl::StatusOr<std::unique_ptr<ProtoAsset<T>>> {
        T proto;
        if constexpr (proto_traits::kIsStandardProto<T>) {
          if (!proto.ParseFromString(resource.GetData().StringView())) {
            return absl::InvalidArgumentError("Unable to parse proto.");
          }
        } else {
          if (!proto::ParseMessage(resource.GetData().StringView(), &proto)) {
            return absl::InvalidArgumentError("Unable to parse proto.");
          }
        }
        return std::make_unique<ProtoAsset<T>>(
            std::move(proto),
            retain_resource
                ? std::optional<resources::Resource>(std::move(resource))
                : std::nullopt);
      },
      Executor::Type::kBackground);
}

template <typename T>
ProtoAsset<T>::ProtoAsset(T proto, std::optional<resources::Resource> resource)
    : proto_(std::move(proto)), resource_(std::move(resource)) {}

template <typename T>
const T& ProtoAsset<T>::GetProto() const {
  return proto_;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_PROTO_ASSET_H_
