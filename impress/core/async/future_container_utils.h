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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_CONTAINER_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_CONTAINER_UTILS_H_

#include <tuple>
#include <type_traits>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "core/async/future.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// Transforms a List<Future<T>> into a Future<List<T>> that collects
// the results from all the individual futures.
//
// This only works with `Future<T>`s, not `Future<absl::Status>`s.
//
// For example, you can use this to combine a series of individual requests into
// a single Future that awaits the results of all the requests:
//
// Future<std::vector<Item>> FetchItems(std::vector<std::string> item_ids) {
//   std::vector<Future<Item>> item_requests;
//   for (const auto& item_id : item_ids) {
//     item_requests.push_back(FetchItem(item_id));
//   }
//   return MergeFutures(item_requests);
// }
//
// If any of the individual Futures fails, the whole future will fail.
template <typename ListT, typename FutureT = typename ListT::value_type,
          typename ValueT = typename FutureT::Value>
ABSL_DEPRECATED("Use Future::MergeList instead.")
Future<std::vector<ValueT>> MergeFutures(const ListT& futures) {
  return FutureT::MergeList(futures);
}

// Transforms a Map<Key, Future<Value>> into a Future<Map<Key, Value>>
// that collects the results from all the individual futures.
//
// This only works with `Future<T>`s, not `Future<absl::Status>`s.
//
// For example, you can use this to combine a series of individual requests
// with IDs into a single Future:
//
// Future<tsl::robin_map<std::string, Item>>
// FetchItems(std::vector<std::string> item_ids) {
//   absl::flat_hash_map<std::string, Future<Item>> item_requests;
//   for (const auto& item_id : item_ids) {
//     item_requests.insert({item_id, FetchItem(item_id)});
//   }
//   return MergeFutures(item_requests);
// }
//
// If any of the individual Futures fails, the whole future will fail.
template <typename MapT, typename KeyT = typename MapT::key_type,
          typename FutureT = typename MapT::mapped_type,
          typename ValueT = typename FutureT::Value>
Future<tsl::robin_map<KeyT, ValueT>> MergeFutures(const MapT& keys_to_futures) {
  return MergeFutures<tsl::robin_map<KeyT, ValueT>>(keys_to_futures);
}

// Transforms a Map<Key, Future<Value>> into a Future<Map<Key, Value>>
// that collects the results from all the individual futures.
//
// This overload allows you to specify the returned map type using a type
// argument.
//
// This only works with `Future<T>`s, not `Future<absl::Status>`s.
//
// For example, you can use this to combine a series of individual requests
// with IDs into a single Future:
//
// Future<absl::flat_hash_map<std::string, Item>>
// FetchItems(std::vector<std::string> item_ids) {
//   absl::flat_hash_map<std::string, Future<Item>> item_requests;
//   for (const auto& item_id : item_ids) {
//     item_requests.insert({item_id, FetchItem(item_id)});
//   }
//   return MergeFutures<absl::flat_hash_map<std::string, Item>>(
//       item_requests);
// }
//
// If any of the individual Futures fails, the whole future will fail.
template <typename ReturnMapT, typename MapT,
          typename KeyT = typename MapT::key_type,
          typename FutureT = typename MapT::mapped_type,
          typename ValueT = typename FutureT::Value>
Future<ReturnMapT> MergeFutures(const MapT& keys_to_futures) {
  Future<absl::Status> combined_future(absl::OkStatus());

  for (const auto& [unused, future] : keys_to_futures) {
    combined_future = combined_future.Combine(future);
  }

  return combined_future.Then(
      [keys_to_futures = tsl::robin_map<KeyT, Future<ValueT>>(
           keys_to_futures.begin(), keys_to_futures.end())]() {
        ReturnMapT result;
        result.reserve(keys_to_futures.size());

        for (const auto& [key, future] : keys_to_futures) {
          if constexpr (std::is_copy_constructible_v<ValueT>) {
            result.insert({key, future.Get().value()});
          } else {
            result.insert({key, std::move(future.Move().value())});
          }
        }

        return result;
      });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_FUTURE_CONTAINER_UTILS_H_
