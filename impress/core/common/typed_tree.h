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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_TREE_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_TREE_H_

#include "absl/container/inlined_vector.h"
#include "absl/status/status.h"
#include "core/common/data_helpers.h"
#include "core/common/paired_span.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"

namespace imp {

// When building directed acyclic graphs (e.g. skeletons), id values pointing to
// descendants (children and next-siblings) always have an index greater than
// the referring id, thus cannot have an index value of 0, so we can use it as a
// sentinel value.
template <typename T, class V>
using TypedDescendantId = TypedIdWithSentinel<T, V, 0>;

// When building directed acyclic graphs (e.g. skeletons), id values pointing to
// parents always have an index less than the reffering id, thus cannot have an
// index value of kMaxValue<ValueType>, so we can use it as a sentinel value.
template <typename T, class V>
using TypedParentId = TypedIdWithSentinel<T, V, kMaxValue<V>>;

class VisitParentThenChildrenStack {
 public:
  void Push(size_t parent_index, size_t num_children_left);
  bool Empty() const;
  size_t ParentForNextIndex();
  void FlushEmptyFrames();

 private:
  static constexpr size_t kWatermark = 32;
  struct Item {
    size_t parent_index;
    size_t num_children_left;
  };
  absl::InlinedVector<Item, kWatermark> stack_;
};

template <typename I>
class TypedDagTools {
 public:
  using ReferredType = typename I::ReferredType;
  using ValueType = typename I::ValueType;
  template <typename T>
  using Lookup = PairedSpan<T, ReferredType>;
  using DescendantId = TypedDescendantId<ReferredType, ValueType>;
  using ParentId = TypedParentId<ReferredType, ValueType>;

  template <typename ChildCountType, typename Visitor>
  static void VisitParentThenChildren(Lookup<ChildCountType> child_counts,
                                      Visitor&& v) {
    using T = typename I::ReferredType;
    using ValueType = typename I::ValueType;
    using ParentId = TypedParentId<T, ValueType>;
    VisitParentThenChildrenStack stack;
    size_t size = child_counts.size();
    size_t cursor = 0;
    while (cursor < size) {
      const size_t root_self_index = cursor++;
      const I root_self = I::At(root_self_index);
      const size_t root_num_children = child_counts[root_self];

      v(root_self, ParentId{});
      if (root_num_children) {
        stack.Push(root_self_index, root_num_children);
        while (!stack.Empty()) {
          const size_t self_index = cursor++;
          const I self = I::At(self_index);
          const size_t num_children = child_counts[self];

          v(self, ParentId::At(stack.ParentForNextIndex()));
          if (num_children) {
            stack.Push(self_index, num_children);
          }

          // Flush empty frames of the stack.
          stack.FlushEmptyFrames();
        }
      }
    }
  }

  template <typename ChildCountType>
  static void ExpandGraph(Lookup<ChildCountType> child_counts,
                          Lookup<ParentId> parents,
                          Lookup<DescendantId> first_children,
                          Lookup<DescendantId> next_siblings) {
    size_t count = child_counts.size();
    assert(count == parents.size() && count == first_children.size() &&
           count == next_siblings.size());
    PairedVector<DescendantId, ReferredType> last_children;
    ParentId last_root;
    last_children.resize(count);

    VisitParentThenChildren<ChildCountType>(
        child_counts, [&parents, &first_children, &next_siblings,
                       &last_children, &last_root](I self, ParentId parent) {
          if (parent) {
            parents[self] = parent;
            if (last_children[parent]) {
              next_siblings[I{last_children[parent]}] = self;
            } else {
              first_children[parent] = self;
            }
            last_children[parent] = self;
          } else {
            if (last_root) {
              next_siblings[last_root] = self;
            }
            last_root = self;
          }
        });
  }

  template <typename ChildCounts>
  static absl::Status VerifyGraph(ChildCounts child_counts,
                                  Lookup<ParentId> parents,
                                  Lookup<DescendantId> first_children,
                                  Lookup<DescendantId> next_siblings) {
    // They must all have the same non-zero size.
    if (parents.empty() || parents.size() != first_children.size() ||
        first_children.size() != next_siblings.size() ||
        child_counts.size() != parents.size()) {
      return absl::InternalError("Sizes don't match");
    }

    // They must have less than kMaxValue<ValueType> elements.
    const size_t node_count = parents.size();
    if (node_count - 1 > kMaxValue<ValueType>) {
      return absl::InternalError("Too many items");
    }

    // Each item (besides the first root, item 0) should be referenced between
    // 'first_children' and 'next_siblings' exactly once.
    // While we're determining that, ensure that all indices are valid.
    PairedVector<uint32_t, ReferredType> refs;
    PairedVector<uint32_t, ReferredType> computed_child_counts;
    refs.resize(node_count);
    computed_child_counts.resize(node_count);
    for (ValueType i = 0; i < node_count; i++) {
      auto self = I(i);
      if (parents[self]) {
        auto parent = I(parents[self]);
        if (ValueType(parent) >= i) {
          return absl::InternalError("Invalid Parent");
        }
        computed_child_counts[parent]++;
      }
      if (first_children[self]) {
        auto first_child = I(first_children[self]);
        auto first_child_value = ValueType(first_child);
        if (first_child_value <= i || first_child_value >= node_count) {
          return absl::InternalError("Invalid first child");
        }
        refs[first_child]++;
      }
      if (next_siblings[self]) {
        auto next_sibling = I(next_siblings[self]);
        auto next_sibling_value = ValueType(next_sibling);
        if (next_sibling_value <= i || next_sibling_value >= node_count) {
          return absl::InternalError("Invalid next sibling");
        }
        refs[next_sibling]++;
      }
    }

    for (ValueType i = 0; i < node_count; i++) {
      auto self = I(i);
      auto expected_refs = i ? 1 : 0;
      if (refs[self] != expected_refs) {
        return absl::InternalError("refs don't match expected");
      }
      if (child_counts[self] != computed_child_counts[self]) {
        return absl::InternalError("child counts don't match computed");
      }
    }

    return absl::OkStatus();
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPED_TREE_H_
