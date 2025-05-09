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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_DIFFER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_DIFFER_H_

#include <stdbool.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/common/template_helpers.h"
#include "core/math/almost_equal.h"
#include "core/proto/proto_common.h"

namespace imp {

namespace proto {

// Proto visitor used to compare the fields of two proto messages of the same
// type, and perform operations based on that comparison.
//
// This essentially does the inverse operation of what happens when two protos
// are merged together, treating msg and other as the two protos that are being
// merged. That means that when repeated fields are being compared, the merge
// strategy is used to determine how to compare the repeated fields.
class ProtoDiffer {
 public:
  enum class Result {
    // Indicates that all of the visited fields match.
    kFoundAllFieldsMatch,
    // Indicates that at least one of the visited fields didn't match.
    kFoundDifferences,
    // Indicates that a visited field didn't match in a way that can't be
    // correctly merged back together when parsing the two protos as a single
    // proto.
    //
    // For example, if a field is a vector and msg contains fewer elements than
    // other, the vector cannot be merged back together because merging can't
    // remove elements from the vector.
    //
    // When using kRemoveMatchingFields, if unmergable differences are found the
    // matching fields aren't all removed.
    kFoundUnmergableDifferences
  };

  enum class Mode {
    // Indicates that when a matching field is encountered by the visitor it
    // will be reset or removed.
    kRemoveMatchingFields,
    // Indicates that the visitor will detect differences in fields, but won't
    // mutate the fields in any way.
    kCheckDifferences
  };

  struct Cursor {
    Result result = Result::kFoundAllFieldsMatch;
    Mode mode = Mode::kRemoveMatchingFields;
  };

  template <int field_type, typename T>
  Cursor* Visit(Cursor* cursor, int field_id, T* field, T* other);

  template <int field_type, typename T>
  Cursor* Visit(Cursor* cursor, int field_id, absl::optional<T>* field,
                absl::optional<T>* other);

  template <int field_type, typename T>
  Cursor* Visit(Cursor* cursor, int field_id, std::unique_ptr<T>* field,
                std::unique_ptr<T>* other);

  template <int field_type, RepeatedMergeStrategy merge_type, typename T>
  Cursor* Visit(Cursor* cursor, int field_id, std::vector<T>* field,
                std::vector<T>* other);

  template <int key_type, int value_type, typename K, typename V>
  Cursor* Visit(Cursor* cursor, int field_id, std::map<K, V>* field,
                std::map<K, V>* other);

  template <typename T, typename FieldType, FieldType... field_types>
  Cursor* VisitVariant(
      Cursor* cursor, T* field, T* other, absl::string_view field_name,
      std::integer_sequence<int, field_types...> variant_field_types,
      const std::vector<int>& variant_field_ids);

  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, bool* field, bool* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, int32_t* field, int32_t* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, uint32_t* field, uint32_t* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, int64_t* field, int64_t* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, uint64_t* field, uint64_t* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, float* field, float* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, double* field, double* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, std::string* field,
                std::string* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, absl::string_view* field,
                absl::string_view* other);
  template <int field_type>
  Cursor* Visit(Cursor* cursor, int field_id, absl::Cord* field,
                absl::Cord* other);

  template <typename Proto>
  Cursor* VisitStandardProto(Cursor* cursor, int field_id, Proto* field,
                             Proto* other);

  Cursor* Unknown(Cursor* cursor);

 private:
  template <int field_type, typename T>
  Cursor* VisitPrimitive(Cursor* cursor, int field_id, T* field, T* other);
};

template <int field_type, typename T>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id, T* field,
                                        T* other) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    static_assert(field_type == TYPE_ENUM);

    if (*field == *other) {
      if (cursor->mode == Mode::kRemoveMatchingFields) {
        *field = T();
      }
    } else {
      cursor->result = Result::kFoundDifferences;
    }
  } else {
    static_assert(field_type == TYPE_MESSAGE);

    cursor = ::imp::proto::VisitPaired(field, this, cursor, other);
  }

  return cursor;
}

template <int field_type, typename T>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        absl::optional<T>* field,
                                        absl::optional<T>* other) {
  // We can only diff them if one of them contains a value.
  if (!field->has_value() || !other->has_value()) {
    if (field->has_value() || other->has_value()) {
      // Only one is empty, so they are different.
      cursor->result = Result::kFoundDifferences;
      if (cursor->mode == ProtoDiffer::Mode::kCheckDifferences) {
        return cursor;
      }
    }
    return cursor;
  }

  Cursor sub_cursor{.mode = cursor->mode};
  Visit<field_type>(&sub_cursor, field_id, &field->value(), &other->value());

  if (sub_cursor.result == Result::kFoundAllFieldsMatch) {
    if (cursor->mode == Mode::kRemoveMatchingFields) {
      field->reset();
    }
  } else {
    cursor->result = Result::kFoundDifferences;
  }

  return cursor;
}

template <int field_type, typename T>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        std::unique_ptr<T>* field,
                                        std::unique_ptr<T>* other) {
  // We can only diff them if one of them contains a value.
  if (!field->get() || !other->get()) {
    if (field->get() || other->get()) {
      // Only one is empty, so they are different.
      cursor->result = Result::kFoundDifferences;
      if (cursor->mode == ProtoDiffer::Mode::kCheckDifferences) {
        return cursor;
      }
    }
    return cursor;
  }

  Cursor sub_cursor{.mode = cursor->mode};
  T* field_value = field->get();
  T* other_value = other->get();
  Visit<field_type>(&sub_cursor, field_id, field_value, other_value);

  if (sub_cursor.result == Result::kFoundAllFieldsMatch) {
    if (cursor->mode == Mode::kRemoveMatchingFields) {
      field->reset();
    }
  } else {
    cursor->result = Result::kFoundDifferences;
  }

  return cursor;
}

// TODO: Add support for RemoveMatchingFields to work correctly in
// the case where there are unmergeable differences.
template <int field_type, RepeatedMergeStrategy merge_type, typename T>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        std::vector<T>* field,
                                        std::vector<T>* other) {
  if constexpr (merge_type == RepeatedMergeStrategy::kOverwrite) {
    // If the vectors are different sizes then they don't match.
    if (field->size() != other->size()) {
      cursor->result = Result::kFoundDifferences;
      return cursor;
    }

    // If *any* element is different than the entire vector is
    // treated as different and we keep the whole thing.
    for (int i = 0; i < field->size(); ++i) {
      Cursor sub_cursor{.mode = Mode::kCheckDifferences};
      Visit<field_type>(&sub_cursor, 0, &field->at(i), &other->at(i));
      if (sub_cursor.result != Result::kFoundAllFieldsMatch) {
        // Even if the sub_cursor found unmergeable differences, still use the
        // kFoundDifferences result here because the entire vector is being
        // replaced so elements within the vector won't actually be merged.
        cursor->result = Result::kFoundDifferences;
        return cursor;
      }
    }

    // The vectors match exactly.
    // If they didn't, we would have returned earlier.
    if (cursor->mode == Mode::kRemoveMatchingFields) {
      field->clear();
    }

    return cursor;
  } else if constexpr (merge_type == RepeatedMergeStrategy::kPerElement) {
    // If the vector is shorter than the other, then the difference is
    // unmergeable.
    if (field->size() < other->size()) {
      cursor->result = Result::kFoundUnmergableDifferences;
      return cursor;
    }

    // Diff each individual element.
    //
    // Start from the back so we can erase perfectly matching elements until we
    // find the first difference.
    for (int i = field->size() - 1; i >= 0; --i) {
      if (i >= other->size()) {
        cursor->result = Result::kFoundDifferences;
        continue;
      }

      Cursor sub_cursor{.mode = cursor->mode};
      Visit<field_type>(&sub_cursor, 0, &field->at(i), &other->at(i));
      if (sub_cursor.result != Result::kFoundAllFieldsMatch) {
        cursor->result = sub_cursor.result;
      } else if (sub_cursor.result == Result::kFoundAllFieldsMatch &&
                 i == field->size() - 1) {
        if (cursor->mode == Mode::kRemoveMatchingFields) {
          field->erase(field->begin() + i);
        }
      }
    }

    return cursor;
  } else if constexpr (merge_type == RepeatedMergeStrategy::kConcat) {
    // If the vector is shorter than the other, then the difference is
    // unmergeable.
    if (field->size() < other->size()) {
      cursor->result = Result::kFoundUnmergableDifferences;
      return cursor;
    }

    // If any element within the base vector is different than the corresponding
    // element in the other vector, then the difference is unmergeable.
    for (int i = 0; i < other->size(); ++i) {
      Cursor sub_cursor{.mode = Mode::kCheckDifferences};
      Visit<field_type>(&sub_cursor, 0, &field->at(i), &other->at(i));
      if (sub_cursor.result != Result::kFoundAllFieldsMatch) {
        cursor->result = Result::kFoundUnmergableDifferences;
        return cursor;
      }
    }

    // Remove all elements from base vector that are in the field.
    if (!other->empty()) {
      if (field->size() > other->size()) {
        cursor->result = Result::kFoundDifferences;
      }
      field->erase(field->begin(), field->begin() + other->size());
    }
    return cursor;
  }
}

template <int key_type, int value_type, typename K, typename V>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        std::map<K, V>* field,
                                        std::map<K, V>* other) {
  // If the maps don't match in size, then we know there are differences.
  // This catches the case where field contains everything that other does, but
  // other also contains additional keys that aren't in field.
  //
  // This must be checked prior to removing fields from the map.
  if (field->size() != other->size()) {
    cursor->result = Result::kFoundDifferences;
    if (cursor->mode == ProtoDiffer::Mode::kCheckDifferences) {
      return cursor;
    }
  }

  auto itr = field->begin();
  while (itr != field->end()) {
    auto other_itr = other->find(itr->first);
    if (other_itr != other->end()) {
      V* value = &itr->second;
      V* other_value = &other_itr->second;
      Cursor sub_cursor{.mode = cursor->mode};
      Visit<value_type>(&sub_cursor, 0, value, other_value);
      if (sub_cursor.result == Result::kFoundAllFieldsMatch) {
        if (cursor->mode == Mode::kRemoveMatchingFields) {
          itr = field->erase(itr);
          continue;
        }
      } else {
        cursor->result = Result::kFoundDifferences;
        if (cursor->mode == ProtoDiffer::Mode::kCheckDifferences) {
          return cursor;
        }
      }
    } else {
      cursor->result = Result::kFoundDifferences;
      if (cursor->mode == ProtoDiffer::Mode::kCheckDifferences) {
        return cursor;
      }
    }
    itr++;
  }

  return cursor;
}

template <typename T, typename FieldType, FieldType... field_types>
ProtoDiffer::Cursor* ProtoDiffer::VisitVariant(
    Cursor* cursor, T* field, T* other, absl::string_view field_name,
    std::integer_sequence<int, field_types...> variant_field_types,
    const std::vector<int>& variant_field_ids) {
  ForConstexpr<0, std::variant_size_v<T>>([this, &cursor, &variant_field_ids,
                                           &variant_field_types, field,
                                           other](auto i) mutable {
    // Only look for differences for the current alternative of the field.
    if (field->index() != i) {
      return;
    }

    if (field->index() != other->index()) {
      // The variants don't hold the same type, that means a difference was
      // found.
      cursor->result = Result::kFoundDifferences;
      return;
    }

    if constexpr (i != 0) {
      using VariantAlternativeT = std::variant_alternative_t<i, T>;

      constexpr int field_type = GetAt<i - 1>(variant_field_types);
      int variant_field_id = variant_field_ids[i - 1];

      Cursor sub_cursor{.mode = cursor->mode};
      if constexpr (field_type == TYPE_MESSAGE &&
                    proto_traits::kIsStandardProto<VariantAlternativeT>) {
        VisitStandardProto(&sub_cursor, variant_field_id,
                           absl::get_if<i>(field), absl::get_if<i>(other));
      } else {
        Visit<field_type>(&sub_cursor, variant_field_id, absl::get_if<i>(field),
                          absl::get_if<i>(other));
      }
      if (sub_cursor.result == Result::kFoundAllFieldsMatch) {
        if (cursor->mode == Mode::kRemoveMatchingFields) {
          *field = absl::monostate();
        }
      } else {
        cursor->result = Result::kFoundDifferences;
      }
    }
  });

  return cursor;
}

template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        bool* field, bool* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        int32_t* field, int32_t* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        uint32_t* field, uint32_t* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        int64_t* field, int64_t* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        uint64_t* field, uint64_t* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        float* field, float* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        double* field, double* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        std::string* field,
                                        std::string* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        absl::string_view* field,
                                        absl::string_view* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}
template <int field_type>
ProtoDiffer::Cursor* ProtoDiffer::Visit(Cursor* cursor, int field_id,
                                        absl::Cord* field, absl::Cord* other) {
  return VisitPrimitive<field_type>(cursor, field_id, field, other);
}

template <typename Proto>
ProtoDiffer::Cursor* ProtoDiffer::VisitStandardProto(Cursor* cursor,
                                                     int field_id, Proto* field,
                                                     Proto* other) {
  size_t num_bytes = field->ByteSizeLong();
  std::string bytes;
  bytes.resize(num_bytes);
  field->SerializeToArray(bytes.data(), num_bytes);

  size_t num_bytes_other = other->ByteSizeLong();
  std::string bytes_other;
  bytes_other.resize(num_bytes_other);
  other->SerializeToArray(bytes_other.data(), num_bytes_other);

  if (bytes == bytes_other) {
    if (cursor->mode == Mode::kRemoveMatchingFields) {
      field->Clear();
    }
  } else {
    cursor->result = Result::kFoundDifferences;
  }

  return cursor;
}

template <int field_type, typename T>
ProtoDiffer::Cursor* ProtoDiffer::VisitPrimitive(Cursor* cursor, int field_id,
                                                 T* field, T* other) {
  bool found_match = false;

  // Special handling for floats and doubles to handle precision.
  if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
    found_match = AlmostEqual(*field, *other);
  } else {
    found_match = *field == *other;
  }

  if (found_match) {
    if (cursor->mode == Mode::kRemoveMatchingFields) {
      *field = T();
    }
  } else {
    cursor->result = Result::kFoundDifferences;
  }

  return cursor;
}

// Recurses through all the fields of msg and other and removes/resets all
// fields that match exactly.
//
// Returns if any differences were found.
template <typename T>
ProtoDiffer::Result RemoveMatchingFields(T& msg, T& other) {
  ProtoDiffer differ;
  ProtoDiffer::Cursor cursor;
  ::imp::proto::VisitPaired(&msg, &differ, &cursor, &other);
  return cursor.result;
}

// Recurses through all the fields of msg and other and returns if any
// differences were found.
template <typename T>
ProtoDiffer::Result CheckIfFieldsMatch(T& msg, T& other) {
  ProtoDiffer differ;
  ProtoDiffer::Cursor cursor;
  cursor.mode = ProtoDiffer::Mode::kCheckDifferences;
  ::imp::proto::VisitPaired(&msg, &differ, &cursor, &other);
  return cursor.result;
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_DIFFER_H_
