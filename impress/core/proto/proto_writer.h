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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_WRITER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/copyable_ptr.h"
#include "core/common/optional_with_default.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_common.h"
#include "boost_beast/varint.hpp"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace proto {

class WriterImpl {
 public:
  using Cursor = char*;

  explicit WriterImpl(std::string* str, const std::vector<size_t>& sizes)
      : str_(str), next_size_(sizes.cbegin()), end_size_(sizes.cend()) {}

  bool Finish(char* end) {
    assert(end == str_->data() + str_->size());
    return true;
  }

 protected:
  char* WriteVarint(char* ptr, uint32_t varint) {
    boost::beast::detail::varint_write(ptr, varint);
    assert(ptr <= str_->data() + str_->size());
    return ptr;
  }
  char* WriteVarint(char* ptr, uint64_t varint) {
    boost::beast::detail::varint_write(ptr, varint);
    assert(ptr <= str_->data() + str_->size());
    return ptr;
  }
  char* WriteVarint(char* ptr, int varint) {
    return WriteVarint(ptr, static_cast<uint64_t>(varint));
  }

  char* StartDelimited(char* ptr, int*) {
    assert(next_size_ < end_size_);
    return WriteVarint(ptr, static_cast<uint64_t>(*next_size_++));
  }
  char* EndDelimited(char* ptr, int) { return ptr; }

  char* Write32(char* ptr, const void* v32) {
    memcpy(ptr, v32, 4);
    assert(ptr + 4 <= str_->data() + str_->size());
    return ptr + 4;
  }
  char* Write64(char* ptr, const void* v64) {
    memcpy(ptr, v64, 8);
    assert(ptr + 8 <= str_->data() + str_->size());
    return ptr + 8;
  }

  template <typename Proto>
  char* WriteStandardProto(char* ptr, Proto* proto) {
    size_t num_bytes = proto->ByteSizeLong();
    proto->SerializeToArray(ptr, num_bytes);
    return ptr + num_bytes;
  }

  char* Copy(char* ptr, const void* data, size_t size) {
    memcpy(ptr, data, size);
    assert(ptr + size <= str_->data() + str_->size());
    return ptr + size;
  }

 private:
  std::string* str_;
  std::vector<size_t>::const_iterator next_size_;
  std::vector<size_t>::const_iterator end_size_;
};

class SizeImpl {
 public:
  using Cursor = size_t;

  explicit SizeImpl(std::vector<size_t>* sizes) : sizes_(sizes) {}

  size_t Finish(Cursor size) { return size; }

 protected:
  Cursor WriteVarint(Cursor size, uint32_t varint) {
    return size + boost::beast::detail::varint_size(varint);
  }
  Cursor WriteVarint(Cursor size, uint64_t varint) {
    return size + boost::beast::detail::varint_size(varint);
  }
  Cursor WriteVarint(Cursor size, int varint) {
    return WriteVarint(size, static_cast<uint64_t>(varint));
  }

  Cursor StartDelimited(Cursor size, int* mark) {
    *mark = sizes_->size();
    sizes_->emplace_back(size);
    return size;
  }
  Cursor EndDelimited(Cursor size, int mark) {
    (*sizes_)[mark] = size - (*sizes_)[mark];
    return size + boost::beast::detail::varint_size((*sizes_)[mark]);
  }

  template <typename Proto>
  Cursor WriteStandardProto(Cursor ptr, Proto* proto) {
    return ptr + proto->ByteSizeLong();
  }

  Cursor Write32(Cursor size, const void* v32) { return size + 4; }
  Cursor Write64(Cursor size, const void* v64) { return size + 8; }
  Cursor Copy(Cursor cursor, const void*, size_t size) { return cursor + size; }

 private:
  std::vector<size_t>* sizes_;
};

template <typename Impl>
class ProtoWriter : public Impl {
 public:
  using Cursor = typename Impl::Cursor;

  template <typename... Args>
  explicit ProtoWriter(Args&&... args) : Impl(std::forward<Args>(args)...) {}

  template <int field_type, typename T>
  Cursor Visit(Cursor ptr, int field_id, T* field, T* other,
               bool optional = false);

  template <int field_type, typename T>
  Cursor Visit(Cursor ptr, int field_id, absl::optional<T>* field,
               absl::optional<T>* other, bool optional = false);

  template <int field_type, typename T, const auto* DefaultValuePointer>
  Cursor Visit(Cursor ptr, int field_id,
               OptionalWithDefault<T, DefaultValuePointer>* field,
               OptionalWithDefault<T, DefaultValuePointer>* other,
               bool optional = false);

  template <int field_type, typename T>
  Cursor Visit(Cursor ptr, int field_id, CopyablePtr<T>* field,
               CopyablePtr<T>* other, bool optional = false);

  template <int field_type, proto::RepeatedMergeStrategy merge_type, typename T>
  Cursor Visit(Cursor ptr, int field_id, std::vector<T>* field,
               std::vector<T>* other, bool optional = false);

  template <int key_type, int value_type, typename K, typename V>
  Cursor Visit(Cursor ptr, int field_id, std::map<K, V>* field,
               std::map<K, V>* other, bool optional = false);

  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, bool* field, bool* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, int32_t* field, int32_t* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, uint32_t* field, uint32_t* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, int64_t* field, int64_t* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, uint64_t* field, uint64_t* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, float* field, float* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, double* field, double* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, std::string* field, std::string* other,
               bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, absl::string_view* field,
               absl::string_view* other, bool optional = false);
  template <int field_type>
  Cursor Visit(Cursor ptr, int field_id, absl::Cord* field, absl::Cord* other,
               bool optional = false);

  template <typename Proto, const auto* DefaultValuePointer>
  Cursor VisitStandardProto(
      Cursor ptr, int field_id,
      OptionalWithDefault<Proto, DefaultValuePointer>* proto,
      OptionalWithDefault<Proto, DefaultValuePointer>* other,
      bool optional = false);

  template <typename Proto>
  Cursor VisitStandardProto(Cursor ptr, int field_id, Proto* proto,
                            Proto* other);

  Cursor Unknown(Cursor ptr) { return ptr; }

 private:
  Cursor WriteZigzag(Cursor ptr, int32_t zigzag);
  Cursor WriteZigzag(Cursor ptr, int64_t zigzag);

  template <int field_type, typename T>
  Cursor WriteInt(Cursor ptr, int field_id, T& field, bool optional);
  template <int field_type, typename T>
  Cursor WriteFloat(Cursor ptr, int field_id, T& field, bool optional);
};

template <typename Impl>
template <int field_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               T* field, T* other,
                                               bool optional) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    static_assert(field_type == TYPE_ENUM);
    if (field_id) {
      if (!*field && !optional) {
        return ptr;
      }
      ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<TYPE_ENUM>());
    }
    ptr = Impl::WriteVarint(ptr, static_cast<uint32_t>(*field));
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<TYPE_MESSAGE>());
    int mark;
    ptr = Impl::StartDelimited(ptr, &mark);
    ptr = ::imp::proto::Visit(field, this, ptr);
    ptr = Impl::EndDelimited(ptr, mark);
  }
  return ptr;
}

template <typename Impl>
template <int field_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               absl::optional<T>* field,
                                               absl::optional<T>* other,
                                               bool optional) {
  if (!field->has_value()) {
    return ptr;
  }
  return Visit<field_type>(ptr, field_id, &(**field), static_cast<T*>(nullptr),
                           true);
}

template <typename Impl>
template <int field_type, typename T, const auto* DefaultValuePointer>
typename Impl::Cursor ProtoWriter<Impl>::Visit(
    Cursor ptr, int field_id,
    OptionalWithDefault<T, DefaultValuePointer>* field,
    OptionalWithDefault<T, DefaultValuePointer>* other, bool optional) {
  if (!field->HasValue()) {
    return ptr;
  }
  return Visit<field_type>(ptr, field_id, &field->MutableValue(),
                           static_cast<T*>(nullptr), true);
}

template <typename Impl>
template <int field_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               CopyablePtr<T>* field,
                                               CopyablePtr<T>* other,
                                               bool optional) {
  if (!*field && !optional) {
    return ptr;
  }
  return Visit<field_type>(ptr, field_id, field->get(),
                           static_cast<T*>(nullptr), optional);
}

template <typename Impl>
template <int field_type, proto::RepeatedMergeStrategy merge_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               std::vector<T>* field,
                                               std::vector<T>* other,
                                               bool optional) {
  if (field->empty() && !optional) {
    return ptr;
  }
  if (IsPacked(field_type)) {
    if (field_id) {
      ptr = Impl::WriteVarint(ptr, field_id << 3 | WIRE_DELIMITED);
    }
    int mark;
    ptr = Impl::StartDelimited(ptr, &mark);
    for (auto& value : *field) {
      ptr =
          Visit<field_type>(ptr, 0, &value, static_cast<T*>(nullptr), optional);
    }
    ptr = Impl::EndDelimited(ptr, mark);
  } else {
    for (auto& value : *field) {
      ptr = Visit<field_type>(ptr, field_id, &value, static_cast<T*>(nullptr),
                              optional);
    }
  }
  return ptr;
}

template <typename Impl>
template <int key_type, int value_type, typename K, typename V>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               std::map<K, V>* field,
                                               std::map<K, V>* other,
                                               bool optional) {
  if (field->empty() && !optional) {
    return ptr;
  }
  constexpr int kKeyId = 1;
  constexpr int kValueId = 2;
  for (auto& [key, value] : *field) {
    ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<TYPE_MESSAGE>());
    int mark;
    ptr = Impl::StartDelimited(ptr, &mark);
    ptr = Visit<key_type>(ptr, kKeyId, const_cast<K*>(&key),
                          static_cast<K*>(nullptr));
    ptr = Visit<value_type>(ptr, kValueId, const_cast<V*>(&value),
                            static_cast<V*>(nullptr));
    ptr = Impl::EndDelimited(ptr, mark);
  }
  return ptr;
}

template <typename Impl>
typename Impl::Cursor ProtoWriter<Impl>::WriteZigzag(Cursor ptr,
                                                     int32_t zigzag) {
  return Impl::WriteVarint(ptr, (static_cast<uint32_t>(zigzag) << 1) ^
                                    (static_cast<uint32_t>(zigzag >> 31)));
}

template <typename Impl>
typename Impl::Cursor ProtoWriter<Impl>::WriteZigzag(Cursor ptr,
                                                     int64_t zigzag) {
  return Impl::WriteVarint(ptr, (static_cast<uint64_t>(zigzag) << 1) ^
                                    (static_cast<uint64_t>(zigzag >> 63)));
}

template <typename Impl>
template <int field_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::WriteInt(Cursor ptr, int field_id,
                                                  T& field, bool optional) {
  if (field_id) {
    if (!field && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<field_type>());
  }
  static_assert(field_type == TYPE_INT32 || field_type == TYPE_UINT32 ||
                field_type == TYPE_INT64 || field_type == TYPE_UINT64 ||
                field_type == TYPE_SINT32 || field_type == TYPE_SINT64 ||
                field_type == TYPE_FIXED32 || field_type == TYPE_SFIXED32 ||
                field_type == TYPE_FIXED64 || field_type == TYPE_SFIXED64);

  if constexpr (field_type == TYPE_INT32 || field_type == TYPE_UINT32 ||
                field_type == TYPE_INT64 || field_type == TYPE_UINT64) {
    ptr = Impl::WriteVarint(ptr, static_cast<uint64_t>(field));
  } else if constexpr (field_type == TYPE_FIXED32 ||
                       field_type == TYPE_SFIXED32) {
    ptr = Impl::Write32(ptr, &field);
  } else if constexpr (field_type == TYPE_FIXED64 ||
                       field_type == TYPE_SFIXED64) {
    ptr = Impl::Write64(ptr, &field);
  } else if constexpr (field_type == TYPE_SINT32) {
    ptr = WriteZigzag(ptr, static_cast<int32_t>(field));
  } else {
    ptr = WriteZigzag(ptr, static_cast<int64_t>(field));
  }
  return ptr;
}

template <typename Impl>
template <int field_type, typename T>
typename Impl::Cursor ProtoWriter<Impl>::WriteFloat(Cursor ptr, int field_id,
                                                    T& field, bool optional) {
  if (field_id) {
    if (field == 0.0f && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<field_type>());
  }

  static_assert(field_type == TYPE_FLOAT || field_type == TYPE_DOUBLE);
  if constexpr (field_type == TYPE_FLOAT) {
    ptr = Impl::Write32(ptr, &field);
  } else {
    ptr = Impl::Write64(ptr, &field);
  }
  return ptr;
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               bool* field, bool* other,
                                               bool optional) {
  if (field_id) {
    if (!*field && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | 0);
  }
  ptr = Impl::WriteVarint(ptr, *field ? 1 : 0);
  return ptr;
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               int32_t* field, int32_t* other,
                                               bool optional) {
  return WriteInt<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               uint32_t* field, uint32_t* other,
                                               bool optional) {
  return WriteInt<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               int64_t* field, int64_t* other,
                                               bool optional) {
  return WriteInt<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               uint64_t* field, uint64_t* other,
                                               bool optional) {
  return WriteInt<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               float* field, float* other,
                                               bool optional) {
  return WriteFloat<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               double* field, double* other,
                                               bool optional) {
  return WriteFloat<field_type>(ptr, field_id, *field, optional);
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               std::string* field,
                                               std::string* other,
                                               bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | 2);
  }
  ptr = Impl::WriteVarint(ptr, static_cast<uint64_t>(field->size()));
  return Impl::Copy(ptr, field->data(), field->size());
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               absl::string_view* field,
                                               absl::string_view* other,
                                               bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | 2);
  }
  ptr = Impl::WriteVarint(ptr, static_cast<uint64_t>(field->size()));
  return Impl::Copy(ptr, field->data(), field->size());
}

template <typename Impl>
template <int field_type>
typename Impl::Cursor ProtoWriter<Impl>::Visit(Cursor ptr, int field_id,
                                               absl::Cord* field,
                                               absl::Cord* other,
                                               bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return ptr;
    }
    ptr = Impl::WriteVarint(ptr, field_id << 3 | 2);
  }
  absl::Cord copy = *field;
  auto view = copy.Flatten();
  ptr = Impl::WriteVarint(ptr, static_cast<uint64_t>(view.size()));
  return Impl::Copy(ptr, view.data(), view.size());
}

template <typename Impl>
template <typename Proto, const auto* DefaultValuePointer>
typename Impl::Cursor ProtoWriter<Impl>::VisitStandardProto(
    Cursor ptr, int field_id,
    OptionalWithDefault<Proto, DefaultValuePointer>* proto,
    OptionalWithDefault<Proto, DefaultValuePointer>* other, bool optional) {
  if (!proto->HasValue()) {
    return ptr;
  }
  return VisitStandardProto(ptr, field_id, &proto->MutableValue(),
                            static_cast<Proto*>(nullptr));
}

template <typename Impl>
template <typename Proto>
typename Impl::Cursor ProtoWriter<Impl>::VisitStandardProto(Cursor ptr,
                                                            int field_id,
                                                            Proto* proto,
                                                            Proto* other) {
  ptr = Impl::WriteVarint(ptr, field_id << 3 | WireType<TYPE_MESSAGE>());
  int mark;
  ptr = Impl::StartDelimited(ptr, &mark);
  ptr = Impl::WriteStandardProto(ptr, proto);
  ptr = Impl::EndDelimited(ptr, mark);
  return ptr;
}

namespace internal {

std::vector<size_t>* ScratchSizes();

}  // namespace internal

template <typename T>
bool SerializeTo(const T* msg, std::string* str) {
  T* m = const_cast<T*>(msg);
  // Figure out the sizes of all delimited pieces.
  auto* scratch = internal::ScratchSizes();
  ProtoWriter<SizeImpl> sizes(scratch);
  size_t size = ::imp::proto::Visit(m, &sizes, 0);

  // Now write the actual stream.
  str->clear();
  str->resize(size);
  ProtoWriter<WriterImpl> stream(str, *scratch);
  auto end = ::imp::proto::Visit(m, &stream, str->data());
  return stream.Finish(end);
}

template <typename T>
bool SerializeTo(const T* msg, absl::Cord* cord) {
  std::string tmp;
  if (SerializeTo(msg, &tmp)) {
    cord->Clear();
    cord->Append(tmp);
    return true;
  }
  return false;
}

// Packs a proto message into the Any out parameter passed in.
template <typename T>
absl::Status PackAny(const T& message,
                     google::protobuf::imp_proto::Any* out_any) {
  if (out_any == nullptr) {
    return absl::InvalidArgumentError("out_any is nullptr");
  }

  T* m = const_cast<T*>(&message);
  if (!proto::SerializeTo(m, &out_any->value)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Unable to serialize message of type %s into an any.", T::kTypeUrl));
  }

  out_any->type_url = T::kTypeUrl;

  return absl::OkStatus();
}

// Packs a proto message into an Any.
template <typename T>
absl::StatusOr<google::protobuf::imp_proto::Any> PackAny(const T& message) {
  google::protobuf::imp_proto::Any result;
  MP_RETURN_IF_ERROR(PackAny(message, &result));
  return result;
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_WRITER_H_
