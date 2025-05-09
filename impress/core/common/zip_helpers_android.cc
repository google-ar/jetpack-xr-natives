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

#include <cstdint>
#include <memory>
#include <set>
#include <string>

#include "absl/status/status.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_context.h"
#include "core/common/jni_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/zip_helpers.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

JniUniquePtr<jbyteArray> BufferToJByteArray(JNIEnv* env,
                                            const BufferAccess& access) {
  JniUniquePtr<jbyteArray> result = CreateJniByteArray(env, access.Size());
  env->SetByteArrayRegion(result.get(), 0, access.Size(),
                          reinterpret_cast<const jbyte*>(access.Data()));
  return result;
}

class ByteArrayInputStream : public JavaWrapper {
 public:
  // Creates the ByteArrayInputStream with a unique_ptr to the jbyteArray From
  // BufferToJByteArray, which then immediately falls out of scope and gets
  // deleted after the JavaWrapper constructor.
  ByteArrayInputStream(JNIEnv* env, const BufferAccess& access)
      : JavaWrapper(env, "java/io/ByteArrayInputStream", "([B)V",
                    BufferToJByteArray(env, access).get()) {}

  ByteArrayInputStream(const ByteArrayInputStream& other) = delete;
  ByteArrayInputStream& operator=(const ByteArrayInputStream& other) = delete;
};

class ZipEntry : public JavaWrapper {
 public:
  ZipEntry(JNIEnv* env, jobject java_zip_entry)
      : JavaWrapper(env, "java/util/zip/ZipEntry") {
    SetSelf(java_zip_entry);

    get_name_ = GetMethodHandle("getName", "()Ljava/lang/String;");
    get_size_ = GetMethodHandle("getSize", "()J");
  }

  std::string GetName() { return CallStringMethod(get_name_); }

  int64_t GetSize() {
    jlong size = CallLongMethod(get_size_);
    return static_cast<int64_t>(size);
  }

 private:
  JniHandle get_name_;
  JniHandle get_size_;
};

class ZipInputStream : public JavaWrapper {
 public:
  ZipInputStream(JNIEnv* env, ByteArrayInputStream& input_stream)
      : JavaWrapper(env, "java/util/zip/ZipInputStream",
                    "(Ljava/io/InputStream;)V", input_stream.WeakReference()) {
    get_next_entry_ =
        GetMethodHandle("getNextEntry", "()Ljava/util/zip/ZipEntry;");
    read_ = GetMethodHandle("read", "([BII)I");
    close_ = GetMethodHandle("close", "()V");
  }

  ~ZipInputStream() override { CallVoidMethod(close_); }

  std::unique_ptr<ZipEntry> GetNextEntry() {
    jobject java_zip_entry = CallObjectMethod(get_next_entry_);
    if (Env()->ExceptionCheck()) {
      Env()->ExceptionDescribe();
      Env()->ExceptionClear();
      return nullptr;
    }

    if (!java_zip_entry) {
      return nullptr;
    }

    return std::make_unique<ZipEntry>(Env(), java_zip_entry);
  }

  void ReadEntry(uint8_t* out_data, int entry_size) {
    JniUniquePtr<jbyteArray> byte_array = CreateJniByteArray(Env(), entry_size);
    int offset = 0;
    int remaining_size = entry_size;
    while (remaining_size > 0) {
      jint bytes_read =
          CallIntMethod(read_, byte_array.get(), offset, remaining_size);
      offset += bytes_read;
      remaining_size -= bytes_read;
    }
    Env()->GetByteArrayRegion(byte_array.get(), 0, entry_size,
                              reinterpret_cast<jbyte*>(out_data));
  }

 private:
  JniHandle get_next_entry_;
  JniHandle read_;
  JniHandle close_;
};

OptionalError GetFilenamesFromZip(const BufferAccess& zip_access,
                                  std::vector<std::string>* out_filenames) {
  JniContext jni_context;
  JNIEnv* env = jni_context.GetJniEnv();
  ByteArrayInputStream input_stream(env, zip_access);
  ZipInputStream zip_input_stream(env, input_stream);

  while (std::unique_ptr<ZipEntry> zip_entry =
             zip_input_stream.GetNextEntry()) {
    std::string entry_name = zip_entry->GetName();
    out_filenames->push_back(entry_name);
  }

  return NoError();
}

absl::StatusOr<std::vector<ZipFile>> GetFilesFromZip(
    const BufferAccess& zip_access, const std::set<std::string>& filenames) {
  std::vector<ZipFile> result;

  if (filenames.empty()) {
    return result;
  }

  JniContext jni_context;
  JNIEnv* env = jni_context.GetJniEnv();
  ByteArrayInputStream input_stream(env, zip_access);
  ZipInputStream zip_input_stream(env, input_stream);

  while (std::unique_ptr<ZipEntry> zip_entry =
             zip_input_stream.GetNextEntry()) {
    std::string entry_name = zip_entry->GetName();
    auto it = filenames.find(entry_name);
    if (it != filenames.end()) {
      int64_t file_size = zip_entry->GetSize();
      if (file_size < 0) {
        // TODO: Implement support for data descriptors.
        return absl::InternalError(absl::StrFormat(
            "Unable to unzip file %s, file size is not known (data descriptor "
            "is not supported)",
            zip_entry->GetName()));
      }
      BufferAccess file_buffer;
      uint8_t* data = BufferAccess::Create(file_size, &file_buffer);
      zip_input_stream.ReadEntry(data, file_size);
      result.emplace_back(std::move(entry_name), std::move(file_buffer));
      if (result.size() == filenames.size()) {
        return result;
      }
    }
  }
  if (result.size() != filenames.size()) {
    return absl::InternalError("Unable to unzip all files.");
  }
  return result;
}

}  // namespace imp
