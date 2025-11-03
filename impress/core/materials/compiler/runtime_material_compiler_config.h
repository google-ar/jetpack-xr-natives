/*
 * Copyright 2025 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_CONFIG_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/filament-matp/include/filament-matp/Config.h"

namespace imp {
class BufferOutput : public matp::Config::Output {
 public:
  explicit BufferOutput(std::ostream& stream) : stream_(stream) {};

  ~BufferOutput() override = default;
  // The matp::Config was designed to be take in file inputs and outputs
  // So the Input and Output classes have functions named open and close.
  // However, for our use case, we are passing in a string as input, so we
  // override the open function to return the size of the string.
  bool open() noexcept override { return true; }

  bool write(const uint8_t* data, size_t size) noexcept override {
    stream_.write(reinterpret_cast<const char*>(data), size);
    return !stream_.fail();
  }

  std::ostream& getOutputStream() noexcept override { return stream_; }

  bool close() noexcept override { return !stream_.fail(); };

 private:
  std::ostream& stream_;
};

class BufferInput : public matp::Config::Input {
 public:
  explicit BufferInput(const std::string_view& data) : data_(data) {}

  ~BufferInput() override = default;

  // The matp::Config was designed to be take in file inputs and outputs
  // So the Input and Output classes have functions named open and close.
  // However, for our use case, we are passing in a string as input, so we
  // override the open function to return the size of the string.
  ssize_t open() noexcept override { return data_.size(); }

  std::unique_ptr<const char[]> read() noexcept override {
    auto buffer = std::make_unique<char[]>(data_.size());
    std::memcpy(buffer.get(), data_.data(), data_.size());
    return buffer;
  }

  bool close() noexcept override { return true; }

  const char* getName() const noexcept override { return "BufferInput"; }

 private:
  std::string_view data_;
};

class RuntimeMaterialCompilerConfig : public matp::Config {
 public:
  RuntimeMaterialCompilerConfig(const std::string_view& input,
                                std::ostream& output)
      : input_(std::make_unique<BufferInput>(input)),
        output_(std::make_unique<BufferOutput>(output)) {};

  ~RuntimeMaterialCompilerConfig() override = default;

  Output* getOutput() const noexcept override { return output_.get(); }

  Input* getInput() const noexcept override { return input_.get(); }

  // No-op. We don't use this in our runtime material compiler.
  std::string toString() const noexcept override { return ""; }

  // Not used for prod. This was recently introduced for debugging purposes in
  // matc.
  std::string toPIISafeString() const noexcept override { return ""; }

  RuntimeMaterialCompilerConfig& SetDebug(bool debug) noexcept {
    mDebug = debug;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetPlatform(Platform platform) noexcept {
    mPlatform = platform;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetOutputFormat(OutputFormat format) noexcept {
    mOutputFormat = format;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetOptimizationLevel(
      Optimization level) noexcept {
    mOptimizationLevel = level;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetTargetApi(TargetApi target_api) noexcept {
    mTargetApi = target_api;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetSamplerValidation(
      bool sampler_validation) noexcept {
    mNoSamplerValidation = !sampler_validation;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetIncludeEssl1(bool include_essl1) noexcept {
    mIncludeEssl1 = include_essl1;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetVariantFilter(
      filament::UserVariantFilterMask variant_filter) noexcept {
    mVariantFilter = variant_filter;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetDefines(
      const StringReplacementMap& defines) noexcept {
    mDefines = defines;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetTemplateMap(
      const StringReplacementMap& template_map) noexcept {
    mTemplateMap = template_map;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetMaterialParameters(
      const StringReplacementMap& material_parameters) noexcept {
    mMaterialParameters = material_parameters;
    return *this;
  }

  RuntimeMaterialCompilerConfig& SetFeatureLevel(
      filament::backend::FeatureLevel feature_level) noexcept {
    mFeatureLevel = feature_level;
    return *this;
  }

 private:
  std::unique_ptr<BufferInput> input_;
  std::unique_ptr<BufferOutput> output_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALCOMPILER_RUNTIME_MATERIAL_COMPILER_CONFIG_H_
