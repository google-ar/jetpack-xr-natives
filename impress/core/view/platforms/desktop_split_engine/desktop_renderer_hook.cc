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

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "third_party/grpc/include/grpcpp/security/server_credentials.h"
#include "third_party/grpc/include/grpcpp/server_builder.h"
#include "third_party/grpc/include/grpcpp/support/interceptor.h"
#include "third_party/grpc/include/grpcpp/support/server_interceptor.h"
#include "core/async/executor.h"
#include "core/common/registry.h"  // IWYU pragma: keep
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_local.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge_service_impl.h"
#include "core/view/base_view.h"
#include "core/view/platforms/desktop/desktop_hooks.h"

ABSL_FLAG(int32_t, port, 10000, "port to listen on");
ABSL_FLAG(bool, multimachine, false,
          "Whether to use multimachine Split Engine.");
ABSL_FLAG(
    uint32_t, mm_grpc_delay_ms, 0,
    "If non-zero, every gRPC call be delayed by this amount of milliseconds.");

namespace imp::split_engine {
namespace {

// An interceptor that adds a fixed delay before each RPC.
class GrpcDelayInterceptor : public grpc::experimental::Interceptor {
 public:
  explicit GrpcDelayInterceptor(absl::Duration delay) : delay_(delay) {}

  void Intercept(
      grpc::experimental::InterceptorBatchMethods* methods) override {
    absl::SleepFor(delay_);
    methods->Proceed();
  }

 private:
  const absl::Duration delay_;
};

// A factory to create new instances of the GrpcDelayInterceptor for the server.
class ServerDelayInterceptorFactory
    : public grpc::experimental::ServerInterceptorFactoryInterface {
 public:
  explicit ServerDelayInterceptorFactory(absl::Duration delay)
      : delay_(delay) {}

  grpc::experimental::Interceptor* CreateServerInterceptor(
      grpc::experimental::ServerRpcInfo* info) override {
    return new GrpcDelayInterceptor(delay_);
  }

 private:
  const absl::Duration delay_;
};

const bool kInstallPostHostSetupFn = imp::desktop_api::SetPostHostSetupFn(
    [](BaseView& view, Executor& executor) {
      if (absl::GetFlag(FLAGS_multimachine)) {
        IMP_LOG(imp::ERROR) << "Running Multimachine variant.";
        const std::string server_address =
            absl::StrCat("[::]:", absl::GetFlag(FLAGS_port));

        // Start gRPC server.
        auto service = std::make_unique<SplitEngineMMDesktopBridgeService>(
            std::make_unique<SplitEngineMMDesktopBridgeServiceImplLocal>(
                view, executor));
        grpc::ServerBuilder builder;
        // Listen on the given address
        // TODO: (broken link) - update server to use different credentials based
        // on the outcome of the bug.
        builder.AddListeningPort(server_address,
                                 grpc::InsecureServerCredentials());
        builder.RegisterService(service.get());

        if (absl::GetFlag(FLAGS_mm_grpc_delay_ms) > 0) {
          std::vector<std::unique_ptr<
              grpc::experimental::ServerInterceptorFactoryInterface>>
              interceptor_creators;
          interceptor_creators.push_back(
              std::make_unique<ServerDelayInterceptorFactory>(
                  absl::Milliseconds(absl::GetFlag(FLAGS_mm_grpc_delay_ms))));

          builder.experimental().SetInterceptorCreators(
              std::move(interceptor_creators));
        }

        // Lifetime of the service is connected to the lifetime of the view.
        view.GetRegistry().Register<SplitEngineMMDesktopBridgeService>(
            std::move(service));

        // Set up the server to start accepting requests.
        auto server = builder.BuildAndStart();
        
        view.GetRegistry().Register<grpc::Server>(std::move(server));
        IMP_LOG(imp::INFO) << "Server listening on " << server_address;
      } else {
        const std::string server_address =
            absl::StrCat("localhost:", absl::GetFlag(FLAGS_port));

        // Start gRPC server.
        auto service = std::make_unique<SplitEngineDesktopBridgeServiceImpl>(
            view, executor);
        grpc::ServerBuilder builder;
        // Listen on the given address
        // TODO: (broken link) - update server to use different credentials based
        // on the outcome of the bug.
        builder.AddListeningPort(server_address,
                                 grpc::InsecureServerCredentials());
        builder.RegisterService(service.get());

        // Lifetime of the service is connected to the lifetime of the view.
        view.GetRegistry().Register<SplitEngineDesktopBridgeServiceImpl>(
            std::move(service));

        // Set up the server to start accepting requests.
        auto server = builder.BuildAndStart();
        
        view.GetRegistry().Register<grpc::Server>(std::move(server));
        IMP_LOG(imp::INFO) << "Server listening on " << server_address;
      }
    });

}  // namespace
}  // namespace imp::split_engine
