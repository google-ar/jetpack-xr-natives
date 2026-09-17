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

// Returns a one-dimensional gaussian kernel with a sigma of 4 and a size of 16.
//
// Used to apply gaussian blurs in separate horizontal and vertical passes.
// It's more performant to do it this way than in a single pass with a 2D kernel
// since this way requires O(n) texture samples per fragment (in each pass) and
// 2D blur is O(n^2).
//
// See: https://en.wikipedia.org/wiki/Gaussian_blur#Implementation
//
// This kernel is calculated as follows:
//
// std::vector<float> CalculateGaussianKernel(int sigma, int size) {
//   std::vector<float> kernel;
//   kernel.reserve(size);
//
//   int half_size = size / 2;
//   for (int i = 0; i < size; i++) {
//     kernel.push_back(static_cast<float>(
//         1.0 / (sqrt(2.0 * M_PI) * sigma) *
//         exp(-(i - half_size) * (i - half_size) / (2.0 * sigma * sigma))));
//   }
//   return kernel;
// }
float[16] getGaussianKernel() {
  return float[16](0.013498, 0.021569, 0.032379, 0.045662, 0.060493, 0.075284,
                   0.088016, 0.096667, 0.099736, 0.096667, 0.088016, 0.075284,
                   0.060493, 0.045662, 0.032379, 0.021569);
}
