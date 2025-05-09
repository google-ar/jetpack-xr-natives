// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Creates a 3x3 matrix from the first three components of the material global vectors.
//
// This function constructs a matrix using the x, y, and z components of the first
// material global vectors (0, 1, and 2). These global vectors are in this context set to
// provide a color conversion matrix.
mat3 createMat3fFromMaterialGlobalVectors() {
    return mat3(
        getMaterialGlobal0().xyz,
        getMaterialGlobal1().xyz,
        getMaterialGlobal2().xyz
    );
}

// Applies a color conversion matrix to the input color.
//
// This can be used for various color space conversions defined globally for the material.
vec3 applyGlobalMaterialConversionMatrix(vec3 materialColor) {
    mat3 globalMaterialConversionMatrix = createMat3fFromMaterialGlobalVectors();
    return globalMaterialConversionMatrix * materialColor;
}
