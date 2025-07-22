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

// Color transfer of a media asset. The enum values are defined to match
// the values returned by media3. Please see here for more information:
// third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
// For the enum values (except sRGB and Gamma 2.2), please see here in the
// Android codebase:
// https://cs.android.com/android/platform/superproject/main/+/main:frameworks/base/media/java/android/media/MediaFormat.java
const int kTransferFunctionLinear = 1;
const int kTransferFunctionSRGB = 2;
const int kTransferFunctionSMPTE170M = 3;
const int kTransferFunctionGamma22 = 10;
const int kTransferFunctionST2084 = 6;
const int kTransferFunctionHLG = 7;

// According to BT2408, HDR Reference White of 203cd/m^3 corresponds to a 75%
// HLG signal (assuming the 1000cd/m^3 peak luminance reference display).  To
// map this to SDR Peak White (80cd/m^3) we can exploit HLG's backward
// compatibility with SDR by applying the HLG EOTF as if for a display whose
// peak luminance is 302cd/m^3. At this luminance, gamma is calculated to == 1.f
// resulting in a no-op OOTF. Thus, to get to SDR-relative linear color we only
// need to apply the inverse OETF and apply a scaling factor. More details can
// be found in https://www.w3.org/Graphics/Color/Workshop/slides/talk/cotton2.
highp const float maxHlgToSdrDisplayLuminance = 302.f;

// Calculates the maximum component of a vec3.
highp float maxOf(highp float3 value) {
    return max(value.r, max(value.g, value.b));
}

// Converts a PQ-encoded color to display-linear color in cd/m^3.
// Returns: color in the range [0,10000]
highp float3 pqToLuminance(highp float3 v_input) { // Changed 'v' to 'v_input' for clarity
    // Clamp the input PQ signal to its defined valid range [0,1]
    highp float3 v = clamp(v_input, 0.0, 1.0);

    // See https://www.itu.int/rec/R-REC-BT.2100 regarding the PQ EOTF for
    // background on the calculation. m1,m2 are inverted from the spec
    // definitions since they're always used inverted.
    const highp float3 m1 = float3(16384.0 / 2610.0);
    const highp float3 m2 = float3(32.0 / 2523.0);
    const highp float3 c2 = float3(2413.0 / 128.0);
    const highp float3 c3 = float3(2392.0 / 128.0);
    const highp float3 c1 = c3 - c2 + float3(1.0);
    highp float3 e = pow(v, m2);
    return 10000.0 * pow(max(e - c1, float3(0.0)) / (c2 - c3 * e), m1);
}

// Calculates display-linear color from BT2020 scene-linear color.
//
// color - Normalized BT2020 scene-linear color in the range [0,1].
// Lw - The display's max luminance of white in cd/m^3.
// luminance[out] - A byproduct of the calculation for potential reuse.
//
// Returns: Display-linear color expressed in cd/m^3.
// See https://www.itu.int/rec/R-REC-BT.2100 Table 5 for more details.
highp float3 hlgOOTF(highp float3 color, highp float Lw, out highp float luminance) {
    // The BT2020 constants to calculate "true luminance" from linear RGB.
    const highp float3 Y = float3(0.2627, 0.6780, 0.0593);

    // For displays where maxOutputLum > 1000. Otherwise gamma should be 1.2.
    highp float gamma = 1.2 + 0.42 * log(Lw / 1000.0) / log(10.0);

    luminance = dot(Y, color);

    return Lw * pow(luminance, gamma - 1.0) * color;
}

// color - HLG-encoded color in the range [0,1].
// Returns: Normalized scene-linear color in the range [0,1].
highp float hlgInvOETF(highp float color) {
    const highp float a = 0.17883277;
    const highp float b = 1.0 - 4.0 * a;
    const highp float c = 0.5 - a * log(4.0 * a);
    return color <= 0.5 ? color * color / 3.0 : (exp((color - c) / a) + b) / 12.0;
}

highp float3 hlgInvOETF(highp float3 color) {
    return float3(
        hlgInvOETF(color.r),
        hlgInvOETF(color.g),
        hlgInvOETF(color.b));
}

// Converts an HLG-encoded color into linear color in the range [0,302].
highp float3 hlgToLuminance(highp float3 color) {
    // Clamp the input HLG signal to its defined valid range [0,1]
    highp float3 clamped_color = clamp(color, 0.0, 1.0);
    float ignore;
    return hlgOOTF(hlgInvOETF(clamped_color), /*Lw=*/maxHlgToSdrDisplayLuminance, /*luminance=*/ignore);
}

highp float smpte170mToLinear(highp float channel) {
    // Section 5.2 in https://www.itu.int/rec/R-REC-BT.1700-0-200502-I/en
    return channel < 0.0812
                ? channel / 4.500
                : pow((channel + 0.099) / 1.099, 1.0 / 0.45);
}

highp float3 smpte170mToLinear(highp float3 color) {
    return float3(
        smpte170mToLinear(color.r),
        smpte170mToLinear(color.g),
        smpte170mToLinear(color.b));
}

// Optimized for round trip accuracy with respect to the sRGB
// gamma -> linear -> gamma conversion. Parameters optimized
// using scipy.optimize.leastsq() for residuals to the function
// linear_to_gamma_sRGB(gamma22ToLinear(srgb)) - srgb
// for srgb sampled uniformly between 0 and 1
highp float3 gamma22ToLinear(highp float3 rgb) {
    const highp float a = -0.29556651;
    const highp float b = 0.82937976;
    const highp float c = 0.40424319;
    const highp float d = 0.05458958;
    highp float3 x = max(float3(0.0), rgb);

    // Quartic polynomial fit gives < 1 / 256 errors for full range
    highp float3 res = a * (x * x * x * x) + b * (x * x * x) + c * (x * x) + d * x;
    return clamp(res, float3(0.0), float3(1.0));
}

highp float3 srgbToLinear(highp float3 color) {
    return gamma22ToLinear(color);
}

// Tone map the input color into SDR.
//
// The units of the input quantities don't matter (i.e. they can be nits or
// pixel values). All arguments must use the same units.
//
// Lc - Content max luminance (in the input image).
// Ld - Display max luminance.
// color - Linear color to be tonemapped.
// luminance - The luminance of the color.
highp float3 tonemapReinhard(highp float Lc, highp float Ld, highp float luminance, highp float3 color) {
    highp float a = Ld / (Lc * Lc);
    highp float b = 1.0 / Ld;
    return color * float3((1.0 + a * luminance) / (1.0 + b * luminance));
}

// TODO: b/400780684 - Investigate if we need to rescale the color to full range.
highp float3 convertColor(highp float3 color, float3x3 colorTransformMatrix) {
    // Assume the input color is already in linear space.
    highp float3 finalColor = color;

    if (materialParams.transferFunction == kTransferFunctionSRGB) {
        finalColor = srgbToLinear(color);
    }

    if (materialParams.transferFunction == kTransferFunctionSMPTE170M) {
        finalColor = smpte170mToLinear(color);
    }

    if (materialParams.transferFunction == kTransferFunctionGamma22) {
        finalColor = gamma22ToLinear(color);
    }

    if (materialParams.transferFunction == kTransferFunctionST2084) {
        // 10000 is the maximum absolute luminance of ST2084 (PQ).
        // TODO - b/410027510: Update SpF shader code to compare maxContentLightLevel to 0.
        // TODO - b/415049056: Move this computation out of the shader code to avoid repeating
        // them for each pixel.
        highp float maxInputLum =
            (materialParams.maxContentLightLevel > 0) ?
            min(float(materialParams.maxContentLightLevel), 10000.0) : 10000.0;
        const highp float maxOutputLum = 203.0; // SDR reference white.

        // Calculate relative luminance (1.0 = SDR white).
        highp float relativeMaxInputLum = maxInputLum / maxOutputLum;

        // Convert to absolute luminance (nits).
        highp float3 linearColor = pqToLuminance(color);

        // Scale to relative luminance (1.0 = SDR white).
        finalColor = linearColor / maxOutputLum;

        // Tonemap
        finalColor = tonemapReinhard(relativeMaxInputLum, 1.0, maxOf(finalColor), finalColor);
    }

    // HLG transfer function.
    if (materialParams.transferFunction == kTransferFunctionHLG) {
        // 1000 is the maximum absolute luminance of HLG.
        // TODO - b/415049056: Move this computation out of the shader code to avoid repeating
        // them for each pixel.
        highp float maxInputLum =
            (materialParams.maxContentLightLevel > 0) ?
            min(float(materialParams.maxContentLightLevel), 1000.0) : 1000.0;
        const highp float maxOutputLum = 203.0;
        highp float relativeMaxInputLum = maxInputLum / maxOutputLum;

        // Convert to absolute luminance (nits).
        highp float3 linearColor = hlgToLuminance(color);

        // Scale to relative luminance (1.0 = SDR white).
        finalColor = linearColor / maxOutputLum;

        // Tonemap
        finalColor = tonemapReinhard(relativeMaxInputLum, 1.0, maxOf(finalColor), finalColor);
    }

    finalColor = colorTransformMatrix * finalColor;
    return clamp(finalColor, 0.0, 1.0);
}
