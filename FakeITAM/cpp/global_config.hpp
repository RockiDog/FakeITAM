//
//  global_config.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/26.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_GLOBAL_CONFIG_HPP_
#define FAKEITAM_CPP_GLOBAL_CONFIG_HPP_

#include <limits>

namespace fakeitam {
namespace config {

/* Init config flags */
const unsigned char USE_LIVE_DATA          = 0x01 << 0;  /* b0000_0001 */
const unsigned char USE_DEPTH_TRACKING     = 0x01 << 1;  /* b0000_0010 */
const unsigned char USE_COLOR_TRACKING     = 0x01 << 2;  /* b0000_0100 */
const unsigned char USE_FEATURES_TRACKING  = 0x01 << 3;  /* b0000_1000 */
const unsigned char USE_MEMORY_SWAPPING    = 0x01 << 4;  /* b0001_0000 */
const unsigned char USE_FORWARD_PROJECTION = 0x01 << 5;  /* b0010_0000 */
const unsigned char USE_GPU_ACCELERATION   = 0x01 << 6;  /* b0100_0000 */
const unsigned char USE_DEBUG_MODE         = 0x01 << 7;  /* b1000_0000 */

const unsigned char gInitializationFlags = USE_DEPTH_TRACKING |
                                           //USE_FORWARD_PROJECTION |
                                           USE_DEBUG_MODE;

const float gMathFloatEpsilon = std::numeric_limits<float>::epsilon();
const double gMathDoubleEpsilon = std::numeric_limits<double>::epsilon();

const int gViewKernalSize = 5;

const int gDepthTrackIcpLevelNum = 3;
const int gDepthTrackIcpZetaMax[] = {10, 5, 4};
const int gDepthTrackMinValidPixelNum = 100;
const float gDepthTrackMaxPixelError = 2;
const float gDepthTrackMaxErrorPerPixel = 1e20;
const float gDepthTrackTopIcpThreshold = 0.01;
const float gDepthTrackConvergenceThreshold = 1e-3;

const float gTsdfBandWidthMu = 0.02;  /* in meter */
const int gTsdfMaxWeight = 100;

/* gBlockHashOrderedArraySize + gBlockHashExcessListSize */
const int gBlockHashMapSize          = 0x120000;
const int gBlockHashOrderedArraySize = 0x100000;
const int gBlockHashExcessListSize   =  0x20000;
const int gBlockHashLocalNum         =  0x40000;

const float gVoxelMetricSize = 0.001;  /* in meter */
const int gVoxelBlockSizeL =   8;      /* linear */
const int gVoxelBlockSizeQ =  64;      /* quadric */
const int gVoxelBlockSizeC = 512;      /* cubic */

const float gRaycastRangeZMin = 0.05;
const float gRaycastRangeZMax = 999999.9;
const float gRaycastSmallTsdfMin = -0.9;
const float gRaycastSmallTsdfMax = 0.9;
const int gBoundBoxFragmentSizeL = 16;  /* linear */
const int gBoundBoxMaxFragmentNum = 65536 << 2;
const int gBoundBoxSubsample = 8;

const int gRenderMaxPointCloudAge = 5;
const float gRenderMaxCameraDistance2 = 0.0005;  /* in meter^2 */

}
}

#endif  /* FAKEITAM_CPP_GLOBAL_CONFIG_HPP_ */
