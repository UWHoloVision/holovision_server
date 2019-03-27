#pragma once

#include "DepthFrameTransformer.h"
#include "RGBFrameTransformer.h"
#include "FrameMessage.h"
#include "PointCloudUtils.h"
#include "Visualizer.h"

#include "FrameSocket.h"

// debugging utils
namespace holovision {

void colorpoints_pipeline();

void render_30_depth_frames_from_socket();

} // namespace holovision
