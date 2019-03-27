#pragma once

#include "DepthFrameTransformer.h"
#include "FrameMessage.h"
#include "MeshMessage.h"
#include "PointCloudUtils.h"
#include "Visualizer.h"

#include "FrameSocket.h"
#include "MeshSocket.h"

// debugging utils
namespace holovision {

void render_30_depth_frames_as_mesh();

void render_30_depth_frames_from_socket();

} // namespace holovision
