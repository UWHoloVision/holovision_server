#pragma once

#include "DepthFrameTransformer.h"
#include "RGBFrameTransformer.h"
#include "FrameMessage.h"
#include "MeshMessage.h"
#include "PointCloudUtils.h"
#include "Visualizer.h"
#include "ColorSegmentation.h"
#include "Registration.h"

#include "FrameSocket.h"
#include "MeshSocket.h"

#include <glob.h>
#include <string.h>
#include <stdexcept>
#include <vector>
#include <sstream>

// debugging utils
namespace holovision {

void master_pipeline(std::string, int);
void colorpoints_pipeline();
void meshsocket_pipeline(std::string, int);

} // namespace holovision
