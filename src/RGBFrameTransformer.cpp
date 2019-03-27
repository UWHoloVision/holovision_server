#include "RGBFrameTransformer.h"

namespace holovision {

RGBFrameTransformer::RGBFrameTransformer(
    FrameMessage&& frame
): _frame(std::move(frame)), _camera_projection(frame.intrinsics){}

void RGBFrameTransformer::compute_world_to_cameraview(){
    _world_to_camera_view = _frame.extrinsics.transpose() *
        _frame.frame_to_origin.transpose().inverse();
}

void RGBFrameTransformer::get_RGBD_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::MatrixXf&& world_pts){
    compute_world_to_cameraview();
    // cameraview points
    Eigen::MatrixXf cameraview_pts = (
        _world_to_camera_view * world_pts
    ).transpose();
    // add col of ones
    cameraview_pts.col(cameraview_pts.cols()-1) = 
        Eigen::VectorXf::Ones(cameraview_pts.rows());

    // camera projection points
    Eigen::MatrixXf camera_projection_pts = (
        _camera_projection * cameraview_pts.transpose()
    ).transpose();

    // 2d pixel coordinates
    Eigen::MatrixXf pixel_coordinates_2d_pts = (
        camera_projection_pts.array().colwise() / 
        camera_projection_pts.col(2).array()
    );
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.block(0,0,pixel_coordinates_2d_pts.rows(), 2);
    pixel_coordinates_2d_pts *= 0.5;
    Eigen::VectorXf centre(2);
    centre << 0.5, 0.5;
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.rowwise() + centre.transpose();
    // split into x and y
    Eigen::VectorXf pixel_x = pixel_coordinates_2d_pts.col(0);
    Eigen::VectorXf pixel_y = pixel_coordinates_2d_pts.col(1);
    // Multiplication by image width
    pixel_x = pixel_x * 1280;
    pixel_y = (Eigen::VectorXf::Ones(pixel_y.rows())-pixel_y)*720;
    assert(pixel_x.rows() == pixel_y.rows());
    // Must flip image horizontal/vertically due to unity coordinate system (?)
    _frame.r->rowwise().reverseInPlace();
    _frame.g->rowwise().reverseInPlace();
    _frame.b->rowwise().reverseInPlace();
    _frame.r->colwise().reverseInPlace();
    _frame.g->colwise().reverseInPlace();
    _frame.b->colwise().reverseInPlace();
    // new xyzrgb points, with uncolored points filtered out
    std::vector<pcl::PointXYZRGB> newpts;
    auto n = std::min((int)cloud->points.size(), (int)pixel_x.rows());
    newpts.reserve(n);
    for(auto i = 0; i < n; i++) {
        int pi_x = (int)pixel_x(i);
        int pi_y = (int)pixel_y(i);
        // filter out invalid pixels; e.g. NaN is set to -MAX_INT
        if(pi_x < 1280 && pi_y < 720 && pi_x >= 0 && pi_y >= 0) {
            uint8_t r = (*_frame.r)(pi_y, pi_x);
            uint8_t g = (*_frame.g)(pi_y, pi_x);
            uint8_t b = (*_frame.b)(pi_y, pi_x);
            pcl::PointXYZRGB pt(r, g, b);
            auto& xyz = cloud->points.at(i);
            pt.x = xyz.x;
            pt.y = xyz.y;
            pt.z = xyz.z;
            newpts.push_back(std::move(pt));
        }
    }
    cloud->points.clear();
    cloud->points.insert(cloud->points.end(), newpts.begin(), newpts.end());
}

}
