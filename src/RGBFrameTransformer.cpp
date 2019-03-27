#include "RGBFrameTransformer.h"

namespace holovision {

RGBFrameTransformer::RGBFrameTransformer(
    FrameMessage&& frame
): _frame(std::move(frame)), _camera_projection(frame.intrinsics){}

void RGBFrameTransformer::compute_world_to_cameraview(){
    _world_to_camera_view = _frame.extrinsics.transpose() *
        _frame.frame_to_origin.transpose().inverse();
}

void RGBFrameTransformer::get_RGBD_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::shared_ptr<Eigen::MatrixXf> world_pts){
    compute_world_to_cameraview();

    // Note this currently changes the world_pts matrix.
    // world_pts.conservativeResize(world_pts.rows(), world_pts.cols()+1);
    // world_pts.col(world_pts.cols()-1) = Eigen::VectorXf::Ones(world_pts.rows());
    *world_pts = world_pts->rowwise().homogeneous();

    Eigen::MatrixXf camera_view_pts = (_world_to_camera_view * world_pts->transpose()).transpose();
    camera_view_pts.col(camera_view_pts.cols()-1) = Eigen::VectorXf::Ones(camera_view_pts.rows());

    Eigen::MatrixXf camera_projection_pts = (_camera_projection*camera_view_pts.transpose()).transpose();

    Eigen::MatrixXf pixel_coordinates_2d_pts = (camera_projection_pts.array().colwise() / camera_projection_pts.col(2).array());
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.block(0,0,pixel_coordinates_2d_pts.rows(), 2);

    pixel_coordinates_2d_pts *= 0.5;
    Eigen::VectorXf centre(2);
    centre << 0.5, 0.5;
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.rowwise() + centre.transpose();

    Eigen::VectorXf pixel_x = pixel_coordinates_2d_pts.col(0);
    Eigen::VectorXf pixel_y = pixel_coordinates_2d_pts.col(1);

    // Multiplication by image width
    pixel_x = pixel_x * 1280;
    pixel_y = (Eigen::VectorXf::Ones(pixel_y.rows())-pixel_y)*720;

    // This could be a source of noise
    Eigen::VectorXi pixel_xi = pixel_x.cast<int>();
    Eigen::VectorXi pixel_yi = pixel_y.cast<int>();
    
    Eigen::MatrixXi pixels(pixel_xi.rows(), 2);
    pixels << pixel_xi, pixel_yi;

    // Eigen::MatrixXi colors(world_pts.rows(), 3);
    for(auto i = 0; i<pixels.rows(); i++){
        int pi_x = pixels(i, 0);
        int pi_y = pixels(i, 1);
        if(pi_x < 1280 and pi_y < 720 and pi_x >= 0 and pi_y >= 0){
            int r_d = (*_frame.r)(pi_y, pi_x); //Confirm this height, width access
            int g_d = (*_frame.g)(pi_y, pi_x);
            int b_d = (*_frame.b)(pi_y, pi_x);
            // colors << r_d, g_d, b_d;
            cloud->points[i].r = r_d;
            cloud->points[i].g = g_d;
            cloud->points[i].b = b_d;
        }
        // else{
        //     colors << 0, 0, 0;  //Black is ignored in image segmentation for skin color.
        // }
    }
    return;
}

}
