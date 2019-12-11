# Holovision: Framework for protoyping AR Apps using Microsoft Hololens

## A Framework for AR-Assisted Surgery

Early-stage breast cancer surgery involves taking an MRI
scan in prone (face-down) position, then operating on the patient who lies on the table face-up. The change in
posture can distort or displace the tumor due to the
malleability of breast tissue, making surgery difficult. 25%
of patients must undergo repeated treatments or
mastectomy due to residual tumors. The HoloVision can
render a real-time hologram of the tumor onto the patient, (based on the MRI scan of the tumor)
and can assist the surgeon in preparing. 

## Pipeline:
- Microsoft Hololens: The HoloLens is equipped with an infrared depth sensor and a color camera. The app sends color (RGB) and depth (D) bitmaps to the server using a TCP socket.
- Point Cloud Registration: Pixels from RGB and D bitmaps must be mapped to world coordinates from the 2D image and local 3D coordinates by applying transformation matrices provided by the camera, HoloLens and Unity. This results in a RGB-D point cloud.
- Color Segmentation: RGB-D points are filtered using the multivariate Gaussian distribution mode. Only points that belong to the breast (determined by the color of the pixel) are aggregated for surface registration.
- Surface Registration and Initial Alignment:
  - Uses Fast Point Feature Histograms (FPFH) for 3D Registration for Initial Alignment (as implemented in the PCL library)
  - We fit the point cloud from the MRI scan to the real-world points using an Iterative Closest Point (ICP) matching algorithm. The optimization equation below minimizes error, and the resulting Singular Value Decomposition matrices are used to transform the tumor point cloud.
- Triangulation/Rendering: The tumor point cloud is downsampled using a voxelgrid filter, then triangulated using PCL. The mesh is sent back to the HoloLens through another TCP socket to render using Unity.

### Other repositories:
- Please look at https://github.com/pranaabdhawan/HoloVisionResearch for simple Python scripts for Color Segmentation and data processing (coupling RGB-D data and mapping from 3d->2d->3d) of Hololens Sensor Data
- Please look at https://github.com/UWHoloVision/Cloud for the Hololens App communicating with this backend

More details to follow!
