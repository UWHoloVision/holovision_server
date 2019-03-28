#include "Registration.h"

namespace holovision {

const std::string Registration::SOURCE_CLOUD_PATH = 
  "../src/source_point_cloud_breast.csv";

const std::string Registration::TUMOR_1_CLOUD_PATH = 
  "../src/tumor_1.csv";

const std::string Registration::TUMOR_2_CLOUD_PATH = 
  "../src/tumor_2.csv";

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};


class CSVRow
{
    public:
        std::string const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str, line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream, cell, ','))
            {
                m_data.push_back(cell);
            }
            // This checks for a trailing comma with no data after it.
            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                m_data.push_back("");
            }
        }
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

void readfile(std::string filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    std::ifstream       file(filepath);
    CSVRow              row;
    int num_rows = 0;
    while(file >> row)
    {   
        double x = atof(row[0].c_str());
        double y = atof(row[1].c_str());
        double z = atof(row[2].c_str());
        if (isnan(x) || isinf(x))
        continue;
        if (isnan(y) || isinf(y))
        continue;
        if (isnan(z) || isinf(z))
        continue;
        pcl::PointXYZ pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(std::move(pt));
        num_rows++;
    }
    // Set the desired properties for saving (unorganized point cloud)
    cloud->height = 1;
    cloud->width = num_rows;
}

Registration::Registration(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    readfile(SOURCE_CLOUD_PATH , cloud_in);
    src = cloud_in;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_in_1(new pcl::PointCloud<pcl::PointXYZ>);
    readfile(TUMOR_1_CLOUD_PATH , tumor_in_1);
    tumor_1 = tumor_in_1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_in_2(new pcl::PointCloud<pcl::PointXYZ>);
    readfile(TUMOR_2_CLOUD_PATH , tumor_in_2);
    tumor_2 = tumor_in_2;
}

void Registration::register_points(pcl::PointCloud<pcl::PointXYZ>::Ptr tgt){

    // Apply voxelization
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_tgt (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (0.005, 0.005, 0.005);
    grid.setInputCloud (src);
    grid.filter (*m_src);

    // pcl::copyPointCloud(*tgt, *m_tgt);
    std::cout << "Finalized 0" << std::endl;
    grid.setInputCloud (tgt);
    grid.filter (*m_tgt);

    //remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*m_src, *m_src, indices);
    pcl::removeNaNFromPointCloud(*m_tgt, *m_tgt, indices);

    std::cout << "Finalized f" << std::endl;
    // Assign to the target FeatureCloud
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(m_tgt);
    std::cout << "Finalized f1" << std::endl;
    FeatureCloud source_cloud;
    source_cloud.setInputCloud(m_src);

    std::cout << "Finalized 1" << std::endl;
    // Add inital alignment:
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    sac_ia_.setMinSampleDistance (0.05f);
    sac_ia_.setMaxCorrespondenceDistance (0.01f*0.01f);
    sac_ia_.setMaximumIterations (500);

    std::cout << "Finalized" << std::endl;
    sac_ia_.setInputTarget(m_tgt);
    sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());

    std::cout << "Finalized 2" << std::endl;
    //Align
    sac_ia_.setInputCloud (source_cloud.getPointCloud ());
    sac_ia_.setSourceFeatures (source_cloud.getLocalFeatures ());
    std::cout << "Finalized 3" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align (registration_output);


    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(m_src);
    // icp.setInputTarget(m_tgt);
    
    //pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.setMaximumIterations(500);
    // icp.setMaxCorrespondenceDistance (0.05); 
    // std::cout <<"Beginning registration" << std::endl;
    // icp.align(Final);

    std::cout << "has converged:" << sac_ia_.hasConverged() << " score: " <<
    sac_ia_.getFitnessScore() << std::endl;

    _transformation = sac_ia_.getFinalTransformation(); // source -> target
}

void Registration::apply_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){
    pcl::transformPointCloud(*input, *output, _transformation);
}

void Registration::apply_transform_on_source(pcl::PointCloud<pcl::PointXYZ>::Ptr output){
    pcl::transformPointCloud(*src, *output, _transformation);
}

} //namespace holovision

