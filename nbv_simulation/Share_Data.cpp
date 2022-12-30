#include "Share_Data.h"

Share_Data::Share_Data(std::string _config_file_path) {
    process_cnt = -1;
    yaml_file_path = std::move(_config_file_path);
    cout << yaml_file_path << endl;
    // Reading yaml files
    cv::FileStorage fs;
    fs.open(yaml_file_path, cv::FileStorage::READ);
    fs["model_path"] >> pcd_file_path;
    fs["name_of_pcd"] >> name_of_pcd;
    fs["method_of_IG"] >> method_of_IG;
    fs["octomap_resolution"] >> octomap_resolution;
    fs["ground_truth_resolution"] >> ground_truth_resolution;
    fs["num_of_max_iteration"] >> num_of_max_iteration;
    fs["show"] >> show;
    fs["move_wait"] >> move_wait;
    fs["nbv_net_path"] >> nbv_net_path;
    fs["p_unknown_upper_bound"] >> p_unknown_upper_bound;
    fs["p_unknown_lower_bound"] >> p_unknown_lower_bound;
    fs["num_of_views"] >> num_of_views;
    fs["cost_weight"] >> cost_weight;
    fs["robot_cost_negative"] >> robot_cost_negative;
    fs["skip_coefficient"] >> skip_coefficient;
    fs["num_of_max_flow_node"] >> num_of_max_flow_node;
    fs["interesting_threshold"] >> interesting_threshold;
    fs["color_width"] >> color_intrinsics.width;
    fs["color_height"] >> color_intrinsics.height;
    fs["color_fx"] >> color_intrinsics.fx;
    fs["color_fy"] >> color_intrinsics.fy;
    fs["color_ppx"] >> color_intrinsics.ppx;
    fs["color_ppy"] >> color_intrinsics.ppy;
    fs["color_model"] >> color_intrinsics.model;
    fs["color_k1"] >> color_intrinsics.coeffs[0];
    fs["color_k2"] >> color_intrinsics.coeffs[1];
    fs["color_k3"] >> color_intrinsics.coeffs[2];
    fs["color_p1"] >> color_intrinsics.coeffs[3];
    fs["color_p2"] >> color_intrinsics.coeffs[4];
    fs["depth_scale"] >> depth_scale;
    fs["input_sfm_file"] >> sfm_file_path;
    fs.release();
    /* Populating the SfM_Data from AliceVision */
    sfm_data.getIntrinsics().emplace(0, std::make_shared<aliceVision::camera::Pinhole>(color_intrinsics.width,
                                                                                       color_intrinsics.height,
                                                                                       color_intrinsics.fx,
                                                                                       color_intrinsics.fy,
                                                                                       color_intrinsics.ppx -
                                                                                       color_intrinsics.width / 2,
                                                                                       color_intrinsics.ppy -
                                                                                       color_intrinsics.height / 2,
                                                                                       std::make_shared<aliceVision::camera::DistortionBrown>(
                                                                                               color_intrinsics.coeffs[0],
                                                                                               color_intrinsics.coeffs[1],
                                                                                               color_intrinsics.coeffs[2],
                                                                                               color_intrinsics.coeffs[3],
                                                                                               color_intrinsics.coeffs[4])));
    // Read the pcd file of the converted model
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_pcd = temp_pcd;
    cout << pcd_file_path + name_of_pcd + ".pcd" << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path + name_of_pcd + ".pcd", *cloud_pcd) == -1) {
        cout << "Can not read 3d model file. Trying to convert it." << endl;
        if (pcl::io::loadOBJFile(pcd_file_path + name_of_pcd + ".obj", *cloud_pcd) == -1) {
            cout << "No OBJ file found. Trying to open a ply file." << endl;
            if (pcl::io::loadPLYFile(pcd_file_path + name_of_pcd + ".ply", *cloud_pcd) == -1) {
            cout << "No PLY file found. Check" << endl;
            }
        }
    }
    octo_model = new octomap::ColorOcTree(octomap_resolution);
    ground_truth_model = new octomap::ColorOcTree(ground_truth_resolution);
    GT_sample = new octomap::ColorOcTree(octomap_resolution);
    if (num_of_max_flow_node == -1)
        num_of_max_flow_node = num_of_views;
    now_camera_pose_world = Eigen::Matrix4d::Identity(4, 4);
    over = false;
    pre_clock = (double) clock();
    valid_clouds = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_final = temp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_ground_truth = temp_gt;
    save_path = "../" + name_of_pcd + '_' + std::to_string(method_of_IG);
    if (method_of_IG == 0)
        save_path += '_' + std::to_string(cost_weight);
    cout << "PCD and Yaml files read." << endl;
    cout << "save_path is: " << save_path << endl;
    srand(clock());
}

Share_Data::~Share_Data() = default;

[[maybe_unused]] double Share_Data::out_clock() {
    auto now_clock = (double) clock();
    double elapsed_time = now_clock - pre_clock;
    pre_clock = now_clock;
    return elapsed_time;
}

/**
 * FIXME : Simplify this function
 */
void Share_Data::access_directory(const std::string &cd) {
    std::string temp;
    for (char i: cd)
        if (i == '/') {
            if (access(temp.c_str(), 0) != 0)
                fs::create_directory(temp);
            temp += i;
        } else
            temp += i;
    if (access(temp.c_str(), 0) != 0)
        fs::create_directory(temp);
}

[[maybe_unused]] void
Share_Data::save_posetrans_to_disk(Eigen::Matrix4d &T, const std::string &cd, const std::string &name,
                                   int frames_cnt) const {
    std::stringstream pose_stream, path_stream;
    std::string pose_file, path;
    path_stream << "../data"
                << "_" << process_cnt << cd;
    path_stream >> path;
    access_directory(path);
    pose_stream << "../data"
                << "_" << process_cnt << cd << "/" << name << "_" << frames_cnt << ".txt";
    pose_stream >> pose_file;
    std::ofstream fout(pose_file);
    fout << T;
}

[[maybe_unused]] void
Share_Data::save_octomap_log_to_disk(int voxels, double entropy, const std::string &cd, const std::string &name,
                                     int iterations) const {
    std::stringstream log_stream, path_stream;
    std::string log_file, path;
    path_stream << "../data"
                << "_" << process_cnt << cd;
    path_stream >> path;
    access_directory(path);
    log_stream << "../data"
               << "_" << process_cnt << cd << "/" << name << "_" << iterations << ".txt";
    log_stream >> log_file;
    std::ofstream fout(log_file);
    fout << voxels << " " << entropy << endl;
}

void Share_Data::save_cloud_to_disk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &cd,
                                    const std::string &name) const {
    std::stringstream cloud_stream, path_stream;
    std::string cloud_file, path;
    path_stream << save_path << cd;
    path_stream >> path;
    access_directory(path);
    cloud_stream << save_path << cd << "/" << name << ".pcd";
    cloud_stream >> cloud_file;
    pcl::io::savePCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
}

[[maybe_unused]] void
Share_Data::save_cloud_to_disk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &cd,
                               const std::string &name,
                               int frames_cnt) const {
    std::stringstream cloud_stream, path_stream;
    std::string cloud_file, path;
    path_stream << "../data"
                << "_" << process_cnt << cd;
    path_stream >> path;
    access_directory(path);
    cloud_stream << "../data"
                 << "_" << process_cnt << cd << "/" << name << "_" << frames_cnt << ".pcd";
    cloud_stream >> cloud_file;
    pcl::io::savePCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
}

[[maybe_unused]] void Share_Data::save_octomap_to_disk(octomap::ColorOcTree *_octo_model, const std::string &cd,
                                                       const std::string &name) const {
    std::stringstream octomap_stream, path_stream;
    std::string octomap_file, path;
    path_stream << "../data"
                << "_" << process_cnt << cd;
    path_stream >> path;
    access_directory(path);
    octomap_stream << "../data"
                   << "_" << process_cnt << cd << "/" << name << ".ot";
    octomap_stream >> octomap_file;
    _octo_model->write(octomap_file);
}