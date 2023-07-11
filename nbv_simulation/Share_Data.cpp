#include "Share_Data.h"

Share_Data::Share_Data(const std::string& _config_file_path, const std::string& _model_path, const std::string& _model_qlt_path,
                       short _method, int _n_iter, const std::string& _save_folder, const std::string& _string_test_time) {

    process_cnt = -1;
    // Reading yaml files
    yamlConfigFilePath = _config_file_path;
    cv::FileStorage fs;
    fs.open(yamlConfigFilePath, cv::FileStorage::READ);
    fs["octomap_resolution"] >> octomap_resolution;
    fs["ground_truth_resolution"] >> ground_truth_resolution;
    fs["num_of_max_iteration"] >> num_of_max_iteration;
    fs["show"] >> show;
    fs["move_wait"] >> move_wait;
    fs["nbvNetPath"] >> nbvNetPath;
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
    fs.release();

    // Test parameters
    alt_method_of_IG = get_method(_method);
    method_of_IG = get_method(0);
    string_test_time = _string_test_time;
    reconstructionIterations = _n_iter;

    objectFilePath = _model_path;
    objectFolderPath = objectFilePath.substr(0, objectFilePath.find_last_of('/')) + '/';
    std::string fullNameOfObject = objectFilePath.substr(objectFilePath.find_last_of('/') + 1, objectFilePath.length());
    nameOfObject = fullNameOfObject.substr(0, fullNameOfObject.find_last_of('.'));
    qualityFilePath = _model_qlt_path;
    viewSpaceFilePath = objectFilePath + ".txt";
    savePath = _save_folder;
    access_directory(savePath);

    test_base_filename = savePath + "/results.csv";
    std::ofstream result(test_base_filename, std::ios_base::app);
    result
            << "alt_method_of_IG,method_of_IG,n_model,n_size,reconstructionIterations,iterations,id_best_view,x,y,z,final_utility,completeness"
            << endl;
    result.close();

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
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(objectFilePath + ".pcd", *cloud_pcd) == -1) {
        cout << "3D PCD File not found. Trying to open OBJ file." << endl;
        if (pcl::io::loadOBJFile<pcl::PointXYZ>(objectFilePath + ".obj", *cloud_pcd) == -1) {
            cout << "3D OBJ File not found. Trying to open PLY file." << endl;
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(objectFilePath + ".ply", *cloud_pcd) == -1) {
                cout << "3D PLY File not found." << endl;
                cout << "No 3D file where found there. Check your input name : " << objectFilePath << endl;
            }
        }
    }
    /* Add the initial PC to the sfm_data pipeline. */
    float i = 0;
    for (auto &pt: cloud_pcd->points) {
        sfm_data.getLandmarks().emplace(i, aliceVision::sfmData::Landmark(Eigen::Matrix<double, 3, 1>(pt.x, pt.y, pt.z),
                                                                          aliceVision::feature::EImageDescriberType::SIFT));
        i++;
    }
    aliceVision::sfmDataIO::saveJSON(sfm_data, savePath + "/scene.sfm", aliceVision::sfmDataIO::ESfMData::ALL);
    cout << "The sfmData file has been saved" << endl;

    // Generating the octomaps
    octo_model = new octomap::ColorOcTree(octomap_resolution);
    ground_truth_model = new octomap::ColorOcTree(ground_truth_resolution);
    GT_sample = new octomap::ColorOcTree(octomap_resolution);

    // Generating the quality weight map for octo_model
    quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();

    // Initializing the camera pose world to identity
    now_camera_pose_world = Eigen::Matrix4d::Identity(4, 4);

    // Initializing parameters
    if (num_of_max_flow_node == -1)
        num_of_max_flow_node = num_of_views;
    over = false;
    pre_clock = (double) clock();
    valid_clouds = 0;

    // Initializing the Point Clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_final = temp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_ground_truth = temp_gt;

    cout << "3D object and YAML files have been read." << endl;
    cout << "The input cloud has: " << cloud_pcd->points.size() << " points." << endl;
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
    path_stream << savePath << cd;
    path_stream >> path;
    access_directory(path);
    cloud_stream << savePath << cd << "/" << name << ".pcd";
    cloud_stream >> cloud_file;
    pcl::io::savePCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
}

short get_method(int _n_test) {
    short method;
    switch (_n_test){
        case 0:
            method = OursIG;
            break;
        case 10:
            method = Test_local;
            break;
        case 11:
            method = Test_flow;
            break;
        case 101:
            method = Test_qlt;
            break;
        case 102:
            method = Test_two;
            break;
        default:
            cout << "This method does not exists, choosing the default one (0).";
            method = OursIG;
    }
    return method;
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