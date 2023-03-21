#include "Share_Data.h"

Share_Data::Share_Data(std::string _config_file_path) {
    yaml_config_file_path = std::move(_config_file_path);

    // Reading yaml file
    cv::FileStorage fs;
    fs.open(yaml_config_file_path, cv::FileStorage::READ);
    fs["object_file_path"] >> object_folder_path;
    fs["name_of_object"] >> name_of_object;
    fs["method_of_IG"] >> method_of_IG;
    fs["octomap_resolution"] >> octomap_resolution;
    fs["num_of_max_iteration"] >> num_of_max_iteration;
    fs["move_wait"] >> move_wait;
    fs["show"] >> show;
    fs["p_unknown_upper_bound"] >> p_unknown_upper_bound;
    fs["p_unknown_lower_bound"] >> p_unknown_lower_bound;
    fs["num_of_views"] >> num_of_views;
    fs["cost_weight"] >> cost_weight;
    fs["skip_coefficient"] >> skip_coefficient;
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
    fs.release();

    // Building the different file paths
    object_file_path = object_folder_path + name_of_object;
    quality_file_path = object_file_path + ".qlt";
    view_space_file_path = object_file_path + ".txt";
    save_path = "../results/" + name_of_object + '_' + std::to_string(method_of_IG);

    // Reading the input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    input_cloud = temp_pcl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(object_file_path + ".pcd", *input_cloud) == -1) {
        cout << "3D PCD File not found. Trying to open OBJ file." << endl;
        if (pcl::io::loadOBJFile(object_file_path + ".obj", *input_cloud) == -1) {
            cout << "3D OBJ File not found. Trying to open PLY file." << endl;
            if (pcl::io::loadPLYFile(object_file_path + ".ply", *input_cloud) == -1) {
                cout << "3D PLY File not found." << endl;
                cout << "No 3D file where found there. Check your input name : " << object_file_path << endl;
            }
        }
    }

    // Creating the Octomap
    octo_model = new octomap::ColorOcTree(octomap_resolution);

    // Initializing the camera pose world to identity
    current_camera_pose_world = Eigen::Matrix4d::Identity(4, 4);

    // Initializing the working PointCloud
    valid_clouds = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    working_cloud = temp_cloud;

    // Creating the quality_weight unordered map
//    quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();

    cout << "3D object and YAML files have been read." << endl;
    cout << "Saving results in: " << save_path << endl;
    srand(clock());
}

Share_Data::~Share_Data() = default;

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

void Share_Data::updateQuality() {
    ifstream quality_file(quality_file_path);
    if (quality_file.is_open()) {
        for (auto &pt: working_cloud->points) {
            std::string line;
            if (getline(quality_file, line)) {
                octomap::OcTreeKey key;
                bool key_have = octo_model->coordToKeyChecked(pt.x, pt.y, pt.z, key);
                if (key_have) {
                    if ((*quality_weight).find(key) == (*quality_weight).end())
                        (*quality_weight)[key] = std::stod(line);
                    else
                        (*quality_weight)[key] = std::min((*quality_weight)[key],std::stod(line));
                }
            }
        }
        quality_file.close();
        cout << "Quality file has been read" << endl;
    } else {
        cout << "No Quality file has been found" << endl;
    }
}
