#include "View.h"

#include <utility>

View::View(Eigen::Vector3d _init_pos) {
    init_pos = std::move(_init_pos);
    pose = Eigen::Matrix4d::Identity(4, 4);
    information_gain = 0;
    voxel_num = 0;
    robot_cost = 0;
    dis_to_object = 0;
    final_utility = 0;
    robot_moved = false;
    path_num = 0;
    vis = 0;
    can_move = true;
}

View::View(const View &other) {
    space_id = other.space_id;
    id = other.id;
    init_pos = other.init_pos;
    pose = other.pose;
    information_gain = (double) other.information_gain;
    voxel_num = (int) other.voxel_num;
    robot_cost = other.robot_cost;
    dis_to_object = other.dis_to_object;
    final_utility = other.final_utility;
    robot_moved = (bool) other.robot_moved;
    path_num = other.path_num;
    vis = other.vis;
    can_move = other.can_move;
    in_coverage = other.in_coverage;
}

View &View::operator=(const View &other) {
    init_pos = other.init_pos;
    space_id = other.space_id;
    id = other.id;
    pose = other.pose;
    information_gain = (double) other.information_gain;
    voxel_num = (int) other.voxel_num;
    robot_cost = other.robot_cost;
    dis_to_object = other.dis_to_object;
    final_utility = other.final_utility;
    robot_moved = (bool) other.robot_moved;
    path_num = other.path_num;
    vis = other.vis;
    can_move = other.can_move;
    in_coverage = other.in_coverage;
    return *this;
}

double View::global_function(int x) {
    return exp(-1.0 * x);
}

double View::get_global_information() {
    double information = 0;
    for (int i = 0; i <= id && i < 64; i++)
        information += in_coverage[i] * global_function(id - i);
    return information;
}

void View::get_next_camera_pos(const Eigen::Matrix4d& now_camera_pose_world, Eigen::Vector3d object_center_world) {
    /* Normalized multiplication */
    /* Re-projecting point in the camera world referential */
    Eigen::Vector4d object_center_now_camera;
    object_center_now_camera =
            now_camera_pose_world.inverse() *
            Eigen::Vector4d(object_center_world(0), object_center_world(1), object_center_world(2), 1);
    /* Re-projecting the coordinates of the view in the new referential. */
    Eigen::Vector4d view_now_camera;
    view_now_camera = now_camera_pose_world.inverse() * Eigen::Vector4d(init_pos(0), init_pos(1), init_pos(2), 1);
    // Define the pointing object as Z+ and the ray from the previous camera position to the current X+,
    //  calculate the transformation matrix between the two camera coordinate systems, object and view are
    //  the coordinates under the previous camera coordinate system
    /* Z, the vector from object to view. */
    Eigen::Vector3d object(object_center_now_camera(0), object_center_now_camera(1), object_center_now_camera(2));
    Eigen::Vector3d view(view_now_camera(0), view_now_camera(1), view_now_camera(2));
    Eigen::Vector3d Z;
    Z = object - view;
    Z = Z.normalized();
    // Be careful with the left and right hand ties, don't get them reversed
    Eigen::Vector3d X;
    X = Z.cross(view);
    X = X.normalized();
    Eigen::Vector3d Y;
    Y = Z.cross(X);
    Y = Y.normalized();
    Eigen::Matrix4d T(4, 4);
    T(0, 0) = 1;
    T(0, 1) = 0;
    T(0, 2) = 0;
    T(0, 3) = -view(0);
    T(1, 0) = 0;
    T(1, 1) = 1;
    T(1, 2) = 0;
    T(1, 3) = -view(1);
    T(2, 0) = 0;
    T(2, 1) = 0;
    T(2, 2) = 1;
    T(2, 3) = -view(2);
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;
    Eigen::Matrix4d R(4, 4);
    R(0, 0) = X(0);
    R(0, 1) = Y(0);
    R(0, 2) = Z(0);
    R(0, 3) = 0;
    R(1, 0) = X(1);
    R(1, 1) = Y(1);
    R(1, 2) = Z(1);
    R(1, 3) = 0;
    R(2, 0) = X(2);
    R(2, 1) = Y(2);
    R(2, 2) = Z(2);
    R(2, 3) = 0;
    R(3, 0) = 0;
    R(3, 1) = 0;
    R(3, 2) = 0;
    R(3, 3) = 1;
    // Rotate around the z-axis so that the angle between the x-axis and the y-axis is minimised in relation to the
    // previous rotation
    Eigen::Matrix3d Rz_min(Eigen::Matrix3d::Identity(3, 3));
    Eigen::Vector4d x(1, 0, 0, 1);
    Eigen::Vector4d y(0, 1, 0, 1);
    Eigen::Vector4d x_ray(1, 0, 0, 1);
    Eigen::Vector4d y_ray(0, 1, 0, 1);
    x_ray = R.inverse() * T * x_ray;
    y_ray = R.inverse() * T * y_ray;
    double min_y = acos(y(1) * y_ray(1));
    double min_x = acos(x(0) * x_ray(0));
    for (double i = 5; i < 360; i += 5) {
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(i * acos(-1.0) / 180.0, Eigen::Vector3d::UnitZ());
        Eigen::Matrix4d Rz(Eigen::Matrix4d::Identity(4, 4));
        Rz(0, 0) = rotation(0, 0);
        Rz(0, 1) = rotation(0, 1);
        Rz(0, 2) = rotation(0, 2);
        Rz(0, 3) = 0;
        Rz(1, 0) = rotation(1, 0);
        Rz(1, 1) = rotation(1, 1);
        Rz(1, 2) = rotation(1, 2);
        Rz(1, 3) = 0;
        Rz(2, 0) = rotation(2, 0);
        Rz(2, 1) = rotation(2, 1);
        Rz(2, 2) = rotation(2, 2);
        Rz(2, 3) = 0;
        Rz(3, 0) = 0;
        Rz(3, 1) = 0;
        Rz(3, 2) = 0;
        Rz(3, 3) = 1;
        Eigen::Vector4d x_ray(1, 0, 0, 1);
        Eigen::Vector4d y_ray(0, 1, 0, 1);
        x_ray = (R * Rz).inverse() * T * x_ray;
        y_ray = (R * Rz).inverse() * T * y_ray;
        double cos_y = acos(y(1) * y_ray(1));
        double cos_x = acos(x(0) * x_ray(0));
        if (cos_y < min_y) {
            Rz_min = rotation.eval();
            min_y = cos_y;
            min_x = cos_x;
        } else if (fabs(cos_y - min_y) < 1e-6 && cos_x < min_x) {
            Rz_min = rotation.eval();
            min_y = cos_y;
            min_x = cos_x;
        }
    }
    Eigen::Vector3d eulerAngle = Rz_min.eulerAngles(0, 1, 2);
    // cout << "Rotate getted with angel " << eulerAngle(0)<<","<< eulerAngle(1) << "," << eulerAngle(2)<<" and l2
    // "<< min_l2 << endl;
    Eigen::Matrix4d Rz(Eigen::Matrix4d::Identity(4, 4));
    Rz(0, 0) = Rz_min(0, 0);
    Rz(0, 1) = Rz_min(0, 1);
    Rz(0, 2) = Rz_min(0, 2);
    Rz(0, 3) = 0;
    Rz(1, 0) = Rz_min(1, 0);
    Rz(1, 1) = Rz_min(1, 1);
    Rz(1, 2) = Rz_min(1, 2);
    Rz(1, 3) = 0;
    Rz(2, 0) = Rz_min(2, 0);
    Rz(2, 1) = Rz_min(2, 1);
    Rz(2, 2) = Rz_min(2, 2);
    Rz(2, 3) = 0;
    Rz(3, 0) = 0;
    Rz(3, 1) = 0;
    Rz(3, 2) = 0;
    Rz(3, 3) = 1;
    pose = ((R * Rz).inverse() * T).eval();
    // pose = (R.inverse() * T).eval();
}

[[maybe_unused]] void View::add_view_coordinates_to_cloud(Eigen::Matrix4d now_camera_pose_world,
                                         pcl::visualization::PCLVisualizer::Ptr viewer, int space_id) {
    // view.get_next_camera_pos(view_space->now_camera_pose_world, view_space->object_center_world);
    Eigen::Vector4d X(0.05, 0, 0, 1);
    Eigen::Vector4d Y(0, 0.05, 0, 1);
    Eigen::Vector4d Z(0, 0, 0.05, 1);
    Eigen::Vector4d weight(final_utility, final_utility, final_utility, 1);
    X = now_camera_pose_world * X;
    Y = now_camera_pose_world * Y;
    Z = now_camera_pose_world * Z;
    weight = now_camera_pose_world * weight;
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(init_pos(0), init_pos(1), init_pos(2)),
                                   pcl::PointXYZ(X(0), X(1), X(2)),
                                   255,
                                   0,
                                   0,
                                   "X" + std::to_string(space_id) + "-" + std::to_string(id));
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(init_pos(0), init_pos(1), init_pos(2)),
                                   pcl::PointXYZ(Y(0), Y(1), Y(2)),
                                   0,
                                   255,
                                   0,
                                   "Y" + std::to_string(space_id) + "-" + std::to_string(id));
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(init_pos(0), init_pos(1), init_pos(2)),
                                   pcl::PointXYZ(Z(0), Z(1), Z(2)),
                                   0,
                                   0,
                                   255,
                                   "Z" + std::to_string(space_id) + "-" + std::to_string(id));
    // viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(init_pos(0), init_pos(1), init_pos(2)), pcl::PointXYZ(weight(0),
    // weight(1), weight(2)), 0, 255, 255, "weight" + to_string(space_id) + "-" + to_string(id));
}

bool view_id_compare(View &a, View &b) { return a.id < b.id; }

bool view_utility_compare(View &a, View &b) {
    if (a.final_utility == b.final_utility)
        return a.robot_cost < b.robot_cost;
    return a.final_utility > b.final_utility;
}