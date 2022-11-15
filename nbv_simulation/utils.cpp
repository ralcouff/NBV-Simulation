#include <cmath>
#include <cfloat>
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <octomap/ColorOcTree.h>

using namespace std;

/** @brief Distortion model: defines how pixel coordinates should be mapped to sensor coordinates. */
typedef enum rs2_distortion
{
    RS2_DISTORTION_NONE,                   /**< Rectilinear images. No distortion compensation required. */
    RS2_DISTORTION_MODIFIED_BROWN_CONRADY, /**< Equivalent to Brown-Conrady distortion, except that tangential
                                              distortion is applied to radially distorted points */
    RS2_DISTORTION_INVERSE_BROWN_CONRADY,  /**< Equivalent to Brown-Conrady distortion, except undistorts image instead
                                              of distorting it */
    RS2_DISTORTION_FTHETA,                 /**< F-Theta fish-eye distortion model */
    RS2_DISTORTION_BROWN_CONRADY,          /**< Unmodified Brown-Conrady distortion model */
    RS2_DISTORTION_KANNALA_BRANDT4,        /**< Four parameter Kannala Brandt distortion model */
    RS2_DISTORTION_COUNT                   /**< Number of enumeration values. Not a valid input: intended to be used
                                              in for-loops. */
} rs2_distortion;

/** @brief Video stream intrinsics. */
typedef struct rs2_intrinsics
{
    int width;  /**< Width of the image in pixels */
    int height; /**< Height of the image in pixels */
    float ppx;  /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float ppy;  /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float fx;   /**< Focal length of the image plane, as a multiple of pixel width */
    float fy;   /**< Focal length of the image plane, as a multiple of pixel height */
    rs2_distortion model; /**< Distortion model of the image */
    float coeffs[5];      /**< Distortion coefficients */
} rs2_intrinsics;

/**
 * Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward
 * distortion coefficients produced by the same camera
 * @param pixel TODO
 * @param intrin TODO
 * @param point TODO
 */
static void rs2_project_point_to_pixel(float pixel[2], const struct rs2_intrinsics* intrin, const float point[3])
{
    float x = point[0] / point[2], y = point[1] / point[2];

    if((intrin->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) ||
       (intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY))
    {
        float r2 = x * x + y * y;
        float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
        x *= f;
        y *= f;
        float dx = x + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
        float dy = y + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
        x = dx;
        y = dy;
    }
    if(intrin->model == RS2_DISTORTION_FTHETA)
    {
        float r = sqrtf(x * x + y * y);
        if(r < FLT_EPSILON)
        {
            r = FLT_EPSILON;
        }
        float rd = (float)(1.0f / intrin->coeffs[0] * atan(2 * r * tan(intrin->coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
    }
    if(intrin->model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        float r = sqrtf(x * x + y * y);
        if(r < FLT_EPSILON)
        {
            r = FLT_EPSILON;
        }
        float theta = atan(r);
        float theta2 = theta * theta;
        float series =
                1 + theta2 * (intrin->coeffs[0] +
                              theta2 * (intrin->coeffs[1] + theta2 * (intrin->coeffs[2] + theta2 * intrin->coeffs[3])));
        float rd = theta * series;
        x *= rd / r;
        y *= rd / r;
    }

    pixel[0] = x * intrin->fx + intrin->ppx;
    pixel[1] = y * intrin->fy + intrin->ppy;
}

/**
 * Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the
 * corresponding point in 3D space relative to the same camera
 * @param point The output 3D point containing the estimated position of the correspondent pixel
 * @param intrin Intrinsic parameters of the camera
 * @param pixel The coordinates of the pixel in image
 * @param depth The depth of the point in the image
 */
static void rs2_deproject_pixel_to_point(float point[3],
                                         const struct rs2_intrinsics* intrin,
                                         const float pixel[2],
                                         float depth)
{
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    // assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to a brown conrady model

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2 = x * x + y * y;
        float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
        float ux = x * f + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
        float uy = y * f + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
        x = ux;
        y = uy;
    }
    if(intrin->model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        float rd = sqrtf(x * x + y * y);
        if(rd < FLT_EPSILON)
        {
            rd = FLT_EPSILON;
        }

        float theta = rd;
        float theta2 = rd * rd;
        for(int i = 0; i < 4; i++)
        {
            float f = theta * (1 + theta2 * (intrin->coeffs[0] +
                                             theta2 * (intrin->coeffs[1] +
                                                       theta2 * (intrin->coeffs[2] + theta2 * intrin->coeffs[3])))) -
                      rd;
            if(abs(f) < FLT_EPSILON)
            {
                break;
            }
            float df = 1 + theta2 * (3 * intrin->coeffs[0] +
                                     theta2 * (5 * intrin->coeffs[1] +
                                               theta2 * (7 * intrin->coeffs[2] + 9 * theta2 * intrin->coeffs[3])));
            theta -= f / df;
            theta2 = theta * theta;
        }
        float r = tan(theta);
        x *= r / rd;
        y *= r / rd;
    }
    if(intrin->model == RS2_DISTORTION_FTHETA)
    {
        float rd = sqrtf(x * x + y * y);
        if(rd < FLT_EPSILON)
        {
            rd = FLT_EPSILON;
        }
        auto r = (float)(tan(intrin->coeffs[0] * rd) / atan(2 * tan(intrin->coeffs[0] / 2.0f)));
        x *= r / rd;
        y *= r / rd;
    }

    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

/**
 * Compute the square of a number
 * @param x The number to compute the square
 * @return The square of the number x
 */
inline double pow2(double x) { return x * x; }

/**
 *
 * @param hull The hull containing the object
 * @param color_intrinsics TODO
 * @return
 */
inline vector<int> get_xmax_xmin_ymax_ymin_in_hull(vector<cv::Point2f>& hull, rs2_intrinsics& color_intrinsics)
{
    float x_max = 0, x_min = color_intrinsics.width - 1, y_max = 0, y_min = color_intrinsics.height - 1;
    for(int i = 0; i < hull.size(); i++)
    {
        x_max = std::max(x_max, hull[i].x);
        x_min = std::min(x_min, hull[i].x);
        y_max = std::max(y_max, hull[i].y);
        y_min = std::min(y_min, hull[i].y);
    }
    std::vector<int> boundary;
    boundary.push_back((int)floor(x_max));
    boundary.push_back((int)floor(x_min));
    boundary.push_back((int)floor(y_max));
    boundary.push_back((int)floor(y_min));
    return boundary;
}

/**
 * Pixel in convex hull test
 * @param hull The convex hull of the object
 * @param pixel The pixel to check
 * @return TRUE if the pixel is in the convex hull
 */
inline bool is_pixel_in_convex(vector<cv::Point2f>& hull, cv::Point2f& pixel)
{
    double hull_value = pointPolygonTest(hull, pixel, false);
    return hull_value >= 0;
}

/**
 * Compute the projection of the 3D convex hull onto the image plane
 * @param convex_3d The 3D convex hull of the object
 * @param now_camera_pose_world The current camera pose
 * @param color_intrinsics TODO
 * @param pixel_interval TODO
 * @param max_range TODO
 * @param octomap_resolution The resolution of the current octomap
 * @return Pixel coordinates of the projection of the 3D convex hull
 */
inline vector<cv::Point2f> get_convex_on_image(vector<Eigen::Vector4d>& convex_3d,
                                               Eigen::Matrix4d& now_camera_pose_world,
                                               rs2_intrinsics& color_intrinsics,
                                               int& pixel_interval,
                                               double& max_range,
                                               double& octomap_resolution)
{
    // Projection of the cube vertices to the image coordinate system
    double now_range = 0;
    vector<cv::Point2f> contours;
    for(int i = 0; i < convex_3d.size(); i++)
    {
        Eigen::Vector4d vertex = now_camera_pose_world.inverse() * convex_3d[i];
        float point[3] = {static_cast<float>(vertex(0)), static_cast<float>(vertex(1)), static_cast<float>(vertex(2))};
        float pixel[2];
        rs2_project_point_to_pixel(pixel, &color_intrinsics, point);
        contours.emplace_back(pixel[0], pixel[1]);
        // cout << pixel[0] << " " << pixel[1] << endl;
        // Calculate the distance of the furthest point from the viewpoint
        Eigen::Vector4d view_pos(
                now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3), 1);
        now_range = max(now_range, (view_pos - convex_3d[i]).norm());
    }
    max_range = min(max_range, now_range);
    // Calculating convex packages
    vector<cv::Point2f> hull;
    convexHull(contours, hull, false, true);
    if(!cv::isContourConvex(hull))
    {
        cout << "No convex Hull found. Check BBX." << endl;
        return contours;
    }
    // Calculate the distance between the furthest two points in space, calculate the distance between the furthest two
    // points of a pixel and get the pixel offset according to the map resolution
    double pixel_dis = 0;
    double space_dis = 0;
    for(int i = 0; i < hull.size(); i++)
        for(int j = 0; j < hull.size(); j++)
            if(i != j)
            {
                Eigen::Vector2d pixel_start(hull[i].x, hull[i].y);
                Eigen::Vector2d pixel_end(hull[j].x, hull[j].y);
                pixel_dis = max(pixel_dis, (pixel_start - pixel_end).norm());
                space_dis = max(space_dis, (convex_3d[i] - convex_3d[j]).norm());
            }
    pixel_interval = (int)(pixel_dis / space_dis * octomap_resolution);
    return hull;
}

/**
 * Project the pixel of an image to the max_depth desired and creates the endpoint at this range.
 * @param x The coordinate x of the pixel
 * @param y The coordinate y of the pixel
 * @param color_intrinsics The intrinsic parameters of the camera
 * @param now_camera_pose_world The current camera position
 * @param max_range The maximum depth desired for the projection
 * @return The position of the point at the maximum range of the viewpoint. In the current viewpoint referential.
 */
inline octomap::point3d project_pixel_to_ray_end(
        int x, int y, rs2_intrinsics& color_intrinsics, Eigen::Matrix4d& now_camera_pose_world, float max_range)
{
    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
    float point[3];
    rs2_deproject_pixel_to_point(point, &color_intrinsics, pixel, max_range);
    Eigen::Vector4d point_world(point[0], point[1], point[2], 1);
    point_world = now_camera_pose_world * point_world;
    return {static_cast<float>(point_world(0)), static_cast<float>(point_world(1)), static_cast<float>(point_world(2))};
}

/**
 * Compute the distance described in eq (2)
 * @param distance The Euclidean distance
 * @param alpha variable related to the octomap resolution
 * @return the distance of eq (2)
 */
inline double distance_function(double distance, double alpha) { return exp(-pow2(alpha) * distance); }

/**
 * Generate a relatively random 0-1 random number and map it to the interval [from,to]
 * @param from Beginning of the interval
 * @param to End of the interval
 * @return A random number between from and to
 */
inline double get_random_coordinate(double from, double to)
{
    double len = to - from;
    long long x = (long long)rand() * ((long long)RAND_MAX + 1) + (long long)rand();
    long long field = (long long)RAND_MAX * (long long)RAND_MAX + 2 * (long long)RAND_MAX;
    return (double)x / (double)field * len + from;
}