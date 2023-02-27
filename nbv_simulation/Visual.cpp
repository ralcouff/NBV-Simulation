#include "Visual.h"

void Visual::voir() {
    pcl::visualization::PCLVisualizer::Ptr visualizer = std::make_shared<pcl::visualization::PCLVisualizer>(
            "Hello there");
    visualizer->setBackgroundColor(255, 255, 255);
    visualizer->addCoordinateSystem(0.1);
    visualizer->initCameraParameters();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr view_space(new pcl::PointCloud<pcl::PointXYZRGB>);
    view_space->is_dense = false;

    int method, model, size, iteration, index, n_cam;
    double x, y, z, qlt;

    std::string fname = "/home/alcoufr/dev/NBV-Simulation/results/2023-02-24-15-31/results.csv";
    std::fstream fin(fname, ios::in);
    std::string line, word;
    std::vector<std::string> row;
    std::vector<std::vector<std::string>> content;

    std::string current_input_filename = "/home/alcoufr/dev/NBV-Simulation/results/2023-02-24-15-31/0_armadillo_qlt/0_armadillo_quality_colored_12_0/armadillo_quality_colored_rescaled";
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud = temp_pcd;
    pcl::io::loadOBJFile(current_input_filename + ".obj", *input_cloud);

    if (fin.is_open()) {
        while (getline(fin, line)) {
            row.clear();
            std::stringstream str(line);
            while (getline(str, word, ',')) {
                row.push_back(word);
            }
            content.push_back(row);
        }
    }
    while (true) {
        visualizer->removeAllPointClouds();
        visualizer->removeAllShapes();
        method = 0;
        model = 0;
        size = 0;
        iteration = 0;
        n_cam = 0;
        cout << "Method : " << endl;
        cin >> method;
        cout << "Model : " << endl;
        cin >> model;
        cout << "Size : " << endl;
        cin >> size;
        cout << "Iteration : " << endl;
        cin >> iteration;

        view_space->points.resize(content.size());
        auto ptr = view_space->points.begin();
        cout << content.size() << endl;
        for (auto r: content) {
            int _method = stoi(r[0]);
            int _model = stoi(r[1]);
            int _size = stoi(r[2]);
            int _iteration = stoi(r[3]);
            int _index = stoi(r[4]);
            double _qlt = stod(r[8]);
            if ((_method == method) || (method == -1)) {
                if ((_model == model) || (model == -1)) {
                    if ((_size == size) || (size == -1)) {
                        if ((_iteration == iteration) || (iteration == -1)) {
                            x = stod(r[5]);
                            y = stod(r[6]);
                            z = stod(r[7]);
                            n_cam++;
                            (*ptr).x = x;
                            (*ptr).y = y;
                            (*ptr).z = z;
                            (*ptr).r = 255;
                            (*ptr).g = 0;
                            (*ptr).b = 0;
                            cout << "cam_index : " << _index << " - met : " << _method << " - mod : " << _model << " - size : " << _size << " - iter : " << _iteration << endl;
                            cout << "Camera pos is : " << x << ", " << y << ", " << z << endl;
                        }
                    }
                }
            }
            ptr++;
        }
        visualizer->addPointCloud<pcl::PointXYZRGB>(view_space, "View Space");
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "View Space");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(input_cloud, 0, 0, 0);
        visualizer->addPointCloud<pcl::PointXYZ>(input_cloud, single_color, "Cloud");
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud");
        cout << n_cam << endl;
        while (!visualizer->wasStopped()) {
            visualizer->spin();
        }
    }
}


