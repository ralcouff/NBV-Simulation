#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Share_Data.h"
#include "NBV_Planner.h"

using namespace std;

atomic<bool> stop{false}; // End of control program
Share_Data *share_data;   // Shared data area pointer
NBV_Planner *nbv_plan;

void get_command() { // Read command strings from the console
    string cmd;
    string cmd2;
    while (!stop && !share_data->over) {
        cout << "Input command 1.stop 2.over 3.next_iteration 4.change_method:" << endl;
        cin >> cmd;
        if (cmd == "1")
            stop = true;
        else if (cmd == "2")
            share_data->over = true;
        else if (cmd == "3")
            share_data->move_on = true;
        else if (cmd == "4") {
            cout << "Choose your next method iteration" << endl;
            cin >> cmd2;
            int method_number = stoi(cmd2);
            share_data->method_of_IG = get_method(method_number);
            cout << "The method number is:" << method_number << endl;
            cout << "The new method is: " << share_data->method_of_IG << endl;
        } else
            cout << "Wrong command. Retry:" << endl;
    }
    cout << "get_command over." << endl;
}

void get_run() {
    // NBV planning period initialisation
    nbv_plan = new NBV_Planner(share_data);
    // Master Control Cycle
    string status;
    // Real-time reading and planning
    while (!stop && nbv_plan->plan()) {
        // Output if there is a change in status
        if (status != nbv_plan->out_status()) {
            status = nbv_plan->out_status();
            cout << "NBV_Planner's status is " << status << endl;
        }
    }
}

int main(int argc, char **argv) {

    const auto config_file = std::string(argv[1]);
    // Init
    ios::sync_with_stdio(false);
    std::string model_path = std::string(argv[2]);
    std::string model_qlt_path = std::string(argv[3]);
    short method = get_method(std::stoi(argv[4]));
    int n_iter = std::stoi(argv[5]);
    std::string save_folder = std::string(argv[6]);
    std::string string_test_time = std::string(argv[7]);

    cout << "---***--- Launching the NBV algorithm ---***--- " << endl;
    cout << "Configuration file: " << config_file << endl;
    cout << "Model: "<< model_path << endl;
    cout << "Model Quality: " << model_qlt_path << endl;
    cout << "Method: " << method << endl;
    cout << "Reconstruction iterations: " << n_iter << endl;
    cout << "Save folder: " << save_folder << endl;
    cout << "Start time: " << string_test_time << endl;

    // Data area initialisation
    share_data = new Share_Data(config_file, model_path, model_qlt_path, method, n_iter, save_folder, string_test_time);
    // Console read command threads
    thread cmd(get_command);
    // NBV system run threads
    thread runner(get_run);
    // Waiting for the thread to finish
    cmd.join();
    runner.join();
    cout << "System over." << endl;
    return EXIT_SUCCESS;
}

//../DefaultConfiguration.yaml ../3d_models/sphere_quality_colored_1024 ../3d_models/sphere_quality_colored_1024.qlt 0 0 ../sphere_quality_colored_1024.0.500000 2023-07-05
//../DefaultConfiguration.yaml ../3d_models/Armadillo ../3d_models/Armadillo.qlt 101 0 ../Armadillo_0_0.500000 2023-07-05