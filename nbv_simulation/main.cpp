#include <atomic>
#include <iostream>
#include <thread>
#include <vector>

#include "Share_Data.h"
#include "NBV_Planner.h"

using namespace std;

atomic<bool> stop{false}; // End of control program
Share_Data *share_data;   // Shared data area pointer
NBV_Planner *nbv_plan;

short get_method(int n_test) {
    short method;
    switch (n_test){
        case 0:
            method = OursIG;
            break;
        case 101:
            method = Test_one;
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
//            share_data->method_of_IG = method_number;
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
    const auto config_file = argc == 1 ? "../DefaultConfiguration.yaml" : std::string(argv[1]);
    // Init
    ios::sync_with_stdio(false);
    std::vector<int> tests{0,101,102};
    std::vector<int> models{0, 1, 2, 3, 4};
    std::vector<int> sizes{1024, 4096};
    std::vector<int> reconstructionIterations{1, 2, 3, 4, 5};

    char str_time [80];
    std::time_t rawTime;
    struct tm * timeInfo;

    time (&rawTime);
    timeInfo = std::localtime(&rawTime);
    std::strftime(str_time, 80, "%F-%H-%M", timeInfo);
    std:: string string_test_time = std::string(str_time);

    for (int n_test: tests) {
        short method = get_method(n_test);
        for (int n_model: models){
            for (int n_size: sizes) {
                for (int n_iter: reconstructionIterations) {
                    share_data = new Share_Data(config_file, n_model, n_size, n_iter, method, string_test_time);
                    thread runner(get_run);
                    runner.join();
                    cout << "System over." << endl;
                }
            }
        }
    }

    // Data area initialisation
//    share_data = new Share_Data(config_file);
//    // Console read command threads
//    thread cmd(get_command);
//    // NBV system run threads
//    thread runner(get_run);
//    // Waiting for the thread to finish
//    cmd.join();
//    runner.join();
//    cout << "System over." << endl;
    return EXIT_SUCCESS;
}
