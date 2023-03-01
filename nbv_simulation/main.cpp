#include <atomic>
#include <iostream>
#include <thread>

#include "Share_Data.h"
#include "NBV_Planner.h"
#include "Visual.h"

using namespace std;

atomic<bool> stop{false}; // End of control program
Share_Data *share_data;   // Shared data area pointer
NBV_Planner *nbv_plan;

void get_command() { // Read command strings from the console
    string cmd;
    while (!stop && !share_data->over) {
        cout << "Input command 1.stop 2.over 3.next_iteration :" << endl;
        cin >> cmd;
        if (cmd == "1")
            stop = true;
        else if (cmd == "2")
            share_data->over = true;
        else if (cmd == "3")
            share_data->move_on = true;
        else
            cout << "Wrong command. Retry :" << endl;
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
    std::vector<int> tests{101, 102, 103, 104, 105, 106, 107, 108};
//    std::vector<int> tests{0};
    std::vector<int> models{0, 1, 2, 3, 4};
//    std::vector<int> models{0};
    std::vector<int> sizes{12, 50};
//    std::vector<int> sizes{12};
    char str_time [80];
    std::time_t rawtime;
    struct tm * timeinfo;

    time (&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(str_time, 80, "%F-%H-%M", timeinfo);
    std:: string string_test_time = std::string(str_time);

//    Visual::voir();
    for (int n_test: tests) {
        short method;
        cout << n_test << endl;
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
            case 103:
                method = Test_three;
                break;
            case 104:
                method = Test_four;
                break;
            case 105:
                method = Test_five;
                break;
            case 106:
                method = Test_six;
                break;
            case 107:
                method = Test_seven;
                break;
            case 108:
                method = Test_eight;
                break;
            case 109:
                method = Test_nine;
                break;
        }
        cout << method << endl;
        for (int n_model: models) {
            for (int n_size: sizes) {
                // Data area initialisation
                share_data = new Share_Data(config_file, n_model, n_size, method, string_test_time);
                // Console read command threads
//                thread cmd(get_command);
                // NBV system run threads
                thread runner(get_run);
                // Waiting for the thread to finish
//                cmd.join();
                runner.join();
                cout << "System over." << endl;
            }
        }
    }
    return EXIT_SUCCESS;
}
