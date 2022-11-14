#include <atomic>
#include <iostream>
#include <thread>
typedef unsigned long long pop_t;


using namespace std;

#include "Share_Data.h"
#include "NBV_Planner.h"

atomic<bool> stop{false}; // End of control program
Share_Data* share_data;   // Shared data area pointer
NBV_Planner* nbv_plan;

void get_command()
{ // Read command strings from the console
    string cmd;
    while(!stop && !share_data->over)
    {
        cout << "Input command 1.stop 2.over 3.next_iteration :" << endl;
        cin >> cmd;
        if(cmd == "1")
            stop = true;
        else if(cmd == "2")
            share_data->over = true;
        else if(cmd == "3")
            share_data->move_on = true;
        else
            cout << "Wrong command. Retry :" << endl;
    }
    cout << "get_command over." << endl;
}

void get_run()
{
    // NBV planning period initialisation
    nbv_plan = new NBV_Planner(share_data);
    // Master Control Cycle
    string status = "";
    // Real-time reading and planning
    while(!stop && nbv_plan->plan())
    {
        // Output if there is a change in status
        if(status != nbv_plan->out_status())
        {
            status = nbv_plan->out_status();
            cout << "NBV_Planner's status is " << status << endl;
        }
    }
}

int main(int argc, char** argv)
{
    const auto config_file = argc == 1 ? "../DefaultConfiguration.yaml" : std::string(argv[1]);
    // Init
    ios::sync_with_stdio(false);
    // Data area initialisation
    share_data = new Share_Data(config_file);
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
