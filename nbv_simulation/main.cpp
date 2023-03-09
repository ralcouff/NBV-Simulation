#include <atomic>

#include "Share_Data.h"
#include "NBV_Planner.h"

using namespace std;

Share_Data *share_data; // Object containing all the data for the project
NBV_Planner *nbv_plan; // The NBV Planner

int main(int argc, char **argv) {
    const auto config_file = argc == 1 ? "../DefaultConfiguration.yaml" : std::string(argv[1]);
    share_data = new Share_Data(config_file);
    nbv_plan = new NBV_Planner(share_data);

    /* While the algorithm hasn't finished */
    string status;
    while (nbv_plan->plan()) {
        if (status != nbv_plan->out_status()) {
            status = nbv_plan->out_status();
            cout << "NBV_Planner's status is " << status << endl;
        }
    }
}