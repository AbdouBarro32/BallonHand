#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <sstream>
#include <fstream>

#include <stdexcept>

#include <pthread.h>

#include <pneumaticbox_traj_playback/pneumaticbox.h>
using namespace AirserverAPI;

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pneumaticbox_traj_playback/goals.h>

#define BEAGLE_ADR "192.168.7.2"
#define BEAGLE_PORT 7650

class PneumaticBoxTrajPlayback{
public:
    PneumaticBoxTrajPlayback(const std::string& filename, int num_runs) 
        : nh_(), num_runs_(num_runs), run_count_(0) {
        pub_= nh_.advertise<pneumaticbox_traj_playback::goals>("pressure_goal",1000);
        loadPressureFromFile(filename);
        currentIndex_=0;
    }

    void run(){
        ros::Rate rate(1);
        while(ros::ok()){
            if (!publishPressure()) {
                break;
            }
            rate.sleep();
            ros::spinOnce();
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::vector<std::vector<float>> pressures;
    std::vector<std::string> grip_commands_; // Added to store grip commands
    size_t currentIndex_;
    int num_runs_;
    int run_count_;

    void loadPressureFromFile(const std::string& filename){
        std::ifstream file(filename);
        if(!file.is_open()){
            ROS_ERROR_STREAM("Failed to open file: " << filename);
            return;
        }
        std::string line;
        int line_number = 0;
        while(std::getline(file,line)){
            line_number++;
            std::vector<float> current_pressures_on_line;
            std::string current_grip_command;
            std::istringstream iss(line);
            float p_value;
            
            // Read 6 pressure values
            for (int i = 0; i < 6; ++i) {
                if (!(iss >> p_value)) {
                    ROS_WARN_STREAM("Line " << line_number << ": Failed to read 6 pressure values. Content: " << line);
                    current_pressures_on_line.clear(); // Mark as invalid for this line
                    break;
                }
                current_pressures_on_line.push_back(p_value);
            }

            if (current_pressures_on_line.size() == 6) {
                // Read the rest of the line as the grip command
                // std::ws consumes leading whitespace before reading the grip string.
                if (std::getline(iss >> std::ws, current_grip_command)) {
                    if (!current_grip_command.empty()) {
                        pressures.push_back(current_pressures_on_line);
                        grip_commands_.push_back(current_grip_command);
                    } else {
                        ROS_WARN_STREAM("Line " << line_number << ": Parsed 6 pressures but grip string is empty. Line: " << line);
                        continue; // Skip this line or handle as an error
                    }
                } else {
                    ROS_WARN_STREAM("Line " << line_number << ": Read 6 pressures, but failed to read grip string. Line: " << line);
                    continue; // Skip this line
                }
            } else {
                // Not enough pressure values read, warning already issued by the loop.
                continue; 
            }
        }
        if (pressures.empty()) {
            ROS_WARN("No valid data lines found in file: %s", filename.c_str());
        } else {
            ROS_INFO("Successfully loaded %zu data lines from %s", pressures.size(), filename.c_str());
        }
    }

    // Returns false if finished all runs, true otherwise
    bool publishPressure(){
        if(pressures.empty() || grip_commands_.empty()){
            ROS_WARN("No Pressure or Grip data loaded to publish.");
            return false; // Stop if no data
        }
        
        // Ensure sizes match, which should be guaranteed by loadPressureFromFile
        if (pressures.size() != grip_commands_.size()) {
            ROS_ERROR("Data inconsistency: pressures and grip_commands_ sizes differ. Aborting.");
            return false;
        }

        if(currentIndex_ >= pressures.size()){
            currentIndex_ = 0; // Reset for next run
            run_count_++;      // Increment run counter
            
            // num_runs_ == 0 means loop indefinitely
            if(num_runs_ > 0 && run_count_ >= num_runs_){
                ROS_INFO("Completed %d run(s) of the trajectory file.", run_count_);
                return false; // All specified runs done
            }
            ROS_INFO("Starting run #%d of the trajectory file.", run_count_ + 1);
        }

        // Additional check for data integrity for the current index
        if (currentIndex_ >= grip_commands_.size()) { // Should not happen if pressures.size() == grip_commands_.size()
             ROS_ERROR("Critical Data inconsistency: currentIndex_ out of bounds for grip_commands_.");
             return false; // Critical error, stop publishing
        }
        if (pressures[currentIndex_].size() != 6) {
            ROS_ERROR_STREAM("Data inconsistency: Expected 6 pressure values for index " << currentIndex_ 
                             << ", but found " << pressures[currentIndex_].size() << ". Skipping this entry.");
            currentIndex_++; // Try to recover by skipping to the next entry
            return true;     // Continue to allow processing of subsequent valid entries
        }

        const std::string& current_grip = grip_commands_[currentIndex_];
        const std::vector<float>& current_pressure_set = pressures[currentIndex_];

        pneumaticbox_traj_playback::goals msg;
        for(size_t i = 0; i < 6; ++i){ // Iterate through the 6 pressure channels
            msg.pressure_goal[i]=pressures[currentIndex_][i];
        }
        msg.grip=grip_commands_[currentIndex_];
        pub_.publish(msg);
        ROS_INFO("Published command");
        ros::Duration(0.5).sleep();
        currentIndex_++; // Move to the next line of data for the next cycle
        return true;     // Successfully published, continue
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc,argv,"pneumaticbox_traj_playback");

    if(argc != 3){
        std::cerr << "Usage: " << argv[0] << " <filename> <num_runs (0=loop, 1=once, 2=twice, ...)>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    int num_runs = std::stoi(argv[2]);
    PneumaticBoxTrajPlayback node(filename, num_runs);
    node.run();

    return 0;
}
