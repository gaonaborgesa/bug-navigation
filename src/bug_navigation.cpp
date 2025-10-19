#include <cmath>
#include <iostream>
#include <signal.h>

#include <mbot_bridge/robot.h>

#include <bug_navigation/common/utils.h>

using namespace std;

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}


int main() {
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the robot.
    mbot_bridge::MBot robot;
    // Reset the robot odometry to zero.
    robot.resetOdometry();
    sleepFor(0.1);

    // initialize variables 
    // Create empty vectors to store the scan data.
    std::vector<float> ranges;
    std::vector<float> thetas;

    // read lidar & odometry
    robot.readLidarScan(ranges, thetas);
    std::vector<float> current_position; 

    // for cross product
    std::vector<float> v_up = {0,0,1};

    float goal_x, goal_y;  // Variables to hold the goal.
    bool goal_met = false; // Set goal met to false
    float setpoint = .5;
    float slice_size = (M_PI/180) * 45 ; // Degree to radians
 
    
    //Get input from user for x and y
    std::cout<<"give me an x value"<<endl;
    std::cin >> goal_x;
    std::cout <<"give me a y value"<<endl;
    std::cin >> goal_y;



    while (!goal_met) {

        // read lidar & odometry
        robot.readLidarScan(ranges, thetas);
        current_position = robot.readOdometry();    // world frame - will return [x, y, theta]
        sleepFor(0.1);

        // calc distance to goal
        float dx = (goal_x - current_position[0]);
        float dy = (goal_y - current_position[1]);

        float angle_to_goal = atan2(dy, dx); 

        // check if goal reached
        float min_dis = .03;


        if (abs(dy) <= min_dis){
            if(abs(dx) <= min_dis) {
                std::cout <<"----------- STOP --------------" << endl;
                std::cout<<"Final Position X: " << current_position[0] << endl;
                std::cout<<"Final Position Y: " << current_position[1]  << endl;
                robot.stop();
                goal_met = true;
                break;
            }
        }

        // to check for obstacles in direction of goal
        int min_idx = findMinDistInSlice(ranges, thetas, angle_to_goal, slice_size);
        float dis_to_ob = ranges[min_idx]; // magnitude
        float ob_angle = thetas[min_idx];

        
        if (dis_to_ob < setpoint){
            
            //do wallfollower
            std::cout<<"--- !!!! OBJECT DETECTED !!!! ... "<< endl;
            std::cout<<"WallFollower-------" << endl;
 
            // Cross Product Calc:
            std::vector<float> v_to_wall = { (1*cos(ob_angle)) , (1*sin(ob_angle)) , 0 };
            std::vector<float> v_forward = crossProduct(v_to_wall, v_up);

            // p-control (correction vector)
            float kp = 0.05; //acceleration (constant)
            float err = dis_to_ob - setpoint;
            float velocity = kp * err;

            // direction 
            float vx = velocity * cos(ob_angle);
            float vy =  velocity * sin(ob_angle);

            // adding vectors
            float vxn = vx + v_forward[0];
            float vyn = vy + v_forward[1];

            // normalize speed
            float max_speed = 0.4;
            float min_speed = 0.1; //was 0.05, increase to .1 for testing
            float speed = sqrt( (vxn*vxn) + (vyn*vyn) );

            if (speed > max_speed) {
                vxn = max_speed * vxn / speed;
                vyn = max_speed * vyn / speed;
            } else if (speed > 0 && speed < min_speed) {
                vxn = min_speed * vxn / speed;
                vyn = min_speed * vyn / speed;
            }

            // final control command
            robot.drive( vxn , vyn , 0 );


        } else {
            //drive to goal
            std::cout<<"DRIVE TO POSE RUNNING... "<< endl;

            std::cout<<"Odometry reading X: " << current_position[0] << endl;
            std::cout<<"Odometry reading Y: " << current_position[1]  << endl;

            float vz = current_position[2];    // only for correction 

            // rotation transformation 
            float rot_x = dx * cos(vz) + dy * sin(vz);
            float rot_y = -dx * sin(vz) + dy * cos (vz);

            // normalize speed
            float max_speed = 0.4;
            float min_speed = 0.1;
            float speed = sqrt( (rot_x*rot_x) + (rot_y*rot_y) );

            if (speed > max_speed) {
                rot_x = max_speed * rot_x / speed;
                rot_y = max_speed * rot_y / speed;
            } else if (speed > 0 && speed < min_speed) {
                rot_x = min_speed * rot_x / speed;
                rot_y = min_speed * rot_y / speed;
            }

            // final drive command
            robot.drive(rot_x, rot_y, 0);
                // sleepFor(1); // buffer to reset odometry   
        }


        if(ctrl_c_pressed) {
            break;
        }
    }

    // Stop the robot.
    robot.stop();

}
