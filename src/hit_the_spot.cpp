#include <iostream>
#include <cmath>

#include <mbot_bridge/robot.h>
#include <bug_navigation/common/utils.h>
using namespace std;


void driveToPose(float goal_x, float goal_y, float goal_theta, mbot_bridge::MBot& robot) {

    vector<float> robo_odometry  = robot.readOdometry(); // world frame - will return [x, y, theta]
    bool reachedThreshold = false;


    while (!reachedThreshold){ 

        robo_odometry = robot.readOdometry(); 
        cout<<"Odometry reading X: " << robo_odometry[0] << endl;
        cout<<"Odometry reading Y: " << robo_odometry[1]  << endl;
        
        // err in respect to odometry 
        float vx = goal_x - robo_odometry[0];
        float vy = goal_y - robo_odometry[1];
        float vz = robo_odometry[2];

        float min_dis = .03;
        if (abs(vx) <= min_dis || abs(vy) <= min_dis){
            cout <<"----------- STOP --------------";
            cout<<"Final Position X: " << vx << endl;
            cout<<"Final Position Y: " << vy  << endl;
            robot.stop();
            reachedThreshold = true; 
        }

        // rotation transformation 
        float rot_vx = vx * cos(vz) + vy * sin(vz);
        float rot_vy = -vx * sin(vz) + vy * cos (vz);

        // normalize speed
        float max_speed = 0.4;
        float min_speed = 0.05;
        float speed = sqrt( (rot_vx*rot_vx) + (rot_vy*rot_vy) );

        if (speed > max_speed) {
            rot_vx = max_speed * rot_vx / speed;
            rot_vy = max_speed * rot_vy / speed;
        } else if (speed > 0 && speed < min_speed) {
            rot_vx = min_speed * rot_vx / speed;
            rot_vy = min_speed * rot_vy / speed;
        }

        // final drive command
        robot.drive(rot_vx, rot_vy, goal_theta);
            sleepFor(1); // buffer to reset odometry   
    }

}







int main(){

    // Initialize the robot.
    mbot_bridge::MBot robot;
    
    // Reset the robot odometry to zero at the beginning of the run.
    robot.resetOdometry();
    sleepFor(1); // buffer to reset odometry

    float target_x;  //init variables
    float target_y;

    //Get input from user for x and y
    cout<<"give me an x value"<<endl;
    cin >> target_x;
    cout <<"give me a y value"<<endl;
    cin >> target_y;

    driveToPose(target_x, target_y, 0, robot);
 
    // Stop the robot before exiting.
    robot.stop();
}
