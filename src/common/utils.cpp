#include <bug_navigation/common/utils.h>

std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2)
{
    std::vector<float> res;

     // v1 = v_to_wall
    float ax = v1[0];
    float ay = v1[1];
    float az = v1[2];

    // v2 = v_up
    float bx = v2[0];
    float by = v2[1];
    float bz = v2[2];


    float cx = ((ay) * (bz)) - ((az) * (by));
    float cy = ((az) * (bx)) - ((ax) * (bz));
    float cz = ((ax) * (by)) - ((ay) * (bx));

    res = {cx, cy, cz};

    return res;
}

int findMinDist(const std::vector<float>& ranges) 
{
    float min_val = std::numeric_limits<float>::infinity(); // Set to largest float possible
    int min_idx = -1; 
  
    for (int i = 0; i < ranges.size(); i++){ //Go through ranges
        if (ranges[i] > 0) {
            if (ranges[i] <= min_val) {
                min_val = ranges[i];
                min_idx = i;              //Set min_inx to current range
            }
        }
    }
    return min_idx;
}


int findMinDistInSlice(const std::vector<float>& ranges, const std::vector<float>& thetas,
                       float goal_angle, float slice_size)
{
    int min_idx = -1;
    float min_dist = HIGH;
    // Iterate through every ray.
    for (int i = 0; i < thetas.size(); i++) {
        // Only look at scans within the view angle.
        if(abs(wrapAngle(goal_angle - thetas[i])) < slice_size) {
            if(ranges[i] < min_dist && ranges[i] > 0) {
                min_dist = ranges[i];
                min_idx = i;
            }
        }
    }

    return min_idx;
}
