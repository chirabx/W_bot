#ifndef ARM_INVERSE_H
#define ARM_INVERSE_H

#include <cmath>
#include <vector>

#include "ros/ros.h"

using namespace std;

class ArmInverse
{
public:
    ArmInverse() {}
    ~ArmInverse() {}

private:
    // Height from base to joint0
    float base_height = 125;
    // Big arm length
    float l1 = 135;
    // Small arm length
    float l2 = 160;
    // Stepper angle (rad/step)
    float step_angle = 0.01125 * M_PI / 180.0;
    // End effector offset in xy plane
    float xy_offset = 40;
    // End effector offset in z direction
    float z_offset = 50;
    // Initial joint angles
    float initial_angles[3] = {0, M_PI / 2, M_PI * 1 / 8};

    int STEP_RANGE[3][2] = {{-8000, 8000}, {0, 6000}, {-8000, 500}};

public:
    vector<int> adjust_position(int x, int y, int z);
    vector<int> step_limiter(vector<int> thetas);
    bool check_steps_reachable(vector<int> steps);
    bool check_valid(int relative_x_dist, int relative_y_dist, int relative_z_dist);
    vector<float> inverse_kinematics(int x, int y, int z);
    vector<int> calculate_steps(vector<float> target_angles);
    vector<int> get_steps_from_absolute_pos(int x, int y, int z);
};

#endif
