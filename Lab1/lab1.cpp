#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "utils.h"
#include "render.h"

//===== Main parameters =====
const int width {800}, height {800};        //Width and height of the environment
const int radius {10};                      //Radius of the robot's circular body
const int min_obj_size {50};                //Maximum object dimension
const int max_obj_size {100};               //Maximum object dimension
const int goal_width {100};                 //Goal width
const int goal_height {100};                //Goal heigth
const int robot_tol {200};                  //Tolerance for robot spawn point
const int occupancy_tol {50};               //Minimum distance between all objects that spawn
const int goal_tol {100};                   //Minimum distance in x,y between robot and goal
const int robot_y_min {500};                //Minimum robot y position
const int goal_y_max {300};                 //Maximum goal y position
int obj_x, obj_y, obj_width, obj_height;    //Parameters for object position/size
int num_objects {0};                        //Number of obstacles in environment

// Grid utility class. Students will not use this for lab 1
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator to spawn robot and goal
random_generator rand_gen;

// Vector of robot positions to pass to renderer code. Update this after each time step!
std::vector<std::vector<int>> robot_pos;

// Did mission succeed? Update this to make sure it succeeds if robot reaches goal, failure if it hits wall.
bool succeed;

//Task 1
bool isRobotOutOfBounds(const Object& robot, int env_width, int env_height) {
    if (robot.x < 0 || robot.y < 0) {
        return true;
    }
    if (robot.x + robot.width > env_width || robot.y + robot.height > env_height) {
        return true;
    }
    return false;
}

//Task 2
bool hasRobotReachedGoal(const Object& robot, const Object& goal) {
    if (robot.x == goal.x && robot.y == goal.y) {
        return true;
    }

    bool x_overlap = (robot.x < goal.x + goal.width) && (robot.x + robot.width > goal.x);
    bool y_overlap = (robot.y < goal.y + goal.height) && (robot.y + robot.height > goal.y);

    return x_overlap && y_overlap;
}

int main(int argc, char const *argv[])
{

    // create robot
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");

    // create the goal
    Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");

    // create the objects
    std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);

    // create copies of robot and goal with their initial positions for purpose of render functions
    Object robot_init = robot;
    Object goal_init = goal;

    grid.writeGridToCSV("grid.csv");

    // place the first robot position to robot_pos
    robot_pos.push_back({robot.x, robot.y});

    // maximum count. Close the loop after 3600 iterations. As the window is displayed at 60fps, this is 60 seconds.
    int max_count=0;

    // Robot was moving too slow so to adjust, the following was done below
    const int step_size = 1;

    // main while loop
    while (true)
    {

//Task 1
  if (isRobotOutOfBounds(robot, width, height)) {
    std::cout << "ERROR: Robot crossed environment boundaries!" << std::endl;
    succeed = false;
    break; 
}

//Task 2
if (hasRobotReachedGoal(robot, goal)) {
    std::cout << "Looks like the Robot will reach the goal!" << std::endl;
    succeed = true;
    break; 
}

// Your navigation logic will go here (replace the example code)
        
//Task 3
// if (robot.x != goal.x) {
    //if (robot.x < goal.x) {
        //robot.x += 1;
    //} else { // robot.y > goal.y
        //robot.x -= 1;
    //}
//} else if (robot.y != goal.y) {
    //if (robot.y < goal.y) {
        //robot.y += 1;
    //} else { // robot.x > goal.x
        //robot.y -= 1;
    //}
//}

    // Task 4
       if (robot.y != goal.y) {
    if (robot.y < goal.y) {
        robot.y += 1;
    } else { // robot.y > goal.y
        robot.y -= 1;
    }
} else if (robot.x != goal.x) {
    if (robot.x < goal.x) {
        robot.x += 1;
    } else { // robot.x > goal.x
        robot.x -= 1;
    }
}

//Task 5
//int dx = goal.x - robot.x;
//int dy = goal.y - robot.y;

//double distance = std::sqrt(dx*dx + dy*dy);

//if (distance > 0) {  
    //double ux = dx / distance;
    //double uy = dy / distance;

    //robot.x += static_cast<int>(std::round(ux));
    //robot.y += static_cast<int>(std::round(uy));
//}

//Task 6
//int target_x = goal.x + goal.width / 2;
//int target_y = goal.y + goal.height / 2;

//int dx = target_x - robot.x;
//int dy = target_y - robot.y;

//double distance = std::sqrt(dx*dx + dy*dy);

//if (distance > 0) {  
    //double ux = dx / distance;
    //double uy = dy / distance;

    //robot.x += static_cast<int>(std::round(ux));
    //robot.y += static_cast<int>(std::round(uy));
//}

// Task 7: straight-line navigation to closest goal corner
// Step 1: pick the closest corner
int goal_corners[4][2] = {
    {goal.x, goal.y},                           // top-left
    {goal.x + goal.width - 1, goal.y},          // top-right  ( -1 to stay inside goal )
    {goal.x, goal.y + goal.height - 1},         // bottom-left
    {goal.x + goal.width - 1, goal.y + goal.height - 1} // bottom-right
};

double min_dist = 2000.0;
int target_x = goal_corners[0][0];
int target_y = goal_corners[0][1];

for (int i = 0; i < 4; ++i) {
    double dist = std::sqrt(
        (goal_corners[i][0] - robot.x) * (goal_corners[i][0] - robot.x) +
        (goal_corners[i][1] - robot.y) * (goal_corners[i][1] - robot.y)
    );
    if (dist < min_dist) {
        min_dist = dist;
        target_x = goal_corners[i][0];
        target_y = goal_corners[i][1];
    }
}

// Step 2: compute deltas & signs
int dx = target_x - robot.x;
int dy = target_y - robot.y;
int sx = (dx > 0) ? 1 : -1;
int sy = (dy > 0) ? 1 : -1;

int abs_dx = std::abs(dx);
int abs_dy = std::abs(dy);

// Step 3: slope logic but always 1-pixel per iteration
static bool x_first_set = false;
static bool x_first;
static int ratio = 0;
static bool init = false;

if (!init) {
    // decide slope once at start
    if (abs_dy > abs_dx) {           // slope > 1 → x is slow axis
        ratio = abs_dy / abs_dx;     // floor(Δy/Δx)
        x_first = true;
    } else {                          // slope ≤ 1 → y is slow axis
        ratio = (abs_dy == 0) ? 0 : abs_dx / abs_dy;
        x_first = false;
    }
    init = true;
}

// Bresenham-like stepping:
// keep counters so we only move 1 pixel each loop
static int counter = 0;

if (robot.x != target_x || robot.y != target_y) {

    if (x_first) {
        // slope > 1 → every loop move mostly in y, sometimes in x
        if (abs_dx > 0) {
            // move in y most of the time
            robot.y += sy;
            counter++;
            if (counter >= ratio) {          // after ratio y-steps, step 1 in x
                robot.x += sx;
                abs_dx--;
                counter = 0;
            }
        } else if (robot.y != target_y) {
            // finished x, just close in y
            robot.y += sy;
        }
    } else {
        // slope ≤ 1 → every loop move mostly in x, sometimes in y
        if (abs_dy > 0) {
            // move in x most of the time
            robot.x += sx;
            counter++;
            if (counter >= ratio) {          // after ratio x-steps, step 1 in y
                robot.y += sy;
                abs_dy--;
                counter = 0;
            }
        } else if (robot.x != target_x) {
            // finished y, just close in x
            robot.x += sx;
        }
    }
}

        // place the current robot position at the time step to robot_pos
        robot_pos.push_back({robot.x, robot.y});
        max_count++;

        // if more than a minute passed (in render window), exit
        if (max_count>=3600) {
            std::cout << "=====1 minute reached with no solution=====" << std::endl;
            break;
        }
    }
    
    // send the results of the code to the renderer
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
    return 0;
}
