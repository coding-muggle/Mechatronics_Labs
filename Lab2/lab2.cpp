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
const int robot_tol {50};                  //Tolerance for robot spawn point
const int occupancy_tol {30};               //Minimum distance between all objects that spawn
const int goal_tol {50};                   //Minimum distance in x,y between robot and goal
const int robot_y_min {500};                //Minimum robot y position
const int goal_y_max {300};                 //Maximum goal y position
int obj_x, obj_y, obj_width, obj_height;    //Parameters for object position/size
int num_objects {15};                       //Number of objects in environment

// Grid utility class
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

// Did mission succeed?
bool succeed;

//Task 1
bool is_collision(const Object& robot, const grid_util& grid) {
    int top_left_x = robot.x;
    int top_left_y = robot.y;
    int top_right_x = robot.x + robot.width - 1;
    int top_right_y = robot.y;
    int bottom_left_x = robot.x;
    int bottom_left_y = robot.y + robot.height - 1;
    int bottom_right_x = robot.x + robot.width - 1;
    int bottom_right_y = robot.y + robot.height - 1;
    
    if (grid.grid[top_left_x][top_left_y] == 2) {
        return true;
    }
    if (grid.grid[top_right_x][top_right_y] == 2) {
        return true;
    }
    if (grid.grid[bottom_left_x][bottom_left_y] == 2) {
        return true;
    }
    if (grid.grid[bottom_right_x][bottom_right_y] == 2) {
        return true;
    }
    
    // If all checks pass, no collision
    return false;
}

//Task 3
// Check if position is within boundaries
bool is_within_bounds(const Object& robot) {
    return (robot.x >= 0 && robot.x + robot.width <= width && 
            robot.y >= 0 && robot.y + robot.height <= height);
    }

// Calculate Euclidean distance to goal
double distance_to_goal(const Object& robot, const Object& goal) {
    int robot_center_x = robot.x + robot.width / 2;
    int robot_center_y = robot.y + robot.height / 2;
    int goal_center_x = goal.x + goal.width / 2;
    int goal_center_y = goal.y + goal.height / 2;
    
    return std::sqrt(std::pow(robot_center_x - goal_center_x, 2) + 
                     std::pow(robot_center_y - goal_center_y, 2));
}

// Task 2: Obstacle avoidance function
//void obstacle_avoidance(Object& robot, const std::string& collision_direction, 
//                       const grid_util& grid, std::vector<std::vector<int>>& robot_pos) {
//    
//    // Move perpendicular to the collision direction
//    if (collision_direction == "y_direction") {
//        // Was moving in Y direction, so move in X direction to clear
//        // Try moving right first
//        while (is_collision(robot, grid)) {
//            robot.x += 1;
//            robot_pos.push_back({robot.x, robot.y}); // IMPORTANT: update position for rendering
//        }
//    } 
//    else if (collision_direction == "x_direction") {
//        // Was moving in X direction, so move in Y direction to clear
//        // Try moving down first
//        while (is_collision(robot, grid)) {
//            robot.y += 1;
//            robot_pos.push_back({robot.x, robot.y}); // IMPORTANT: update position for rendering
//        }
//    }
//}

//Task 3
void obstacle_avoidance(Object& robot, const std::string& collision_direction, 
                       const grid_util& grid, std::vector<std::vector<int>>& robot_pos,
                       const Object& goal) {
    
    if (collision_direction == "y_direction") {
        // Calculate position after avoiding right
        Object robot_right = robot;
        while (is_collision(robot_right, grid)) {
            robot_right.x += 1;
        }
        double dist_right = distance_to_goal(robot_right, goal);
        
        // Calculate position after avoiding left
        Object robot_left = robot;
        while (is_collision(robot_left, grid)) {
            robot_left.x -= 1;
        }
        double dist_left = distance_to_goal(robot_left, goal);
        
        // Choose optimal direction
        if (dist_right < dist_left) {
            while (is_collision(robot, grid)) {
                robot.x += 1;
                robot_pos.push_back({robot.x, robot.y});
            }
        } else {
            while (is_collision(robot, grid)) {
                robot.x -= 1;
                robot_pos.push_back({robot.x, robot.y});
            }
        }
    } 
    else if (collision_direction == "x_direction") {
        // Calculate position after avoiding DOWN
        Object robot_down = robot;
        while (is_collision(robot_down, grid)) {
            robot_down.y += 1;
        }
        double dist_down = distance_to_goal(robot_down, goal);
        
        // Calculate position after avoiding UP
        Object robot_up = robot;
        while (is_collision(robot_up, grid)) {
            robot_up.y -= 1;
        }
        double dist_up = distance_to_goal(robot_up, goal);
        
        // Choose optimal direction
        if (dist_down < dist_up) {
            while (is_collision(robot, grid)) {
                robot.y += 1;
                robot_pos.push_back({robot.x, robot.y});
            }
        } else {
            while (is_collision(robot, grid)) {
                robot.y -= 1;
                robot_pos.push_back({robot.x, robot.y});
            }
        }
    }
}
//Task 2
// Function to detect which direction robot was moving when collision occurred
std::string get_collision_direction(const Object& current_robot, const Object& previous_robot) {
    if (current_robot.x != previous_robot.x) {
        return "x_direction";
    } else if (current_robot.y != previous_robot.y) {
        return "y_direction";
    }
    return "unknown";
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
    // also create a copy for predicting collisions
    Object robot_copy = robot;

    grid.writeGridToCSV("grid.csv");

    robot_pos.push_back({robot.x, robot.y});

    // maximum count. Close the loop after 3600 iterations. As the window is displayed at 60fps, this is 60 seconds.
    int max_count = 0;


//Task 2
 Object previous_robot = robot;

   // main loop
while (true)   // main simulation loop
{
        
   // Task 1: Simple movement towards goal 
        //if (robot.x < goal.x) {
        //    robot.x += 1;
        //} else if (robot.x > goal.x) {
        //    robot.x -= 1;
        //}
        
        //if (robot.y < goal.y) {
        //    robot.y += 1;
        //} else if (robot.y > goal.y) {
        //    robot.y -= 1;
        //}
        
        // Check for collision
        //if (is_collision(robot, grid)) {
        //    std::cout << "=====Collision detected! Mission failed.=====" << std::endl;
        //    succeed = false;
        //    break;
        //}
        
        // Check if robot reached goal
        //if (robot.x >= goal.x && robot.x <= goal.x + goal.width &&
        //    robot.y >= goal.y && robot.y <= goal.y + goal.height) {
        //    std::cout << "=====Goal reached! Mission successful.=====" << std::endl;
        //    succeed = true;
        //    break;
        //}
        
        //Task 2
        previous_robot = robot;
        
        // Incorporation of Task 4 navigation
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
        
        // Check for collision after movement
        if (is_collision(robot, grid)) {
            // Determine which direction robot is moving when collision occurred
            std::string collision_dir = get_collision_direction(robot, previous_robot);
            
            // Perform obstacle avoidance
            obstacle_avoidance(robot, collision_dir, grid, robot_pos, goal);
        }
        
        // Check if robot reached goal
        if (robot.x < goal.x + goal.width && robot.x + robot.width > goal.x &&
            robot.y < goal.y + goal.height && robot.y + robot.height > goal.y) {
            std::cout << "=====Goal reached! Mission successful.=====" << std::endl;
            succeed = true;
            break;
        }
        
        // Check boundaries
        if (robot.x < 0 || robot.x >= width || robot.y < 0 || robot.y >= height) {
            std::cout << "=====Robot out of bounds! Mission failed.=====" << std::endl;
            succeed = false;
            break;
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
