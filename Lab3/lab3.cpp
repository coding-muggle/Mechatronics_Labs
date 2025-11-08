#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include "utils.h"
#include "render.h"

// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
// Task 1 & 2
class my_robot : public Object {
private:
    int range;  // LiDAR range
public:
    // Constructor that calls Object constructor
  static const int radius = 10;
    
    // Constructor that calls Object constructor
    my_robot(int width, int height, int env_width, int env_height, int lidar_range) 
        : Object(width, height, env_width, env_height), range(lidar_range) {
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
    }
    
    std::vector<std::vector<int>> grid;
    
    // LiDAR mapping function
    void map_environment(grid_util& true_grid) {
        int center_x = this->x + radius;  // Robot center x
        int center_y = this->y + radius;  // Robot center y
        
        // Task 2: Travel through the square area around the robot
        for (int i = center_x - range; i <= center_x + range; i++) {
            for (int j = center_y - range; j <= center_y + range; j++) {
                // Check if within circle using circle equation
                if ((i - center_x)*(i - center_x) + (j - center_y)*(j - center_y) <= range*range) {
                    // Check bounds
                    if (i >= 0 && i < 800 && j >= 0 && j < 800) {
                        // Use Object's grid_value function to get true grid value
                        int true_value = Object::grid_value(true_grid, this, i, j, range);
                        if (true_value != -1) { 
                            grid[i][j] = true_value;
                        }
                    }
                }
            }
        }
    }
    
    // Task 2
    // Collision Detection function
    std::pair<int, int> collision_detection(int tol) {
        int center_x = this->x + radius;
        int center_y = this->y + radius;
        
        int v = 0; // vertical: 1=top hit, 0=free, -1=bottom hit
        int h = 0; // horizontal: 1=right hit, 0=free, -1=left hit
        
        // Check top (y - tol)
        if (center_y - tol >= 0 && center_y - tol < 800 && 
            grid[center_x][center_y - tol] == 1) {
            v = 1;
        }
        // Check bottom (y + tol)
        else if (center_y + tol >= 0 && center_y + tol < 800 && 
                 grid[center_x][center_y + tol] == 1) {
            v = -1;
        }
        // Otherwise free
        else {
            v = 0;
        }
        
        // Check right (x + tol)
        if (center_x + tol >= 0 && center_x + tol < 800 && 
            grid[center_x + tol][center_y] == 1) {
            h = 1;
        }
        // Check left (x - tol)
        else if (center_x - tol >= 0 && center_x - tol < 800 && 
                 grid[center_x - tol][center_y] == 1) {
            h = -1;
        }
        // Otherwise free
        else {
            h = 0;
        }
        
        return std::make_pair(v, h);
    }
    
    // Move function based on collision detection
    void move_based_on_collision(int h, int v) {
        if (h == 1 && v == 1) {
            this->y += 1; // move down (top-right hit)
        }
        else if (h == 1 && v == 0) {
            this->y += 1; // move down (right hit)
        }
        else if (h == 1 && v == -1) {
            this->x -= 1; // move left (bottom-right hit)
        }
        else if (h == 0 && v == 1) {
            this->x += 1; // move right (top hit)
        }
        else if (h == 0 && v == 0) {
            this->x -= 1; // move left (free space)
        }
        else if (h == 0 && v == -1) {
            this->x -= 1; // move left (bottom hit)
        }
        else if (h == -1 && v == 1) {
            this->x += 1; // move right (top-left hit)
        }
        else if (h == -1 && v == 0) {
            this->y -= 1; // move up (left hit)
        }
        else if (h == -1 && v == -1) {
            this->y -= 1; // move up (bottom-left hit)
        }
    }
    
    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        // Determine the maximum row size by finding the size of the longest inner vector
        size_t maxRowSize = 0;
        for (const auto& col : grid) {
            if (col.size() > maxRowSize) {
                maxRowSize = col.size();
            }
        }

        // Output the grid in transposed form (columns become rows in CSV)
        for (size_t row = 0; row < maxRowSize; ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                if (row < grid[col].size()) {
                    file << grid[col][row];
                }
                if (col < grid.size() - 1) {
                    file << ","; // Add comma except after the last element
                }
            }
            file << "\n"; // New line after each row
        }

        file.close();
        std::cout << "Robot's grid written to " << filename << std::endl;
    }      
};

//===== Main parameters =====
const int env_width {800}, env_height {800};        //Width and height of the environment
const int radius {10};                              //Radius of the robot's circular body
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3
int lidar_range{50};                                //Lidar range, radiating from center of robot

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT AND WALLS==========

    // read config file
    std::pair<std::string, bool> config = read_csv();

    // create the walls
    std::vector<Object*> walls;

    // normal perpendicular walls
    if (config.first == "environment1.csv") {
        walls = grid.create_walls(config.first);
    }
    // angled walls
    else {
        walls = grid.create_angled_walls(config.first);
    }

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv"); 

    // 2*radius is used for width/height of robot
    my_robot robot(2*radius, 2*radius, env_width, env_height, lidar_range);

    // create a copy. change this to a my_robot class as well
    my_robot robot_init = robot;

    // Push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;
    // Run the program indefinitely until robot hits the goal or an obstacle
    
    while (true)
{
    limit_count++;

    // Call LiDAR mapping function
    robot.map_environment(grid);
    
    // Task 1
    // robot.x -= 1;      

    // Task 2
    // Wall following algorithm
    int tolerance = 20; // constant distance from wall
    
    // Get collision detection results
    auto collision = robot.collision_detection(tolerance);
    int v = collision.first;  // vertical collision
    int h = collision.second; // horizontal collision
    
    // Move based on collision detection
    robot.move_based_on_collision(h, v);
    
    robot_pos.push_back({robot.x, robot.y});
    
    if (limit_count>=3600) {
        std::cout << "====Program terminated after 3600 iterations====" << std::endl;
        break;
    }
}

    std::vector<std::vector<int>> vec(800, std::vector<int>(800, -1));
    float accuracy = grid.grid_accuracy(robot.grid);
std::cout << "Percent of walls correctly mapped: " << accuracy*100.0 << "%" << std::endl;
if (config.second){
    render_window(robot_pos, walls, robot_init, env_width, env_height);
}
render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range);
    
    return 0;
}
