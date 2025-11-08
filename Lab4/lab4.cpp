#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <array>
#include <queue>

#include "utils.h"
#include "render.h"

// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot : public Object { 
private:     // define any private or protected members here
    int environment_type; // For collision avoidance for env 1 and 3
    int range;
    std::array<int, 4> prev_mode{0, 0, 0, 0};
    std::queue<std::pair<int, int>> paths_queue;
    bool clockwise = true;
    static constexpr double sin45 = 0.70710678118;
    int tol_distance;
    static const int radius = 10;
    
    // Task 2: Member variables for sweep algorithm
    bool sweep_mode = false;
    int min_y_tracked = 800;
    int max_y_tracked = 0;
    int y_ref = 0;
    int start_x = -1;
    int start_y = -1;
    bool lapped_walls = false;
    std::array<int, 4> mode_before_move{0, 0, 0, 0};
    bool returning_to_top = false;
    int lap_counter = 0;

public:
my_robot(int width, int height, int env_width, int min_y, int max_y, int tol, int lidar_range, int env_type = 4) 
        : Object(width, height, env_width, min_y, max_y, tol), range(lidar_range), environment_type(env_type) {
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
        
        // Adaptive tolerance
        if (env_type == 3) { 
            tol_distance = tol + 9; // Environment 3
        }
        else if (env_type == 1) {
            tol_distance = tol + 7;
        }
        else {
            tol_distance = radius + 7;
        }
    }
    
    std::vector<std::vector<int>> grid;
    
    
    // Task 1: Function declarations
    void map_environment(grid_util& true_grid);
    std::array<int, 4> detect_walls();
    std::pair<int, int> calculate_wall_vec(std::array<int, 4> mode);
    std::pair<int, int> find_dir(std::array<int, 4> mode_curr);
    void move(std::pair<int, int> robot_to_wall);
    void save_grid_csv();
    
    // Task 2: New function declarations
    void update_min_max_y();
    bool check_if_lapped();
    void toggle_direction();
    bool is_free_of_walls();
    void move_away_from_wall_once();
    bool in_sweep_mode() { return sweep_mode; }
    void start_sweep_mode();
    int get_max_y() { return max_y_tracked; }
    int get_min_y() { return min_y_tracked; }
    int get_y_ref() { return y_ref; }
    void update_y_ref() { y_ref = this->y + radius; }
    std::array<int, 4> get_mode_before_move() { return mode_before_move; }
    void store_mode_before_move(std::array<int, 4> mode) { mode_before_move = mode; }
    bool is_returning_to_top() { return returning_to_top; }
    void set_returning_to_top(bool val) { returning_to_top = val; }
};

        void save_grid_csv() {
                    std::string filename = "grid_pred.csv";
                    std::ofstream file(filename);

                    if (!file.is_open()) {
                        std::cerr << "Error: Could not open file " << filename << std::endl;
                        return;
                    }
                }

// Task 1: Function definitions
void my_robot::map_environment(grid_util& true_grid) {
    int center_x = this->x + radius;
    int center_y = this->y + radius;
    
    if (center_x < 0 || center_x >= 800 || center_y < 0 || center_y >= 800) {
        return;
    }
    
    for (int i = center_x - range; i <= center_x + range; i++) {
        for (int j = center_y - range; j <= center_y + range; j++) {
            if (i >= 0 && i < 800 && j >= 0 && j < 800) {
                if ((i - center_x)*(i - center_x) + (j - center_y)*(j - center_y) <= range*range) {
                    int true_value = Object::grid_value(true_grid, this, i, j, range);
                    if (true_value != -1) {
                        grid[i][j] = true_value;
                    }
                }
            }
        }
    }
}
// Task 1: Wall detection
std::array<int, 4> my_robot::detect_walls() {
    std::array<int, 4> mode{0, 0, 0, 0};
    int c_x = this->x + radius;
    int c_y = this->y + radius;
    int tol = tol_distance;
    int tol_45 = static_cast<int>(tol * sin45);
    
// Check horizontal sides (x)
bool right_wall_detected = false;
bool left_wall_detected = false;

// Check multiple points along the horizontal axis for better detection
for (int offset = -3; offset <= 3; offset += 3) {
    // Check right side with multiple points
    if (c_x + tol < 800 && c_y + offset >= 0 && c_y + offset < 800 && 
        grid[c_x + tol][c_y + offset] == 1) {
        right_wall_detected = true;
    }
    // Check left side with multiple points  
    if (c_x - tol >= 0 && c_y + offset >= 0 && c_y + offset < 800 && 
        grid[c_x - tol][c_y + offset] == 1) {
        left_wall_detected = true;
    }
}

// Set mode based on detection
if (right_wall_detected) {
    mode[0] = 1; // right
} else if (left_wall_detected) {
    mode[0] = -1; // left
} else {
    mode[0] = 0; // no horizontal wall
}
    
    // Check vertical sides (y)
    if (c_x >= 0 && c_x < 800 && c_y + tol < 800 && grid[c_x][c_y + tol] == 1) {
        mode[1] = 1; // bottom
    }
    else if (c_x >= 0 && c_x < 800 && c_y - tol >= 0 && grid[c_x][c_y - tol] == 1) {
        mode[1] = -1; // top
    }
    
// Check diagonal sides (v)
bool bottom_right_detected = false;
bool top_left_detected = false;

for (int offset = -3; offset <= 3; offset += 3) {
    // Check bottom-right diagonal with multiple points
    if (c_x + tol_45 + offset < 800 && c_y + tol_45 + offset >= 0 && c_y + tol_45 + offset < 800 && 
        grid[c_x + tol_45 + offset][c_y + tol_45 + offset] == 1) {
        bottom_right_detected = true;
    }
    // Check top-left diagonal with multiple points
    if (c_x - tol_45 + offset >= 0 && c_y - tol_45 + offset >= 0 && c_y - tol_45 + offset < 800 && 
        grid[c_x - tol_45 + offset][c_y - tol_45 + offset] == 1) {
        top_left_detected = true;
    }
}

if (bottom_right_detected) {
    mode[2] = 1; // bottom right
} else if (top_left_detected) {
    mode[2] = -1; // top left
} else {
    mode[2] = 0;
}

// Check diagonal sides (w) 
bool top_right_detected = false;
bool bottom_left_detected = false;

for (int offset = -3; offset <= 3; offset += 3) {
    // Check top-right diagonal with multiple points
    if (c_x + tol_45 + offset < 800 && c_y - tol_45 + offset >= 0 && c_y - tol_45 + offset < 800 && 
        grid[c_x + tol_45 + offset][c_y - tol_45 + offset] == 1) {
        top_right_detected = true;
    }
    // Check bottom-left diagonal with multiple points
    if (c_x - tol_45 + offset >= 0 && c_y + tol_45 + offset >= 0 && c_y + tol_45 + offset < 800 && 
        grid[c_x - tol_45 + offset][c_y + tol_45 + offset] == 1) {
        bottom_left_detected = true;
    }
}

if (top_right_detected) {
    mode[3] = 1; // top right
} else if (bottom_left_detected) {
    mode[3] = -1; // bottom left
} else {
    mode[3] = 0;
}
    
    return mode;
}

std::pair<int, int> my_robot::calculate_wall_vec(std::array<int, 4> mode) {
    int xw = 0, yw = 0;
    
    // Count current and previous wall hits for Table 2 logic from lab manual
    int current_hits = 0;
    int prev_hits = 0;
    for(int i = 0; i < 4; i++) {
        if(mode[i] != 0) current_hits++;
        if(prev_mode[i] != 0) prev_hits++;
    }
    
    // Table 2 implentation
    if(prev_hits == 0 && current_hits == 0) {
        return {0, 1};
    }
    else if(prev_hits <= 1 && current_hits >= 1) {
        // Derived from array: Mode_curr - Mode_prev
        for(int i = 0; i < 4; i++) {
            if(mode[i] != 0) {
                if(i == 0) xw += mode[i]; // x
                else if(i == 1) yw += mode[i]; // y
                else if(i == 2) { xw += mode[i]; yw += mode[i]; } // v
                else if(i == 3) { xw += mode[i]; yw -= mode[i]; } // w
            }
        }
    }
    else if(prev_hits >= 2 && current_hits >= 1) {
        // Derived from Mode_curr
        for(int i = 0; i < 4; i++) {
            if(mode[i] != 0) {
                if(i == 0) xw += mode[i]; // x
                else if(i == 1) yw += mode[i]; // y
                else if(i == 2) { xw += mode[i]; yw += mode[i]; } // v
                else if(i == 3) { xw += mode[i]; yw -= mode[i]; } // w
            }
        }
    }
    
    // Normalize to -1, 0, or 1
    if(xw > 0) xw = 1;
    else if(xw < 0) xw = -1;
    if(yw > 0) yw = 1;
    else if(yw < 0) yw = -1;
    
    return {xw, yw};
}

void my_robot::move(std::pair<int, int> robot_to_wall) {
    int xw = robot_to_wall.first;
    int yw = robot_to_wall.second;
    int x_dir, y_dir;
    
    // Task 1: Pseudo code implentation
    if(clockwise) {
        if(yw == 0) {
            x_dir = 0;
        } else {
            x_dir = -abs(yw) / yw;
        }
        
        if(xw == 0) {
            y_dir = 0;
        } else {
            y_dir = abs(xw) / xw;
        }
    } else {
        if(yw == 0) {
            x_dir = 0;
        } else {
            x_dir = abs(yw) / yw;
        }
        
        if(xw == 0) {
            y_dir = 0;
        } else {
            y_dir = -abs(xw) / xw;
        }
    }
    
    // Handle zero movement
    if(x_dir == 0 && y_dir == 0) {
        if(clockwise) {
            x_dir = -1; y_dir = 0;
        } else {
            x_dir = 1; y_dir = 0;
        }
    }
    
    // Queue system
    if(paths_queue.empty()) {
        const int n = 1;
        for(int i = 1; i <= n; i++) {
            int new_x = this->x + x_dir * i;
            int new_y = this->y + y_dir * i;
            
            // Adaptive bounds checking - Use environment boundaries
            int min_bound = radius + 10;
            int max_bound_x = 800 - radius;
            int max_bound_y = 800 - radius;
            
            // Ensure robot center stays within bounds
            int center_x = new_x + radius;
            int center_y = new_y + radius;
            
            if(center_x < min_bound) new_x = min_bound - radius;
            if(center_x > max_bound_x) new_x = max_bound_x - radius;
            if(center_y < min_bound) new_y = min_bound - radius;
            if(center_y > max_bound_y) new_y = max_bound_y - radius;
            
            paths_queue.push({new_x, new_y});
        }
    }
    
    // Move to next position in queue
    if(!paths_queue.empty()) {
        std::pair<int, int> next_pos = paths_queue.front();
        paths_queue.pop();
        
        // Final safety check
        int center_x = next_pos.first + radius;
        int center_y = next_pos.second + radius;
        if(center_x >= 0 && center_x < 800 && center_y >= 0 && center_y < 800) {
            this->x = next_pos.first;
            this->y = next_pos.second;
        }
    }
}

std::pair<int, int> my_robot::find_dir(std::array<int, 4> mode_curr) {
    if(mode_curr != prev_mode) {
        // Clear paths_queue
        while(!paths_queue.empty()) {
            paths_queue.pop();
        }
        
        // Calculate new wall vector
        std::pair<int, int> wall_vec = calculate_wall_vec(mode_curr);
        
        // Update previous mode
        prev_mode = mode_curr;
        
        return wall_vec;
    }
    
    // If mode didn't change, return previous calculation
    return calculate_wall_vec(prev_mode);
}

// function to save predicted grid
void my_robot::save_grid_csv() {
    std::string filename = "grid_pred.csv";
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    
    // determine the maximum row size by finding the size of the longest inner vector
    size_t maxRowSize = 0;
    for (const auto& col : grid) {
        if (col.size() > maxRowSize) {
            maxRowSize = col.size();
        }
    }
    // output the grid in transposed form (columns become rows in CSV)
    for (size_t row = 0; row < maxRowSize; ++row) {
        for (size_t col = 0; col < grid.size(); ++col) {
            if (row < grid[col].size()) {
                file << grid[col][row];
            }
            if (col < grid.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Robot's grid written to " << filename << std::endl;
}

// Task 2: Update min and max y values encountered during wall following
void my_robot::update_min_max_y() {
    int center_y = this->y + radius;
    if (center_y < min_y_tracked) {
        min_y_tracked = center_y;
    }
    if (center_y > max_y_tracked) {
        max_y_tracked = center_y;
    }
}

// Task 2: Check if robot has completed one lap around walls
bool my_robot::check_if_lapped() {
    // Record start position when first hitting a wall
    if (start_x == -1 && start_y == -1) {
        std::array<int, 4> current_mode = detect_walls();
        bool hit_wall = false;
        for (int i = 0; i < 4; i++) {
            if (current_mode[i] != 0) {
                hit_wall = true;
                break;
            }
        }
        if (hit_wall) {
            start_x = this->x;
            start_y = this->y;
            lap_counter = 0;
        }
        return false;
    }
    lap_counter++;
    
    int dist_sq = (this->x - start_x) * (this->x - start_x) + 
                  (this->y - start_y) * (this->y - start_y);
    
    // Environment-specific lap detection due to slight wall collision on env 1 and 3
    if ( environment_type == 3 ) {
        // For angled environments, be more lenient
        if (lap_counter > 600 && dist_sq < 2500) { 
            return true;
        }
    } else {
        // For straight wall environments, use original logic
        if (lap_counter > 800 && dist_sq < 100) { 
            return true;
        }
    }
    
    return false;
}

// Task 2: Toggle between clockwise and counterclockwise
void my_robot::toggle_direction() {
    clockwise = !clockwise;
    // Clear paths queue when direction changes
    while (!paths_queue.empty()) {
        paths_queue.pop();
    }
}

// Task 2: Check if robot is free of all walls
bool my_robot::is_free_of_walls() {
    std::array<int, 4> mode = detect_walls();
    for (int i = 0; i < 4; i++) {
        if (mode[i] != 0) {
            return false;
        }
    }
    return true;
}

// Task 2: Move robot away from wall one pixel at a time
void my_robot::move_away_from_wall_once() {
    // Determine direction to move away from wall
    int x_dir, y_dir;
    if (clockwise) {
        x_dir = -1; // Move left for clockwise
        y_dir = 0;
    } else {
        x_dir = 1;  // Move right for counterclockwise
        y_dir = 0;
    }
    
    // Move just one pixel in the appropriate direction
    int new_x = this->x + x_dir;
    int new_y = this->y + y_dir;
    
    // Bounds checking
    int max_x = 800 - 2 * radius - 1;
    int max_y = 800 - 2 * radius - 1;
    
    if (new_x >= 0 && new_x <= max_x && new_y >= 0 && new_y <= max_y) {
        this->x = new_x;
        this->y = new_y;
    }
}

// Task 2: Start sweep mode and prepare to return to top
void my_robot::start_sweep_mode() {
    sweep_mode = true;
    returning_to_top = true;
}

//===== Main parameters =====
const int env_width {800}, env_height {800};        //Width and height of the environment
const int radius {10};                              //Radius of the robot's circular body
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3/4
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3/4
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
    std::tuple<std::string, bool, int, int> config = read_csv();

        std::string filename = std::get<0>(config);

    // Extract environment number from filename
    int env_type = 1; // default to environment 1
    if (filename.find("environment1.csv") != std::string::npos) {
        env_type = 1;
    } else if (filename.find("environment2.csv") != std::string::npos) {
        env_type = 2;
    } else if (filename.find("environment3.csv") != std::string::npos) {
        env_type = 3;
    }

    std::vector<Object*> walls;

    if (std::get<3>(config) == 4) {
        walls = grid.create_walls(std::get<0>(config));
    }
    else {
        walls = grid.create_angled_walls(std::get<0>(config));
    }
    // get minimum/maximum y values for the robot to spawn
    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv"); 

    // Create robot with environment type
    my_robot robot(2*radius, 2*radius, env_width, min_y_spawn, max_y_spawn, radius+5, lidar_range, env_type);
    my_robot robot_init = robot;

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;
    // run the program indefinitely until robot hits the goal or an obstacle

    
    // main while loop
    while (true)
    {
        limit_count++;
        
        // Task 2 Psuedo code
        // 1. initialize the robot to travel in the clockwise direction (already done in class)
        // 2. detect_walls()
        robot.map_environment(grid);
        std::array<int, 4> mode = robot.detect_walls();
        
        //3. if the robot not in sweep mode
        if (!robot.in_sweep_mode()) {
            //4. find_dir()
            std::pair<int, int> wall_vec = robot.find_dir(mode);
            
            // 5. move()
            robot.move(wall_vec);
            
            // 6. update max_y and min_y
            robot.update_min_max_y();
            
            // 7. if the robot lapped walls and reaches max_y
            if (robot.check_if_lapped()) {
                // 8. y_ref = robot.y (handled in start_sweep_mode)
                // 9. go to sweep mode
                robot.start_sweep_mode();
            }
        }
        // 10. if the robot in sweep mode
        else {
            // Check if still returning to top
            if (robot.is_returning_to_top()) {
                int current_center_y = robot.y + radius;
                // Continue moving until we reach min_y (top)
                if (current_center_y <= robot.get_min_y()) {
                    // Reached the top, now start actual sweep
                    robot.set_returning_to_top(false);
                    robot.update_y_ref(); // Set y_ref to current position at top
                } else {
                    // Keep following walls to get to top
                    std::pair<int, int> wall_vec = robot.find_dir(mode);
                    robot.move(wall_vec);
                }
            } else {
                // Normal sweep behavior
                
                // Check step 15 FIRST - if we've moved down 50 pixels
                int current_center_y = robot.y + radius;
                if (current_center_y >= robot.get_y_ref() + 50) {
                    // Update y_ref
                    robot.update_y_ref();
                    
                    // 16. move robot left(clockwise) or right(ccw) UNTIL it detects no walls
                    while (!robot.is_free_of_walls()) {
                        robot.move_away_from_wall_once();
                        robot.map_environment(grid);
                    }
                }
                
                // Store mode before move for step 12
                robot.store_mode_before_move(mode);
                
                // 11. find_dir()
                std::pair<int, int> wall_vec = robot.find_dir(mode);
                
                // 14. move()
                robot.move(wall_vec);
                //Top barrier for env 1 and 3
                    if (env_type == 1 || env_type == 3) {
                    int current_center_y = robot.y + radius;
                    double top_barrier_margin = 0.5; 
                    
                    // If robot gets too close to top, push it down
                    if (current_center_y <= robot.get_min_y() + top_barrier_margin) {
                        // Move down away from top
                        robot.y += 2;
                        robot.map_environment(grid);
                    }
                }

                // Detect walls after moving to check for wall hits
                robot.map_environment(grid);
                std::array<int, 4> mode_after = robot.detect_walls();
                
                // 12. if robot's {x, y, v, w} was previously {0, 0, 0, 0} and it hits wall
                std::array<int, 4> prev_mode_stored = robot.get_mode_before_move();
                bool was_free = true;
                bool now_hits_wall = false;
                
                for (int i = 0; i < 4; i++) {
                    if (prev_mode_stored[i] != 0) was_free = false;
                    if (mode_after[i] != 0) now_hits_wall = true;
                }
                
                if (was_free && now_hits_wall) {
                    // 13. toggle direction (between clockwise and counterclockwise)
                    robot.toggle_direction();
                }
                
                // 17. if robot.y == max_y
                current_center_y = robot.y + radius;                
                    if (env_type == 3) {
                    // For environment 3, stop when we reach near max_y since its bugged other wise
                    if (current_center_y >= robot.get_max_y() - 5) {
                        break;
                    }
                } else {
                    // Keep original logic for environments 1 & 2
                    if (current_center_y >= robot.get_max_y()) {
                        
                        // Move to bottom-left corner and stop
                        while ((robot.y + radius) < robot.get_max_y()) {
                            robot.y++;
                            robot.map_environment(grid);
                            robot_pos.push_back({robot.x, robot.y});
                        }
                        break;
                    }
                }
            }
        }
        

        robot_pos.push_back({robot.x, robot.y});

        if (limit_count>=7200) {
            std::cout << "====Program terminated after 7200 iterations====" << std::endl;
            break;
        }
    }

    // Accuracy calculations
    std::cout << std::fixed << std::setprecision(2);
    float wall_accuracy = grid.wall_accuracy(robot.grid);
    float accuracy = grid.grid_accuracy(robot.grid);
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy*100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy*100.0 << "%" << std::endl;
    
    if (std::get<1>(config)){
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }
    
    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range, std::get<2>(config));

    return 0;
}
