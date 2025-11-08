# MTE301 Labs: Programming for Mechatronics

## Table of Contents

- [Workspace Setup](#workspace-setup)
- [Lab 1: Elementary Robotic Navigation](#lab-1-elementary-robotic-navigation)
- [Lab 2: Obstacle Avoidance](#lab-2-obstacle-avoidance)
- [Lab 3: Mapping and Wall Following](#lab-3-mapping-and-wall-following)
- [Lab 4: Mapping, Wall Following, and Sweeping](#lab-4-mapping-wall-following-and-sweeping)
- [Lab 5: Mapping, Wall Following, and Sweeping with Obstacles](#lab-5-mapping-wall-following-and-sweeping-with-obstacles)

## Workspace Setup

### Basic Setup
Follow the same steps discussed in the lab manuals. The process may vary depending on whether it's your first time cloning or you're re-cloning while preserving previous work.

**Clone the repository:**
```
git clone https://github.com/TakShimoda/MTE301_Lab
```

**Navigate and open in VS Code:**
```
Example for Lab 1:

cd ~/MTE301_Lab/Lab1  
code . 
```

### Extra Functionalities
- **Pause and Rewind/Fast Forward**: Press `p` to pause/unpause the SFML window
- **Frame Control**: While paused, use left/right arrow keys to rewind or fast-forward
- **Debugger**: Optional debugging setup available in the `VSCode_Files` folder

## Lab 1: Elementary Robotic Navigation

### Completed Tasks

**Task 1 - Wall Collision Detection**
- Robot moves upward until hitting top wall
- Exits with failure upon wall collision

**Task 2 - Goal Collision Detection** 
- Robot navigates to goal position
- Exits with success upon goal collision

**Task 3 & 4 - Navigation Methods**
- Horizontal-then-vertical and vertical-then-horizontal navigation
- Ensures robot reaches goal position efficiently

**Task 5 - Diagonal Motion Cases**
- **Case 1**: Diagonal movement when `1 < abs(del_y/del_x) < 2`
- **Case 2**: Diagonal movement when `1 < abs(del_x/del_y) < 2`
- Optimized path planning with pixel-perfect movement

**Task 6 - Fast Diagonal Motion**
- Rapid diagonal movement with ratio-based speed adjustment
- Final approach to goal center

**Task 7 - Shortest Path to Corner**
- Robot identifies and moves to nearest corner
- Optimized path selection algorithm

### Execution [Same For All Labs]
```
cd ~/MTE301_Lab/Lab(# of lab)
make lab(# of lab)
./lab(# of lab)
```

## Lab 2: Obstacle Avoidance

### Completed Tasks

**Task 3 (Bonus) - Optimal Path Obstacle Avoidance**
- Advanced obstacle avoidance algorithm
- Always results in shortest path to goal
- Intelligent path planning around obstacles

**Enhanced Features**
- Anti-stuck mechanisms for complex environments
- Robust navigation in obstacle-dense areas
- Efficient re-routing when blocked

## Lab 3: Mapping and Wall Following

### Completed Features
- Real-time environment mapping
- Wall following algorithm implementation
- Spatial awareness and boundary detection
- Continuous map updating during navigation

## Lab 4: Mapping, Wall Following, and Sweeping

### Completed Tasks

**Environment 1 - Tasks 1 & 2**
- Wall following while mapping environment
- Interior sweeping algorithm implementation
- Complete area coverage mapping

**Environment 2 - Task 1**
- Advanced wall following in complex layout
- Efficient boundary mapping
- Optimized navigation patterns

**Environment 2 - Task 2**
- Interior space sweeping after wall mapping
- Comprehensive area coverage
- Systematic exploration algorithm

**Environment 3 - Tasks 1 & 2**
- Combined wall following and interior sweeping
- Robust performance in varied environments
- Complete mapping solution

## Lab 5: Mapping, Wall Following, and Sweeping with Obstacles

### Current Progress

**Task 1 - Obstacle-Aware Sweeping**
- Interior environment sweeping with obstacle avoidance
- Maintains mapping functionality while navigating obstacles
- Adaptive path planning around obstructions

**Task 2 - Goal Navigation with Obstacles**
- Goal-oriented navigation with obstacle avoidance
- Efficient pathfinding in obstacle-populated environments
- Combines mapping with targeted movement

### Status: **IN PROGRESS**
- Basic functionality implemented
- Ongoing optimization and bug fixes
- Final testing and refinement needed

## Project Structure
```
MTE301_Lab/
├── Lab1/          # Elementary Navigation
├── Lab2/          # Obstacle Avoidance  
├── Lab3/          # Mapping & Wall Following
├── Lab4/          # Advanced Mapping & Sweeping
├── Lab5/          # Obstacle Integration (In Progress)
└── VSCode_Files/  # Debugging Configuration
```

## Technologies & Skills
- **C++ Programming** - Core algorithm implementation
- **SFML Graphics** - Visualization and simulation
- **Robotic Navigation** - Path planning and obstacle avoidance
- **Mapping Algorithms** - Environment representation
- **Debugging Tools** - VS Code integration and debugging

## Author
**coding-muggle** - MTE301 Programming for Mechatronics Labs

## Repository
[MTE301 Lab Repository](https://github.com/TakShimoda/MTE301_Lab)
