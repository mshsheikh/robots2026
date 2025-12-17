---
sidebar_position: 3
---

# Week 2: Motion Planning Fundamentals

## Introduction to Motion Planning
This week covers the essential concepts of motion planning, which is critical for humanoid robots to navigate complex environments and perform tasks safely.

- **Configuration Space**: Understanding C-space, obstacles, and pathfinding algorithms - the mathematical foundations for planning robot movements in constrained environments.

- **Path Planning Algorithms**: Exploring A*, RRT, and PRM algorithms for finding collision-free paths, with practical implementation considerations for real-time robotics applications.

- **Trajectory Generation**: Creating smooth, dynamically feasible trajectories from planned paths, considering robot dynamics and environmental constraints for safe execution.

This foundational knowledge provides the tools to develop navigation systems for humanoid robots in both structured and unstructured environments.

## Learning Paths by Experience Level

### Newbie in Tech
**Audience**: Complete beginner

#### Intuition-First Explanation of Motion Planning
Motion planning is like giving a robot the ability to figure out how to move from one place to another without bumping into things. Think of it as the robot's GPS system, but instead of just finding directions on a map, it has to consider the robot's size, shape, and the obstacles around it. Just like how you might plan your path through a crowded room to avoid bumping into people, a robot needs to plan its movements to avoid hitting walls, furniture, or other objects.

#### Configuration Space Explained with Real-World Analogies
Imagine you're trying to move a large piece of furniture through your house. The configuration space is like a map that shows all the possible positions and orientations the furniture can have without getting stuck. If you're moving a couch, the configuration space would show every way the couch can be positioned and rotated while still fitting through doorways and around corners. For a robot, the configuration space represents all the possible joint angles and positions that don't result in collisions.

#### Learning Objectives
- Understand the basic concept of motion planning and why robots need it
- Grasp the idea of configuration space using familiar analogies
- Recognize how obstacles affect a robot's movement possibilities
- Appreciate the complexity of planning for robots with multiple joints
- Identify everyday examples of motion planning in human activities

#### Guided Activities
1. **Map Out Your Room's Configuration Space** (Time estimate: 15-20 minutes)
   - Draw a simple map of your room on paper
   - Mark where furniture and obstacles are located
   - Choose a simple object (like a box) to "move" through the space
   - Mark all the places this object could fit without hitting obstacles

2. **Analyze Human Motion Planning** (Time estimate: 20-25 minutes)
   - Observe how people naturally plan paths when moving through crowded spaces
   - Notice how people adjust their movements to avoid collisions
   - Think about how this might be similar to or different from robot motion planning
   - Consider how a robot might need to plan more carefully than humans

3. **Research Motion Planning in Daily Life** (Time estimate: 25-30 minutes)
   - Look up examples of motion planning in everyday technology (GPS, video games, etc.)
   - Find one example of motion planning in nature (animals navigating obstacles)
   - Compare how these different systems approach the same problem
   - Consider what makes robot motion planning unique

#### Glossary of Key Terms
1. **Motion Planning**: The process of finding a safe, collision-free path for a robot to move from one configuration to another
2. **Configuration Space (C-space)**: The mathematical space representing all possible positions and orientations of a robot
3. **Path Planning**: The process of finding a sequence of positions that connect a start and goal location
4. **Trajectory**: A path with timing information, specifying how the robot should move over time
5. **Obstacle**: Any object or area that the robot must avoid during movement
6. **Collision-Free**: A path that does not result in the robot intersecting with any obstacles

### Junior / Beginner
**Audience**: Early engineer

#### Learning Objectives
- Understand the principles of grid-based planning
- Implement basic pathfinding algorithms like A*
- Apply motion planning concepts to simple robotics scenarios
- Analyze the trade-offs between different planning approaches
- Identify common implementation challenges
- Use visualization tools to understand planning results

#### Grid-Based Planning Concepts
Grid-based planning divides the environment into a discrete grid of cells, where each cell represents a possible location for the robot. This approach simplifies the continuous space into a discrete graph that can be searched using classical algorithms. The resolution of the grid affects both the accuracy of the solution and the computational complexity - finer grids provide more precise paths but require more computation time.

In grid-based planning, each cell can be either free (traversable) or occupied (contains an obstacle). The planning algorithm searches for a path from the start cell to the goal cell by moving between adjacent free cells. Common movement patterns include 4-connected (up, down, left, right) or 8-connected (including diagonals) neighborhoods.

#### A* Planner Explained Visually
The A* algorithm is a graph traversal algorithm that finds the shortest path between a start and goal node. It uses a heuristic function to guide the search toward the goal, making it more efficient than uninformed search algorithms like Dijkstra's. The algorithm maintains two values for each node:
- g(n): the actual cost from the start node to the current node
- h(n): the estimated cost from the current node to the goal (heuristic)

The algorithm selects nodes to expand based on f(n) = g(n) + h(n), ensuring optimal paths while prioritizing nodes that appear closer to the goal according to the heuristic.

#### Small Hands-On Exercise
Design a simple grid-based pathfinding algorithm using A* for a 2D environment:

```
# Pseudocode for A* algorithm
function AStar(start, goal, grid):
    open_set = PriorityQueue()
    open_set.add(start)
    came_from = {}

    g_score = {node: infinity for node in grid}
    g_score[start] = 0

    f_score = {node: infinity for node in grid}
    f_score[start] = heuristic(start, goal)

    while open_set is not empty:
        current = open_set.get_min_f_score_node()

        if current == goal:
            return reconstruct_path(came_from, current)

        open_set.remove(current)

        for neighbor in current.neighbors():
            if neighbor is not in grid or is_occupied(neighbor):
                continue

            tentative_g_score = g_score[current] + distance(current, neighbor)

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

                if neighbor not in open_set:
                    open_set.add(neighbor)

    return failure  # No path found
```

#### Common Pitfalls
- Using an inadmissible heuristic (one that overestimates the actual cost) which can lead to suboptimal solutions
- Not properly handling the grid resolution - too coarse may miss valid paths, too fine may be computationally expensive
- Forgetting to check for obstacles when expanding nodes
- Not considering the robot's size when determining if a cell is traversable
- Incorrectly calculating distances or using inconsistent units in the heuristic function

### Mid-Level Engineer
**Audience**: Practicing roboticist

#### Learning Objectives
- Implement sampling-based planners for high-dimensional spaces
- Integrate motion planning with ROS 2 and MoveIt
- Analyze the computational complexity of different planning approaches
- Design custom state spaces and distance metrics for specific robots
- Optimize planning algorithms for real-time performance
- Evaluate planning success rates and solution quality
- Handle dynamic environments and replanning scenarios

#### Sampling-Based Planners (RRT, PRM)
Sampling-based planners are designed to handle high-dimensional configuration spaces where grid-based approaches become computationally infeasible. These methods randomly sample the configuration space and build a graph or tree structure to represent possible paths.

Rapidly-exploring Random Trees (RRT) incrementally build a tree of feasible configurations by randomly sampling the space and extending the tree toward these samples. The algorithm starts from the initial configuration and repeatedly:
1. Samples a random configuration in the space
2. Finds the nearest node in the existing tree
3. Extends from this node toward the random sample
4. Checks for collisions along the extension
5. Adds the new valid configuration to the tree

Probabilistic Roadmap (PRM) pre-processes the configuration space by randomly sampling configurations and connecting nearby samples to form a roadmap. This approach is particularly effective for environments with many obstacles where pre-computation can be leveraged.

#### ROS 2 + MoveIt Planning Overview
MoveIt is the de facto motion planning framework for ROS 2, providing a comprehensive set of tools for motion planning, inverse kinematics, collision checking, and trajectory execution. The planning pipeline typically involves:

1. Setting up the planning scene with robot state and environment obstacles
2. Defining the planning request with start state and goal constraints
3. Selecting appropriate planning algorithms and parameters
4. Executing the planner and validating the solution
5. Smoothing and optimizing the resulting trajectory
6. Executing the trajectory on the robot

MoveIt supports multiple planning backends including OMPL (Open Motion Planning Library), CHOMP (Covariant Hamiltonian Optimization for Motion Planning), and STOMP (Stochastic Trajectory Optimization).

#### Code Snippets
Basic MoveIt motion planning implementation:
```cpp
// Initialize MoveIt interface
moveit::planning_interface::MoveGroupInterface move_group("arm");
moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

// Set target pose
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.28;
target_pose.position.y = -0.7;
target_pose.position.z = 1.0;
target_pose.orientation.w = 1.0;

move_group.setPoseTarget(target_pose);

// Plan and execute
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

if(success) {
    move_group.execute(my_plan);
}
```

RRT implementation example:
```cpp
class RRTPlanner {
private:
    std::vector<Node> tree;
    double step_size;

public:
    Node* findNearestNode(const Node& sample) {
        Node* nearest = &tree[0];
        double min_dist = distance(*nearest, sample);

        for(auto& node : tree) {
            double dist = distance(node, sample);
            if(dist < min_dist) {
                min_dist = dist;
                nearest = &node;
            }
        }
        return nearest;
    }

    bool extendTree(const Node& random_node) {
        Node* nearest = findNearestNode(random_node);
        Node new_node = extendToward(*nearest, random_node, step_size);

        if(!isInCollision(new_node) && isValidExtension(*nearest, new_node)) {
            tree.push_back(new_node);
            new_node.parent = nearest;
            return true;
        }
        return false;
    }
};
```

#### Challenge Problems
1. Implement a bidirectional RRT that grows trees from both start and goal configurations, connecting them when they come close enough
2. Design a dynamic replanning system that can update the motion plan when new obstacles are detected during execution
3. Create a hybrid planning approach that combines sampling-based methods for global planning with optimization-based methods for local refinement

### Senior / Executive
**Audience**: Technical lead / architect

#### Learning Objectives
- Evaluate different motion planning approaches for specific use cases
- Design scalable motion planning architectures for robot fleets
- Establish performance metrics and success criteria for planning systems
- Balance computational requirements against real-time constraints
- Plan for safety and reliability in motion planning systems
- Assess the impact of motion planning on overall system architecture

#### Planner Selection Trade-offs
Different motion planning algorithms have distinct advantages and limitations that make them suitable for specific scenarios:

**Grid-based planners** excel in 2D navigation tasks with static environments, offering guaranteed optimal solutions and simple implementation. However, they scale poorly to high-dimensional spaces and require careful resolution selection.

**Sampling-based planners** (RRT, PRM) handle high-dimensional spaces effectively and can accommodate complex kinematic constraints, but provide no optimality guarantees and may have variable execution times.

**Optimization-based planners** (CHOMP, STOMP) generate smooth, dynamically feasible trajectories but require good initialization and may converge to local minima.

**Learning-based planners** can adapt to complex environments but require extensive training and may lack safety guarantees.

#### System Architecture Diagram Explanation (Text Only)
The motion planning system architecture typically follows a layered approach:

1. **Sensor Interface Layer**: Processes raw sensor data (LiDAR, cameras, etc.) to create environment representations
2. **Scene Understanding**: Interprets sensor data to identify static and dynamic obstacles
3. **Planning Request Handler**: Interprets high-level navigation goals and constraints
4. **Motion Planner**: Executes selected planning algorithm based on task requirements
5. **Trajectory Optimizer**: Refines planned paths for smoothness and dynamic feasibility
6. **Execution Monitor**: Tracks execution progress and triggers replanning when needed
7. **Feedback Loop**: Provides success/failure metrics to improve future planning

This architecture allows for modularity and flexibility in algorithm selection while maintaining consistent interfaces.

#### Safety, Latency, and Evaluation Metrics
**Safety Metrics:**
- Collision avoidance rate (target: >99.99%)
- Minimum distance maintained from obstacles
- Recovery behavior from planning failures
- Graceful degradation in uncertain environments

**Latency Metrics:**
- Planning time (typically <100ms for real-time applications)
- Replanning frequency in dynamic environments
- Trajectory update latency
- System response to emergency stops

**Evaluation Metrics:**
- Path optimality ratio (actual path length vs. theoretical optimum)
- Success rate in various environment types
- Computational resource utilization
- Solution smoothness and dynamic feasibility

#### Deployment Checklist
- [ ] **Algorithm Selection**: Is the chosen planner appropriate for the robot's DOF and environment complexity?
- [ ] **Safety Validation**: Have safety margins and emergency procedures been thoroughly tested?
- [ ] **Performance Requirements**: Does the planner meet real-time constraints under worst-case conditions?
- [ ] **Environment Modeling**: Are static and dynamic obstacles properly represented in the planning scene?
- [ ] **Fallback Mechanisms**: Are there appropriate recovery behaviors for planning failures?
- [ ] **Resource Constraints**: Does the planning system fit within computational and memory limits?
- [ ] **Testing Coverage**: Have edge cases and failure modes been adequately tested?
- [ ] **Integration Points**: Are interfaces with perception and control systems properly defined?