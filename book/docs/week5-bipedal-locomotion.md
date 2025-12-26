---
sidebar_position: 6
---

# Week 5: Bipedal Locomotion Principles

## Introduction to Humanoid Walking
This week covers the essential concepts of bipedal locomotion, which is fundamental to humanoid robot mobility.

- **Walking Gaits and Patterns**: Understanding the physics of bipedal walking, including single and double support phases, zero-moment point (ZMP) theory, and dynamic balance.

- **Balance Control**: Techniques for maintaining stability during walking, including feedback control, preview control, and recovery strategies for perturbations.

- **Terrain Adaptation**: Methods for adapting walking patterns to different surfaces and obstacles, including stair climbing, stepping over objects, and walking on uneven terrain.

This knowledge provides the foundation for developing stable and versatile walking capabilities in humanoid robots.

## Learning Paths by Experience Level

### Newbie in Tech
**Audience**: Complete beginner

#### Intuition: Why Walking is Hard for Robots
Walking is one of the most complex tasks for robots because it requires constant balance, coordination, and adaptation. Unlike wheeled robots that roll smoothly, walking robots must lift one leg at a time while keeping their body stable. This is like trying to balance on one foot while moving forward - challenging for humans, and even more so for machines that don't have the millions of years of evolution that humans have.

#### Balance, Falling, and Recovery Explained Simply
Balance is like a constant dance between falling and catching yourself. When you walk, you're actually falling forward slightly with each step, but your next foot placement catches you and propels you forward again. For robots, this requires sensors to detect when they're starting to fall and quick adjustments to prevent a complete fall.

#### Human Walking vs Robot Walking
Humans walk with a natural, fluid motion because our nervous system constantly makes tiny adjustments without us thinking about it. Robot walking often looks more rigid and calculated because each movement must be planned and controlled precisely. Humans can adapt to unexpected obstacles instantly, while robots need to sense, process, and plan their response.

#### Learning Objectives
- Understand why bipedal walking is a complex engineering challenge
- Recognize the difference between static and dynamic balance
- Appreciate the coordination required for stable walking
- Identify basic components of walking motion
- Compare human and robot locomotion approaches

#### Conceptual Activities
1. **Balance Challenge** (Time estimate: 20-25 minutes)
   - Stand on one foot and notice how your body makes tiny adjustments to maintain balance
   - Try to identify which parts of your body move to maintain stability
   - Compare standing on two feet versus one foot in terms of stability
   - Record observations about the complexity of human balance

2. **Walking Observation** (Time estimate: 25-30 minutes)
   - Watch videos of different robots walking (like Atlas, ASIMO, or ANYmal)
   - Compare their walking style to how humans walk
   - Note differences in fluidity, adaptability, and stability
   - Identify what makes robot walking look different from human walking

3. **Falling Prevention Analysis** (Time estimate: 30-35 minutes)
   - Think about what happens when you trip or slip
   - Consider the different strategies humans use to recover from stumbles
   - Compare this to what might happen to a robot in the same situation
   - Reflect on why fall recovery is such a critical capability for walking robots

#### Glossary of Key Terms
1. **Bipedal Locomotion**: The act of walking on two legs
2. **Center of Mass**: The point where the body's mass is concentrated, critical for balance
3. **Zero-Moment Point (ZMP)**: A point where the net moment of the ground reaction force is zero
4. **Gait**: The pattern of movement during walking
5. **Support Phase**: When one or both feet are in contact with the ground
6. **Swing Phase**: When a leg is moving forward during walking
7. **Stability**: The ability to maintain balance and avoid falling
8. **Dynamic Balance**: Balance maintained through movement rather than standing still

### Junior / Beginner
**Audience**: Early robotics learner

#### Learning Objectives
- Explain the difference between center of mass and center of pressure
- Describe the gait cycle phases (stance and swing)
- Understand the simplified concept of Zero-Moment Point (ZMP)
- Apply basic gait planning principles
- Identify common challenges in bipedal walking
- Connect walking concepts to control theory from Week 4

#### Center of Mass vs Center of Pressure
The center of mass (CoM) is the point where the robot's mass is balanced in all directions. For stable standing, the CoM must be positioned over the support base (the area between the feet). The center of pressure (CoP) is the point where the ground reaction force is applied - essentially where the robot's weight is concentrated on the ground. For stability, the CoP should remain within the foot's contact area.

#### Gait Cycles (Stance, Swing)
Walking consists of two main phases:
- **Stance Phase**: When the foot is in contact with the ground, supporting the robot's weight
- **Swing Phase**: When the leg is moving forward in preparation for the next step

A complete gait cycle includes both phases for one leg, typically beginning and ending with the same foot touching the ground.

#### Simplified ZMP Idea
The Zero-Moment Point is a critical concept in bipedal walking. For a robot to remain stable, the ZMP must remain within the support polygon (the area covered by the feet). If the ZMP moves outside this area, the robot will tip over. This is why walking robots must carefully plan their foot placements and CoM trajectories.

#### Guided Gait-Design Exercise
Consider the basic steps to design a simple walking gait:

```
# Basic gait planning algorithm
function planStep(current_state):
    # 1. Determine where to place the next foot
    next_foot_position = determineStepLocation(current_state)

    # 2. Plan CoM trajectory to maintain stability
    com_trajectory = planCoMTrajectory(current_state, next_foot_position)

    # 3. Ensure ZMP remains within support polygon
    if not zmpInSupportPolygon(com_trajectory, next_foot_position):
        adjustCoMTrajectory(com_trajectory)

    # 4. Generate joint angle trajectories
    joint_trajectories = inverseKinematics(com_trajectory, next_foot_position)

    return joint_trajectories
```

#### Common Mistakes
- Not accounting for the robot's momentum during walking
- Placing feet too close together, reducing stability margin
- Ignoring the timing of CoM movement relative to foot placement
- Forgetting that walking is a dynamic process, not static balance
- Overlooking the importance of preview control for stability

### Mid-Level Engineer
**Audience**: Practicing roboticist

#### Learning Objectives
- Apply ZMP formulation to walking stability analysis
- Implement inverted pendulum models for walking control
- Design stable trajectory planning algorithms
- Integrate feedback stabilization with trajectory planning
- Evaluate walking performance quantitatively
- Connect perception and control systems for walking
- Analyze walking dynamics for specific robot platforms

#### ZMP Formulation
The Zero-Moment Point (ZMP) formulation is based on the equation:

ZMP_x = x - (g/z_com) * (ẍ/ω²)

Where:
- x is the horizontal position of the center of mass
- z_com is the height of the center of mass
- g is gravitational acceleration
- ẍ is the horizontal acceleration of the center of mass
- ω² = g/z_com (natural frequency squared)

For stable walking, the ZMP must remain within the convex hull of the foot support polygon.

#### Inverted Pendulum Model
The linear inverted pendulum model (LIPM) simplifies the walking dynamics:

ẍ = ω²(x - zmp)

This model assumes constant CoM height and is useful for planning stable walking trajectories. The CoM trajectory follows a second-order system that can be solved analytically for given ZMP reference trajectories.

#### Trajectory Planning
Walking trajectories are typically planned in two phases:
1. **Preview Planning**: Calculate CoM and footstep trajectories based on desired walking speed and direction
2. **Real-time Adjustment**: Modify trajectories based on sensor feedback to maintain stability

#### Feedback Stabilization
Walking controllers typically use multiple feedback loops:
- High-level: Adjust footsteps based on CoM state
- Mid-level: Regulate ZMP position
- Low-level: Joint position/force control

#### Code Snippets
Inverted pendulum trajectory generation:
```python
import numpy as np

class InvertedPendulumTrajectory:
    def __init__(self, z_com_height, g=9.81):
        self.z_com_height = z_com_height
        self.g = g
        self.omega = np.sqrt(g / z_com_height)

    def compute_com_trajectory(self, zmp_trajectory, dt, duration):
        """
        Compute CoM trajectory from ZMP reference using LIPM
        """
        n_steps = int(duration / dt)
        com_x = np.zeros(n_steps)
        com_y = np.zeros(n_steps)
        com_z = np.full(n_steps, self.z_com_height)

        # Initial conditions
        com_x[0] = 0.0  # Starting CoM position
        com_y[0] = 0.0
        com_vel_x = 0.0  # Initial CoM velocity
        com_vel_y = 0.0

        for i in range(1, n_steps):
            # Update CoM position using LIPM dynamics
            # ẍ = ω²(x - zmp)
            acc_x = self.omega**2 * (com_x[i-1] - zmp_trajectory['x'][i-1])
            acc_y = self.omega**2 * (com_y[i-1] - zmp_trajectory['y'][i-1])

            # Integrate to get velocity and position
            com_vel_x += acc_x * dt
            com_vel_y += acc_y * dt

            com_x[i] = com_x[i-1] + com_vel_x * dt
            com_y[i] = com_y[i-1] + com_vel_y * dt

        return {
            'x': com_x,
            'y': com_y,
            'z': com_z
        }

    def compute_footstep_plan(self, start_pos, goal_pos, step_length=0.3, step_width=0.2):
        """
        Plan footsteps for walking from start to goal position
        """
        dx = goal_pos[0] - start_pos[0]
        dy = goal_pos[1] - start_pos[1]

        # Calculate number of steps needed
        distance = np.sqrt(dx**2 + dy**2)
        n_steps = int(np.ceil(distance / step_length))

        footsteps = []
        for i in range(n_steps):
            # Interpolate between start and goal
            ratio = (i + 1) / n_steps
            x = start_pos[0] + ratio * dx
            # Alternate feet to create walking pattern
            y_offset = step_width/2 if i % 2 == 0 else -step_width/2
            y = start_pos[1] + ratio * dy + y_offset

            footsteps.append([x, y])

        return footsteps

# Example usage
pendulum = InvertedPendulumTrajectory(z_com_height=0.8)  # 80cm CoM height
footsteps = pendulum.compute_footstep_plan([0, 0], [2.0, 0.1])
print(f"Planned {len(footsteps)} footsteps for walking 2m forward")
```

#### Challenge Problems
1. Implement a stabilizing controller that can handle external disturbances during walking by adjusting the ZMP reference in real-time
2. Design a transition controller that smoothly switches between different walking speeds while maintaining stability

### Senior / Executive
**Audience**: System designer / lead

#### Learning Objectives
- Evaluate locomotion stack architecture options for different robot platforms
- Assess trade-offs between efficiency, robustness, and hardware limitations
- Plan for sim-to-real transfer challenges in walking systems
- Design safety and fall mitigation strategies
- Establish deployment and validation protocols
- Balance computational requirements with performance goals

#### Locomotion Stack Architecture
A typical bipedal locomotion stack includes:
- **High-level Planning**: Path planning and global navigation
- **Gait Planning**: Step location and timing decisions
- **Trajectory Generation**: CoM and foot trajectory planning
- **Balance Control**: Real-time stabilization algorithms
- **Joint Control**: Low-level motor control and feedback

#### Trade-offs: Efficiency, Robustness, Hardware Limits
**Energy Efficiency vs Robustness**: More conservative walking patterns are more robust but less energy efficient. Dynamic walking is more efficient but requires more sophisticated control.

**Hardware Capabilities**: Joint torque limits, maximum speeds, and sensor accuracy all constrain walking performance. Walking controllers must respect these physical limitations.

**Real-time Requirements**: Walking control typically requires 100-1000 Hz control rates, constraining computational complexity.

#### Sim-to-Real Concerns
Simulation models never perfectly match reality. Key concerns include:
- Model inaccuracies (mass distribution, friction)
- Sensor noise and delay
- Actuator dynamics not captured in simulation
- Environmental factors (floor compliance, cable management)

#### Safety and Fall Mitigation
- **Prevention**: Robust control design and stability margins
- **Detection**: Early fall detection algorithms
- **Recovery**: Strategies to prevent falls when possible
- **Mitigation**: Safe fall strategies when prevention fails

#### Deployment Checklist
- [ ] **Stability Validation**: Has the walking system been tested under perturbations and disturbances?
- [ ] **Hardware Limits**: Are control commands within actuator capabilities (torque, speed, position)?
- [ ] **Real-time Performance**: Does the control system meet timing requirements consistently?
- [ ] **Sensor Reliability**: How does the system handle sensor failures or degraded performance?
- [ ] **Calibration Protocol**: Are there procedures for maintaining system calibration?
- [ ] **Safety Procedures**: What happens when the walking system fails or encounters unexpected conditions?
- [ ] **Testing Coverage**: Has the system been tested on various terrains and conditions?
- [ ] **Monitoring Tools**: Are there appropriate diagnostics for tracking walking performance?
