---
id: 008
title: "Expand Week 7 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 7 into four learner tracks"
labels: ["documentation", "curriculum", "humanoid-hardware"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week7-humanoid-hardware.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 7 into four learner tracks

  title: Expand Week 7 — Humanoid Hardware & Sensors
  target_path: book/docs/week7-humanoid-hardware.md

  meta:
    week: 7
    title: "Humanoid Hardware & Sensors"
    word_targets:
      newbie: 420
      beginner: 700
      mid: 1100
      senior: 800

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introduction
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: complete beginner
        include:
          - what makes a robot "humanoid"
          - main body parts (head, torso, arms, legs)
          - sensors explained intuitively (eyes, ears, balance)
          - difference between sensors and actuators
          - 3 observation activities
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early robotics learner
        include:
          - common humanoid platforms overview
          - motors vs servos
          - basic sensor types (IMU, camera, force/torque)
          - hardware ↔ ROS 2 data flow
          - simple hardware inspection checklist
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - actuator selection trade-offs
          - sensor fusion basics
          - wiring, buses (CAN, EtherCAT)
          - calibration procedures
          - minimal fenced pseudo-code (sensor node config)
          - 2 troubleshooting scenarios
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: system designer / lead
        include:
          - humanoid hardware architecture
          - cost, reliability, and maintainability
          - vendor and supply-chain considerations
          - safety and redundancy
          - deployment readiness checklist
          - learning objectives (4–6)

    - connect this week to Week 5 (Bipedal Locomotion) and Week 6 (Manipulation)
    - use plain Markdown / MDX only (no JSX, no embedded HTML)
    - ensure SSG-safe formatting
    - do not modify any other files

  output:
    - updated week7-humanoid-hardware.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 7 Humanoid Hardware & Sensors file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuitive explanations of humanoid form and sensors
  - Junior/Beginner: Early robotics learners with platform overviews and ROS 2 integration
  - Mid-Level Engineer: Practicing roboticists with actuator selection and sensor fusion
  - Senior/Executive: System designers with architecture and deployment considerations

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 7 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 7 into Four Learner Tracks

## Summary
Updated the Week 7 Humanoid Hardware & Sensors file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week7-humanoid-hardware.md`: Expanded with four learner tracks as specified