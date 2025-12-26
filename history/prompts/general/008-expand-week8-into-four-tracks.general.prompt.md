---
id: 009
title: "Expand Week 8 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-18"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 8 into four learner tracks"
labels: ["documentation", "curriculum", "ai-integration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week8-ai-integration.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 8 into four learner tracks

  title: Expand Week 8 — AI Integration in Humanoid Robotics
  target_path: book/docs/week8-ai-integration.md

  meta:
    week: 8
    title: "AI Integration in Humanoid Robotics"
    word_targets:
      newbie: 450
      beginner: 750
      mid: 1200
      senior: 850

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introduction
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: AI/robotics newcomer
        include:
          - what "AI inside a robot" actually means
          - perception → decision → action loop (intuitive explanation)
          - examples of AI in daily-life robots
          - 3 guided thought experiments
          - glossary (VLA, model, inference, policy)
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early AI/robotics learner
        include:
          - where ML models live in a robot stack
          - difference between rules, classical control, and learning
          - intro to Vision-Language-Action (VLA)
          - simple ROS 2 + AI pipeline diagram (described in text)
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: AI/robotics practitioner
        include:
          - integrating perception models with planners
          - latency, throughput, and hardware constraints
          - model deployment (edge vs cloud)
          - failure modes and fallbacks
          - minimal fenced pseudo-code for inference loop
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: system architect / decision maker
        include:
          - AI system architecture for humanoids
          - safety, verification, and monitoring
          - cost, scalability, and update strategy
          - evaluation metrics (task success, robustness)
          - build vs buy considerations
          - learning objectives (4–6)

    - connect this week to Week 7 (Hardware & Sensors) and Week 9 (Human-Robot Interaction)
    - use plain Markdown / MDX only (no JSX, no embedded HTML)
    - ensure SSG-safe formatting
    - do not modify any other files

  output:
    - updated week8-ai-integration.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 8 AI Integration in Humanoid Robotics file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: AI/robotics newcomers with intuitive explanations of AI in robots
  - Junior/Beginner: Early learners with ML model placement and VLA concepts
  - Mid-Level Engineer: Practitioners with integration patterns and deployment strategies
  - Senior/Executive: Architects with system architecture and evaluation metrics

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 8 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 8 into Four Learner Tracks

## Summary
Updated the Week 8 AI Integration in Humanoid Robotics file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week8-ai-integration.md`: Expanded with four learner tracks as specified