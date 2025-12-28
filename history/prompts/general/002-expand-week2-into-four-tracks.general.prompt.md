---
id: 002
title: "Expand Week 2 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 2 into four learner tracks"
labels: ["documentation", "curriculum", "motion-planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week2-motion-planning.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 2 into four learner tracks

  title: Expand Week 2 — Motion Planning
  target_path: book/docs/week2-motion-planning.md

  meta:
    week: 2
    title: "Motion Planning"
    word_targets:
      newbie: 400
      beginner: 600
      mid: 900
      senior: 700

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and intro content
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: complete beginner
        include:
          - intuition-first explanation of motion planning
          - configuration space explained with real-world analogies
          - 3 guided activities (no coding required)
          - glossary of 6 key terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early engineer
        include:
          - grid-based planning concepts
          - A* planner explained visually
          - 1 small hands-on exercise (pseudo-code allowed)
          - common pitfalls
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - sampling-based planners (RRT, PRM)
          - ROS 2 + MoveIt planning overview
          - minimal fenced code snippets
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: technical lead / architect
        include:
          - planner selection trade-offs
          - system architecture diagram explanation (text only)
          - safety, latency, and evaluation metrics
          - deployment checklist
          - learning objectives (4–6)

    - use plain Markdown / MDX (no JSX expressions)
    - keep headings consistent and SSG-safe
    - do not modify other weeks

  output:
    - updated week2-motion-planning.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 2 Motion Planning file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuition-first explanations and real-world analogies
  - Junior/Beginner: Early engineers with grid-based planning and A* algorithm concepts
  - Mid-Level Engineer: Practicing roboticists with sampling-based planners and ROS 2/MoveIt
  - Senior/Executive: Technical leads with system architecture and evaluation metrics

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 2 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 2 into Four Learner Tracks

## Summary
Updated the Week 2 Motion Planning file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week2-motion-planning.md`: Expanded with four learner tracks as specified