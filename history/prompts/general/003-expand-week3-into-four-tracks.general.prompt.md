---
id: 004
title: "Expand Week 3 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 3 into four learner tracks"
labels: ["documentation", "curriculum", "perception"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week3-perception-systems.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 3 into four learner tracks

  title: Expand Week 3 — Perception Systems
  target_path: book/docs/week3-perception-systems.md

  meta:
    week: 3
    title: "Perception Systems"
    word_targets:
      newbie: 400
      beginner: 650
      mid: 950
      senior: 750

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introductory content
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: complete beginner
        include:
          - intuition-first explanation of robot perception
          - difference between sensing vs understanding
          - examples using human senses as analogy
          - 3 hands-on observation activities (no code)
          - glossary of 6–8 key terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early engineer
        include:
          - camera, LiDAR, IMU overview
          - basic perception pipeline (sense → preprocess → detect)
          - simple computer vision concepts (edges, features)
          - one guided mini-project (pseudo-code allowed)
          - common pitfalls
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - sensor fusion concepts
          - classical vs deep learning perception
          - ROS 2 perception stack overview
          - minimal fenced code snippets
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: technical lead / architect
        include:
          - perception system architecture trade-offs
          - latency, accuracy, robustness metrics
          - dataset strategy and evaluation
          - deployment checklist
          - learning objectives (4–6)

    - ensure content scales logically from Week 2 (Motion Planning)
    - use plain Markdown / MDX (no JSX expressions)
    - keep headings consistent and SSG-safe
    - do not modify other weeks

  output:
    - updated week3-perception-systems.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 3 Perception Systems file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuition-first explanations and human sense analogies
  - Junior/Beginner: Early engineers with sensor overviews and perception pipeline concepts
  - Mid-Level Engineer: Practicing roboticists with sensor fusion and ROS 2 perception
  - Senior/Executive: Technical leads with system architecture and evaluation metrics

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 3 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 3 into Four Learner Tracks

## Summary
Updated the Week 3 Perception Systems file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week3-perception-systems.md`: Expanded with four learner tracks as specified