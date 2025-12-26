---
id: 007
title: "Expand Week 6 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 6 into four learner tracks"
labels: ["documentation", "curriculum", "manipulation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week6-manipulation-knowledge.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 6 into four learner tracks

  title: Expand Week 6 — Manipulation & Knowledge Representation
  target_path: book/docs/week6-manipulation-knowledge.md

  meta:
    week: 6
    title: "Manipulation & Knowledge Representation"
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
          - what manipulation means in robotics
          - hands vs grippers explained simply
          - object properties (shape, weight, fragility)
          - everyday examples (picking up a cup)
          - 3 conceptual activities
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early robotics learner
        include:
          - grasp types (power vs precision)
          - basic kinematics intuition
          - perception-to-grasp pipeline
          - one guided grasp-planning exercise
          - common mistakes
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - forward/inverse kinematics
          - grasp synthesis basics
          - symbolic vs geometric representations
          - task and motion planning (TAMP)
          - minimal fenced pseudo-code
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: system designer / lead
        include:
          - manipulation stack architecture
          - data-driven vs model-based trade-offs
          - reliability and failure modes
          - sim-to-real and dataset concerns
          - deployment checklist
          - learning objectives (4–6)

    - connect this week explicitly to Week 3 (Perception Systems)
    - use plain Markdown / MDX only (no JSX, no embedded HTML)
    - ensure SSG-safe formatting
    - do not modify any other files

  output:
    - updated week6-manipulation-knowledge.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 6 Manipulation & Knowledge Representation file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with conceptual explanations and everyday examples
  - Junior/Beginner: Early robotics learners with grasp types and kinematics intuition
  - Mid-Level Engineer: Practicing roboticists with forward/inverse kinematics and TAMP
  - Senior/Executive: System designers with architecture and deployment considerations

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 6 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 6 into Four Learner Tracks

## Summary
Updated the Week 6 Manipulation & Knowledge Representation file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week6-manipulation-knowledge.md`: Expanded with four learner tracks as specified