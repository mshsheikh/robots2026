---
id: 006
title: "Expand Week 5 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 5 into four learner tracks"
labels: ["documentation", "curriculum", "bipedal-locomotion"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week5-bipedal-locomotion.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 5 into four learner tracks

  title: Expand Week 5 — Bipedal Locomotion
  target_path: book/docs/week5-bipedal-locomotion.md

  meta:
    week: 5
    title: "Bipedal Locomotion"
    word_targets:
      newbie: 420
      beginner: 700
      mid: 1050
      senior: 800

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introduction
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: complete beginner
        include:
          - intuition: why walking is hard for robots
          - balance, falling, and recovery explained simply
          - human walking vs robot walking
          - 3 conceptual activities (no math)
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early robotics learner
        include:
          - center of mass vs center of pressure
          - gait cycles (stance, swing)
          - simplified ZMP idea
          - one guided gait-design exercise
          - common mistakes
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - ZMP formulation
          - inverted pendulum model
          - trajectory planning
          - feedback stabilization
          - minimal fenced pseudo-code
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: system designer / lead
        include:
          - locomotion stack architecture
          - trade-offs: efficiency, robustness, hardware limits
          - sim-to-real concerns
          - safety and fall mitigation
          - deployment checklist
          - learning objectives (4–6)

    - connect this week explicitly to Week 4 (Control Theory)
    - use plain Markdown / MDX only (no JSX, no embedded HTML)
    - ensure SSG-safe formatting
    - do not modify any other files

  output:
    - updated week5-bipedal-locomotion.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 5 Bipedal Locomotion file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuition-first explanations and conceptual activities
  - Junior/Beginner: Early robotics learners with center of mass and ZMP concepts
  - Mid-Level Engineer: Practicing roboticists with ZMP formulation and inverted pendulum models
  - Senior/Executive: System designers with architecture and deployment considerations

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 5 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 5 into Four Learner Tracks

## Summary
Updated the Week 5 Bipedal Locomotion file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week5-bipedal-locomotion.md`: Expanded with four learner tracks as specified