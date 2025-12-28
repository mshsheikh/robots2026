---
id: 014
title: "Expand Week 6 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 6 into four learner tracks"
labels: ["documentation", "curriculum", "manipulation-knowledge"]
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
        audience: complete beginner
        include:
          - what manipulation means in robotics
          - everyday examples (factory robots, service robots, etc.)
          - basic grasp types (power vs precision)
          - 3 hands-on activities (no coding)
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early robotics learner
        include:
          - kinematics intuition (forward vs inverse)
          - grasp planning basics
          - perception-action pipeline
          - one guided exercise
          - common pitfalls
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - mathematical kinematics (DH parameters, Jacobians)
          - grasp synthesis algorithms
          - TAMP (Task and Motion Planning)
          - minimal fenced code snippets
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: technical lead / architect
        include:
          - manipulation stack architecture
          - sim-to-real transfer strategies
          - deployment considerations
          - evaluation metrics
          - build vs buy decisions
          - learning objectives (4–6)

    - connect this week explicitly to Week 5 (Motion Planning) and Week 7 (Perception)
    - use plain Markdown / MDX only
    - ensure Docusaurus-safe formatting
    - do not modify any other files

  output:
    - updated week6-manipulation-knowledge.md
    - suggested git commit message
    - PHR title
response_text: |
  Updated the Week 6 Manipulation & Knowledge Representation file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuitive explanations of manipulation concepts
  - Junior/Beginner: Early learners with kinematics intuition and grasp planning
  - Mid-Level Engineer: Practitioners with mathematical formulations and algorithms
  - Senior/Executive: Architects with system architecture and deployment considerations

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