---
id: 001
title: "Expand Week 1 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 1 into four learner tracks"
labels: ["documentation", "curriculum", "ros2"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week1-ros2-basics.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 1 into four learner tracks

  title: Expand Week 1 — ROS 2 Basics
  target_path: book/docs/week1-ros2-basics.md

  meta:
    week: 1
    title: "ROS 2 Basics"
    word_targets:
      newbie: 450
      beginner: 650
      mid: 1000
      senior: 800

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introductory content
    - append a new section titled "Learning Paths by Experience Level"
    - under that section, create 4 subsections:

      - "Newbie in Tech"
        audience: complete newcomer
        include:
          - short plain-English overview
          - 3 small guided activities (with time estimates)
          - glossary of 5 key terms
          - learning objectives (3–5 bullets)

      - "Junior / Beginner"
        include:
          - guided tutorial
          - small hands-on ROS 2 project
          - common pitfalls
          - learning objectives (4–6 bullets)

      - "Mid-Level Engineer"
        include:
          - deeper architectural concepts
          - minimal code snippets (fenced)
          - challenge problems
          - recommended readings

      - "Senior / Executive"
        include:
          - system-level perspective
          - design & evaluation checklist
          - real-world trade-offs

    - adapt tone per audience (supportive → concise)
    - avoid JSX, React, or unescaped braces
    - ensure MDX/Markdown is Docusaurus-safe
    - do not modify any other files

  outputs:
    - modified week1 markdown file
    - commit message suggestion
    - PHR title
response_text: |
  Updated the Week 1 ROS 2 Basics file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete newcomers with plain-English explanations, guided activities, and glossary
  - Junior/Beginner: Developers with programming experience, including tutorials and common pitfalls
  - Mid-Level Engineer: Experienced engineers with deeper architectural concepts and code snippets
  - Senior/Executive: Leadership with system-level perspective and evaluation checklists

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 1 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 1 into Four Learner Tracks

## Summary
Updated the Week 1 ROS 2 Basics file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week1-ros2-basics.md`: Expanded with four learner tracks as specified