---
id: 005
title: "Expand Week 4 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-17"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 4 into four learner tracks"
labels: ["documentation", "curriculum", "control-theory"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week4-control-theory.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 4 into four learner tracks

  title: Expand Week 4 — Control Theory
  target_path: book/docs/week4-control-theory.md

  meta:
    week: 4
    title: "Control Theory"
    word_targets:
      newbie: 380
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
          - intuition-first explanation of control (feedback, correction)
          - everyday examples (thermostat, cruise control, balance)
          - block diagram explained in words
          - 3 non-math activities
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early engineer
        include:
          - open-loop vs closed-loop systems
          - PID intuition (no equations first)
          - simple tuning ideas
          - one guided simulation-style exercise
          - common pitfalls
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: practicing roboticist
        include:
          - mathematical PID formulation
          - stability intuition
          - discrete-time control
          - minimal fenced pseudo-code
          - 2 challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: technical lead / architect
        include:
          - control architecture choices (PID vs MPC vs learning-based)
          - trade-offs: stability, latency, safety
          - validation & testing strategy
          - deployment checklist
          - learning objectives (4–6)

    - ensure content builds on Week 3 (Perception → Control loop)
    - use plain Markdown / MDX (no JSX expressions)
    - keep headings consistent and SSG-safe
    - do not modify other weeks

  output:
    - updated week4-control-theory.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 4 Control Theory file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with intuition-first explanations and everyday examples
  - Junior/Beginner: Early engineers with open/closed-loop concepts and PID intuition
  - Mid-Level Engineer: Practicing roboticists with mathematical formulations and discrete control
  - Senior/Executive: Technical leads with architecture choices and validation strategies

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 4 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 4 into Four Learner Tracks

## Summary
Updated the Week 4 Control Theory file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week4-control-theory.md`: Expanded with four learner tracks as specified