---
id: 010
title: "Expand Week 9 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-18"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 9 into four learner tracks"
labels: ["documentation", "curriculum", "human-robot-interaction"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week9-human-robot-interaction.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus spec: expand Week 9 into four learner tracks

  title: Expand Week 9 — Human-Robot Interaction (HRI)
  target_path: book/docs/week9-human-robot-interaction.md

  meta:
    week: 9
    title: "Human-Robot Interaction"
    word_targets:
      newbie: 450
      beginner: 700
      mid: 1100
      senior: 800

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter and introduction
    - append a new section titled "Learning Paths by Experience Level"

    - create four subsections:

      - "Newbie in Tech"
        audience: non-technical learner
        include:
          - what HRI means in simple terms
          - everyday examples (voice assistants, service robots)
          - trust, comfort, and predictability
          - 3 observation-based activities
          - glossary (intent, autonomy, feedback)
          - learning objectives (3–5)

      - "Junior / Beginner"
        audience: early robotics learner
        include:
          - basic HRI components (input, output, feedback loop)
          - sensors used for HRI (camera, mic, touch)
          - simple state machines for interaction
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        audience: robotics / AI engineer
        include:
          - multimodal interaction (speech, vision, gesture)
          - intent recognition and ambiguity handling
          - latency and robustness issues
          - error recovery strategies
          - learning objectives (5–7)

      - "Senior / Executive"
        audience: product owner / system architect
        include:
          - UX vs safety tradeoffs
          - social acceptance and regulation
          - deployment in public vs private spaces
          - evaluation metrics (trust, task success)
          - learning objectives (4–6)

    - connect this week to Week 8 (AI Integration) and Week 10 (Ethics & Safety)
    - use plain Markdown / MDX only (no JSX, no embedded HTML)
    - ensure SSG-safe formatting
    - do not modify any other files

  output:
    - updated week9-human-robot-interaction.md
    - suggested commit message
    - PHR title
response_text: |
  Updated the Week 9 Human-Robot Interaction file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Non-technical learners with simple explanations of HRI concepts
  - Junior/Beginner: Early learners with basic HRI components and state machines
  - Mid-Level Engineer: Robotics/AI engineers with multimodal interaction and error handling
  - Senior/Executive: Product owners with UX/safety tradeoffs and deployment considerations

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 9 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 9 into Four Learner Tracks

## Summary
Updated the Week 9 Human-Robot Interaction file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week9-human-robot-interaction.md`: Expanded with four learner tracks as specified