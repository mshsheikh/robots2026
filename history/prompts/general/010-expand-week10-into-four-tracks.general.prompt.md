---
id: 011
title: "Expand Week 10 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-18"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 10 into four learner tracks"
labels: ["documentation", "curriculum", "ethics-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week10-ethics-robotics.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus: Expand Week 10 into four learner tracks

  title: Expand Week 10 — Ethics in Robotics
  target_path: book/docs/week10-ethics-robotics.md

  meta:
    week: 10
    title: "Ethics, Safety, and Responsibility in Robotics"
    word_targets:
      newbie: 400
      beginner: 650
      mid: 1000
      senior: 750

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter, headings, and introductory text
    - do NOT rename the file
    - append a new top-level section titled "Learning Paths by Experience Level"

    - inside that section, create exactly four subsections:

      - "Newbie in Tech"
        include:
          - what ethics means in robotics
          - simple real-world examples (delivery robots, humanoids, drones)
          - safety vs intelligence
          - everyday analogies
          - 3 reflection activities
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        include:
          - common ethical risks in robotics
          - bias, privacy, and safety
          - rule-based vs learned behavior
          - one short case study
          - common misconceptions
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        include:
          - ethical failure modes
          - human-in-the-loop design
          - safety constraints in AI systems
          - regulation awareness
          - minimal fenced examples (conceptual, not legal)
          - 2 discussion challenges
          - learning objectives (5–7)

      - "Senior / Executive"
        include:
          - governance and compliance
          - deployment risk assessment
          - organizational responsibility
          - public trust considerations
          - decision checklist
          - learning objectives (4–6)

    - explicitly connect ethics to earlier weeks (AI, control, HRI)
    - use plain Markdown / MDX only
    - ensure Docusaurus-safe formatting
    - do not modify any other files

  output:
    - updated week10-ethics-robotics.md
    - suggested git commit message
    - PHR title
response_text: |
  Updated the Week 10 Ethics and Safety in Robotics file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with simple explanations of ethics in robotics
  - Junior/Beginner: Early learners with ethical risks and case studies
  - Mid-Level Engineer: Practitioners with ethical failure modes and constraint systems
  - Senior/Executive: Decision makers with governance and deployment risk assessment

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 10 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 10 into Four Learner Tracks

## Summary
Updated the Week 10 Ethics and Safety in Robotics file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week10-ethics-robotics.md`: Expanded with four learner tracks as specified