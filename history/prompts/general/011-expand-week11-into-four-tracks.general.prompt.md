---
id: 012
title: "Expand Week 11 into Four Learner Tracks"
stage: "misc"
date_iso: "2025-12-18"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "claude"
command: "expand Week 11 into four learner tracks"
labels: ["documentation", "curriculum", "applications-domains"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - book/docs/week11-applications-domains.md
tests_yaml: |
  - none
prompt_text: |
  # Spec-Kit Plus: Expand Week 11 into four learner tracks

  title: Expand Week 11 — Applications & Domains
  target_path: book/docs/week11-applications-domains.md

  meta:
    week: 11
    title: "Applications and Domains of Robotics"
    word_targets:
      newbie: 420
      beginner: 700
      mid: 1100
      senior: 800

  instructions:
    - update the existing markdown file at target_path
    - preserve existing frontmatter, headings, and introductory content
    - do NOT rename the file
    - append a new top-level section titled "Learning Paths by Experience Level"

    - create exactly four subsections:

      - "Newbie in Tech"
        include:
          - what robotics applications mean
          - everyday examples (factories, hospitals, homes)
          - robots vs traditional machines
          - 3 domain exploration activities
          - glossary of 6–8 terms
          - learning objectives (3–5)

      - "Junior / Beginner"
        include:
          - major robotics domains (industrial, medical, service, logistics)
          - typical sensors and actuators per domain
          - strengths and limitations of robots in each area
          - one guided comparison exercise
          - learning objectives (4–6)

      - "Mid-Level Engineer"
        include:
          - system requirements by domain
          - autonomy levels and constraints
          - domain-specific failure modes
          - perception and control trade-offs
          - minimal fenced pseudo-code or system flow
          - 2 applied challenge problems
          - learning objectives (5–7)

      - "Senior / Executive"
        include:
          - market segmentation
          - deployment economics
          - scalability and maintenance
          - regulatory and safety considerations
          - ROI and adoption checklist
          - learning objectives (4–6)

    - explicitly connect this week to Weeks 5 (Locomotion), 6 (Manipulation), and 9 (HRI)
    - use plain Markdown / MDX only
    - ensure Docusaurus-safe formatting
    - do not modify any other files

  output:
    - updated week11-applications-domains.md
    - suggested git commit message
    - PHR title
response_text: |
  Updated the Week 11 Applications and Domains file to include four distinct learning tracks tailored to different experience levels:
  - Newbie in Tech: Complete beginners with simple explanations of robotics applications
  - Junior/Beginner: Early learners with domain comparisons and exercises
  - Mid-Level Engineer: Practitioners with system requirements and technical challenges
  - Senior/Executive: Decision makers with market analysis and economic considerations

  All existing content was preserved while adding the new structured learning paths with appropriate content and tone for each audience.
outcome: |
  Successfully expanded Week 11 content into four learner tracks as specified, with appropriate content depth and tone for each experience level.
evaluation: |
  The changes meet all requirements in the spec: existing content preserved, new sections added with appropriate content for each audience, and Docusaurus-safe Markdown formatting used throughout.
---

# Expand Week 11 into Four Learner Tracks

## Summary
Updated the Week 11 Applications and Domains file to include four distinct learning tracks tailored to different experience levels.

## Changes Made
- Preserved existing frontmatter and introductory content
- Added "Learning Paths by Experience Level" section
- Created four subsections: Newbie in Tech, Junior/Beginner, Mid-Level Engineer, and Senior/Executive
- Each track includes content appropriate to the experience level with varying degrees of depth and complexity

## Files Modified
- `book/docs/week11-applications-domains.md`: Expanded with four learner tracks as specified