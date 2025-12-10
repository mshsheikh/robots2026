<!--
Sync Impact Report:
Version change: 1.0.0 -> 1.1.0
Modified principles:
- Added Small, Token-Efficient Prompts
- Added Spec-Driven Pipeline
- Added Preserve History
- Added Use Feature Branches
- Added Avoid Committing Local Venv
Added sections: None
Removed sections: Deliver Working Product, Token-Efficient Generation, Reproducible Spec-Driven Pipeline, Minimal, Testable Scaffolds, Documented Artifact Paths
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Course Constitution

## Core Principles

### Small, Token-Efficient Prompts
Use small, token-efficient prompts and outputs to maintain efficiency and reduce costs.

### Spec-Driven Pipeline
Follow spec-driven pipeline: spec → plan → tasks → implement → phr for all development work.

### Preserve History
Preserve all history under history/prompts to maintain complete project traceability.

### Use Feature Branches
Use feature branches for implementation to maintain clean main branch and enable parallel development.

### Avoid Committing Local Venv
Avoid committing local venv; use requirements.txt instead to ensure reproducible environments.

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, migration plan when applicable.

All PRs/reviews must verify compliance with these principles; Complexity must be justified with clear benefits.

**Version**: 1.1.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-10