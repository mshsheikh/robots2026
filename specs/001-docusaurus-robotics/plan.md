# Implementation Plan: Physical AI & Humanoid Robotics Course

**Branch**: `001-docusaurus-robotics` | **Date**: 2025-12-08 | **Spec**: [specs/001-docusaurus-robotics/spec.md](specs/001-docusaurus-robotics/spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-robotics/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a Docusaurus-based textbook for Physical AI & Humanoid Robotics course with 13 weekly chapters (each with 3 sections of ~180 words), integrated RAG chatbot using FastAPI and Qdrant, and deployment to GitHub Pages.

## Technical Context

**Language/Version**: Python 3.11, Node.js 18+
**Primary Dependencies**: Docusaurus, FastAPI, Qdrant, Claude Code
**Storage**: GitHub Pages (static), Qdrant vector database
**Testing**: pytest for backend, Jest for frontend
**Target Platform**: Web (GitHub Pages + FastAPI server)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <3s response time for RAG queries, <2s page load times
**Constraints**: <200ms p95 for API responses, Claude Code integration for content generation
**Scale/Scope**: 1000 concurrent users, 52 course sections (~9,360 words total content)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Deliver Working Product: Plan delivers working Docusaurus book + embedded RAG by deadline
- ✅ Token-Efficient Generation: Sections ~180 words for token efficiency
- ✅ Reproducible Spec-Driven Pipeline: Maintains spec → plan → tasks → implement flow
- ✅ Minimal, Testable Scaffolds: Plan focuses on minimal viable implementation
- ✅ Documented Artifact Paths: Artifacts under documented paths (book/, rag/)

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-robotics/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book/
├── docs/                # Course content (13 weeks, 3 sections each)
│   ├── week1/
│   ├── week2/
│   └── ...
│   └── week13/
├── src/
│   └── components/      # Custom Docusaurus components
├── docusaurus.config.js
└── package.json

rag/
├── fastapi/
│   ├── main.py          # FastAPI RAG endpoint
│   └── models.py        # Data models
├── qdrant/
│   └── ingest.py        # Content ingestion script
└── client/
    └── chat-widget.js   # Client-side chat widget

specs/
└── 001-docusaurus-robotics/  # This feature's specs
```

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## 6-Week Milestone Plan

milestone | due | est_hours | notes
---|---|---|---
Docusaurus site setup and basic structure | Week 1 | 16 | Set up Docusaurus, basic navigation, theme configuration
Course content generation (13 weeks x 3 sections) | Week 2-3 | 40 | Generate 52 sections (~180 words each) using Claude Code
RAG backend (FastAPI + Qdrant integration) | Week 3-4 | 32 | Implement FastAPI endpoints, Qdrant vector storage, content ingestion
Client-side chat widget with selected-text mode | Week 4 | 24 | Create chat interface that works with selected text from course content
Integration and testing | Week 5 | 24 | Connect frontend to RAG backend, implement end-to-end functionality
Deployment to GitHub Pages | Week 6 | 8 | Configure GitHub Actions, deploy site with RAG endpoint
