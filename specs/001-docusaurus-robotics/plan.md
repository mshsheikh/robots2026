# Implementation Plan: Physical AI & Humanoid Robotics Course

**Branch**: `001-docusaurus-robotics` | **Date**: 2025-12-10 | **Spec**: specs/001-docusaurus-robotics/spec.md
**Input**: Feature specification from `/specs/001-docusaurus-robotics/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Development of a Docusaurus-based textbook for Physical AI & Humanoid Robotics with integrated RAG chatbot. The system includes 13 weekly chapters (3 sections each ~180 words), FastAPI backend, Qdrant vector database, and GitHub Pages deployment.

## Technical Context

**Language/Version**: Python 3.12, TypeScript/JavaScript for frontend
**Primary Dependencies**: Docusaurus, FastAPI, Qdrant Client, uvicorn, React
**Storage**: Qdrant vector database for RAG, GitHub Pages for static content
**Testing**: pytest for backend, Jest for frontend
**Target Platform**: Web application (Linux server backend, browser frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <3 second response time for RAG queries, 99% GitHub Pages uptime
**Constraints**: <200ms p95 for API responses, token-efficient generation, selected-text-only chat mode
**Scale/Scope**: Educational content for course students, 52 sections (~9,360 words total)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Small, Token-Efficient Prompts: Following token-efficiency constraints from spec
- ✅ Spec-Driven Pipeline: Following spec → plan → tasks → implement → phr workflow
- ✅ Preserve History: All changes preserved under history/prompts
- ✅ Use Feature Branches: Using 001-docusaurus-robotics branch
- ✅ Avoid Committing Local Venv: Will use requirements.txt instead of committing venv

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

backend/
├── rag/
│   ├── fastapi/
│   │   └── main.py
│   └── qdrant/
│       └── ingest.py
├── scripts/
│   └── deploy_github_pages.sh
└── requirements.txt

frontend/
├── book/                # Docusaurus textbook site
│   ├── docs/
│   │   ├── intro.md
│   │   └── week1-ros2-basics.md
│   ├── src/
│   │   ├── components/
│   │   └── css/
│   ├── static/
│   └── docusaurus.config.js
└── code-to-humanoid/    # Existing Docusaurus site

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Milestones

milestone | due | est_hours | notes
Scaffold Docusaurus site | 2025-12-01 | 4 | Basic Docusaurus setup with GitHub Pages config
Create textbook content | 2025-12-03 | 16 | 13 weeks × 3 sections (~180w each)
Implement RAG backend | 2025-12-05 | 12 | FastAPI /ask endpoint with Qdrant integration
Build content ingestion | 2025-12-06 | 8 | Qdrant ingest pipeline for textbook content
Add client chat widget | 2025-12-07 | 10 | Selected-text-only mode widget
Deploy and test system | 2025-12-07 | 6 | Full integration and deployment to GitHub Pages
