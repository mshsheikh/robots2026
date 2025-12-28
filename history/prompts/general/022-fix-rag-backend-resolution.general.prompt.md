---
id: 022-fix-rag-backend-resolution
title: fix-rag-backend-resolution
stage: general
date: 2025-12-28
model: Claude Sonnet 4.5
surface: agent
feature: none
branch: main
user: Claude Code
command: fix-rag-backend-resolution
labels:
  - chat
  - widget
  - ui
  - docusaurus
  - react
  - rag
  - backend
  - environment
  - configuration
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - book/src/components/ChatWidget.tsx
  - book/static/js/rag-config.js
  - book/src/theme/Layout/index.tsx
tests: []
---

## PROMPT_TEXT

```bash
fix-rag-backend-resolution
```

## RESPONSE_TEXT

Fixed RAG backend reachability by resolving API base correctly at runtime for the global chat widget, following Spec-Kit Plus methodology. The changes included:

1. Updated API base resolution logic in `book/src/components/ChatWidget.tsx` with proper priority order: `window.__RAG_API_BASE__` → `process.env.REACT_APP_RAG_API_BASE` → `https://robots2026.up.railway.app` (safe default)

2. Created runtime configuration file `book/static/js/rag-config.js` that sets `window.__RAG_API_BASE__` to the production API endpoint

3. Updated global layout `book/src/theme/Layout/index.tsx` to inject the runtime config script using Docusaurus Head component

4. Improved error message accuracy to conditionally show localhost-specific instructions only when the API_BASE contains localhost/127.0.0.1

5. Ensured all fetch calls use the resolved API_BASE consistently across both `/ask` and `/verify` endpoints

The implementation follows the runtime configuration pattern that works well with static hosting (like GitHub Pages) while maintaining flexibility for different deployment environments.

## OUTCOME

Successfully fixed RAG backend resolution with a runtime configuration approach that works across different deployment scenarios. The chat widget now properly resolves the API endpoint based on runtime configuration, improving reliability on GitHub Pages and other static hosting platforms.

## EVALUATION

The RAG backend resolution fix was implemented successfully. The runtime configuration approach allows the application to work properly on GitHub Pages while maintaining flexibility for local development. The priority-based resolution ensures proper fallback behavior, and the conditional error messages provide appropriate guidance based on the environment.