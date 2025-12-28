---
id: 023-fix-api-base-scope-crash
title: fix-api-base-scope-crash
stage: general
date: 2025-12-28
model: Claude Sonnet 4.5
surface: agent
feature: none
branch: main
user: Claude Code
command: fix-api-base-scope-crash
labels:
  - chat
  - widget
  - ui
  - docusaurus
  - react
  - rag
  - backend
  - scoping
  - error-handling
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - book/src/components/ChatWidget.tsx
tests: []
---

## PROMPT_TEXT

```bash
fix-api-base-scope-crash
```

## RESPONSE_TEXT

Fixed API_BASE scoping bug causing runtime crash in global chat widget by implementing proper variable scoping and error handling. The changes included:

1. Created a `resolveApiBase()` helper function at the module level to centralize API_BASE resolution with the priority: `window.__RAG_API_BASE__` → `process.env.REACT_APP_RAG_API_BASE` → `https://robots2026.up.railway.app`

2. Removed duplicate API_BASE declarations inside functions by using the helper function

3. Fixed error handling by moving the API_BASE resolution outside the try block in handleSend function, ensuring the API_BASE variable is available in the catch block for conditional error messaging

4. Added proper runtime safety checks with `typeof window !== "undefined"` to prevent server-side errors

5. Maintained the conditional localhost error messaging functionality while ensuring safe variable access

The implementation resolves the scoping issue that was causing full-page crashes by ensuring consistent API_BASE resolution and safe error handling across all functions.

## OUTCOME

Successfully fixed the API_BASE scoping bug that was causing runtime crashes in the global chat widget. The chat widget now properly resolves the API endpoint with consistent behavior across all functions and safe error handling.

## EVALUATION

The API_BASE scoping fix was implemented successfully. The helper function approach centralizes the resolution logic, eliminates duplicate code, and ensures safe variable access in error handling. The runtime safety checks prevent server-side errors, and the fix maintains all existing functionality while preventing crashes.