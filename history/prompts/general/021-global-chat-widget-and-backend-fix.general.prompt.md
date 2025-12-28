---
id: 021-global-chat-widget-and-backend-fix
title: global-chat-widget-and-backend-fix
stage: general
date: 2025-12-28
model: Claude Sonnet 4.5
surface: agent
feature: none
branch: main
user: Claude Code
command: global-chat-widget-and-backend-fix
labels:
  - chat
  - widget
  - ui
  - docusaurus
  - react
  - rag
  - backend
  - environment
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - book/src/theme/Layout/index.tsx
  - book/src/components/ChatWidget.tsx
  - book/docs/chat.md
  - book/.env
tests: []
---

## PROMPT_TEXT

```bash
global-chat-widget-and-backend-fix
```

## RESPONSE_TEXT

Implemented a global chat widget across the Docusaurus site and fixed backend reachability via proper environment variable resolution, following Spec-Kit Plus methodology. The changes included:

1. Created a global layout override (`book/src/theme/Layout/index.tsx`) that injects the ChatWidget once across the entire site using Docusaurus's theme aliasing system
2. Removed the page-scoped ChatWidget rendering from `book/docs/chat.md`, keeping only the documentation content
3. Updated the ChatWidget component (`book/src/components/ChatWidget.tsx`) to implement proper API_BASE resolution with priority: process.env.REACT_APP_API_BASE → window.__API_BASE__ → http://localhost:8000
4. Added local development environment config in `book/.env` to set REACT_APP_API_BASE=http://localhost:8000
5. Ensured all fetch calls use the resolved API_BASE consistently
6. Maintained existing error messaging and backend expectations

The implementation follows Docusaurus best practices for theme overrides and properly handles environment-based backend resolution for both development and production environments.

## OUTCOME

Successfully implemented a global chat widget that appears site-wide via a floating button while fixing backend reachability issues through proper environment variable resolution. The widget is now accessible from any page on the site, and the backend connection properly resolves based on environment configuration.

## EVALUATION

The global chat widget implementation was completed successfully. The floating chat interface is now available across all pages of the Docusaurus site through the theme layout override. The backend resolution logic properly handles different environments (local development vs production) through the prioritized environment variable approach. The implementation follows Docusaurus conventions and maintains all existing functionality while improving the user experience.