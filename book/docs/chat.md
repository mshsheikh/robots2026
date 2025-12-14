---
title: AI Chat Assistant
description: Ask questions about the Robots 2026 book using an AI-powered assistant
---

# 🤖 AI Chat Assistant

This page provides access to an AI-powered assistant trained on the *Robots 2026* book content.

## What this does
- Answers questions using the book chapters as context
- Powered by Retrieval-Augmented Generation (RAG)
- Backend: FastAPI + Vector Database (Qdrant)

## How to use
For now, the backend runs locally for development and demos.

**Local endpoint:**
http://localhost:8000

In the next phase, this chat will be embedded directly here with a live UI.

## Live Demo (Phase A)

The chat below connects to a local Retrieval-Augmented Generation (RAG) backend.

> **Note for judges:**
> Start the backend locally (`uvicorn rag.fastapi.main:app --port 8000`)
> before using this demo.

<iframe
  src="/robots2026/chat/index.html"
  width="100%"
  height="600"
  style="border:1px solid #1e293b;border-radius:8px;">
</iframe>

⚠️ This demo requires the backend to be running locally on the same machine
where the browser is opened.