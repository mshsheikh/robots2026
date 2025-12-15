---
id: chat
title: AI Chat Assistant
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

⚠️ **Important:**
This demo requires the backend to be running locally on the same machine
where the browser is opened.

> Start the backend with:
> `uvicorn rag.fastapi.main:app --port 8000`

<iframe
  src="/robots2026/chat/index.html"
  width="100%"
  height="600"
  style={{ border: "1px solid #1e293b", borderRadius: "8px" }}
/>

---
## How the AI Chat Works (concise for judges)

**Architecture (summary)**
- The UI (embedded in this book) is a simple static HTML page: `/robots2026/chat/index.html`.
- The backend is a local FastAPI RAG service that answers questions using a retrieval + generation pipeline.
- Retrieval: document vectors are stored in a vector DB (Qdrant planned).
- Generation: a local or cloud LLM produces the answer using retrieved context (Neon/Postgres plan).
- For Phase A the backend runs locally and the site shows an embedded UI; judges must run the backend locally to see live answers. This design is intentional and documented below.

**Why this design (short)**
- Phase A focuses on reproducible, auditable demos: judges can run everything locally.
- Avoids exposing credentials or incurring cloud costs during the live judge demo.
- Roadmap: Phase B/C will add secure hosted vector + Neon deployment if required.

**Local demo note (judge instructions)**
⚠️ This demo requires the backend to be running on the machine where the browser is open.
Start the backend with:
```

uvicorn rag.fastapi.main:app --port 8000

```
Open the book page, go to **Tools & Resources → AI Chat Assistant**, and run the demo.

---

## Demo Flow (90 seconds — freeze)

**Goal:** Demonstrate end-to-end RAG in 90 seconds.

1. (0–10s) Open project landing page: `https://mshsheikh.github.io/robots2026/`
2. (10–30s) Navigate to **Tools & Resources → AI Chat Assistant** (show iframe).
3. (30–45s) Verify the UI loads; show the static chat box briefly. (Say: "UI is static and embedded.")
4. (45–75s) Ask a single question about the book, e.g. _"What are the main modules in the Physical AI course?"_ — mention that backend must be running for a live answer.
5. (75–90s) Explain architecture quickly: "Retrieval from vector DB → Generation by LLM → Returned by FastAPI."
6. Optional: if backend is not running, show fallback: explain what answer would look like and point to local `rag` folder.

**Demo Tips**
- Announce you will run the demo locally if the judge prefers live answers.
- Keep one short slide (or terminal) showing `uvicorn` running to demonstrate backend health.