# 🎬 90-Second Judge Demo — robots2026

## 0–10s — What This Is
This is **robots2026**, an AI-native textbook for *Physical AI & Humanoid Robotics*, built with Docusaurus and an embedded Retrieval-Augmented Generation (RAG) chatbot.

The goal is not just content, but a **working AI-assisted learning system**.

---

## 10–30s — What Is Live
You are currently viewing the **live GitHub Pages deployment**:

- Docusaurus textbook
- Structured robotics curriculum
- Embedded chat interface

Because GitHub Pages is HTTPS-only, the chat frontend is deployed statically.

---

## 30–55s — How the Chat Works (Architecture)
The chat uses a **local FastAPI backend**:

- Markdown content is chunked and embedded
- Stored in **Qdrant** for vector search
- `/query` performs real semantic retrieval
- Results are returned as grounded context chunks

This is **true RAG**, not a mock or static response.

---

## 55–70s — Why the Chat Is Local
GitHub Pages cannot securely call a local HTTP API.

This is a known **mixed-content browser restriction**, not a bug.

For the demo:
- Backend runs locally
- Frontend correctly demonstrates integration
- Architecture mirrors real production setups

---

## 70–85s — What's Next (Roadmap)
Planned upgrades (Phase C):

- Hosted Qdrant Cloud
- Neon Postgres memory
- Claude sub-agents for chapter assistance
- Authentication and personalization

---

## 85–90s — Closing
This project prioritizes **correct architecture, clarity, and extensibility** — exactly what's required for AI-native education systems.