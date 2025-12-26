---
id: chat
title: AI Chat Assistant & How It Works
sidebar_position: 15
---

# 🤖 AI Chat Assistant

This page provides a live demo and detailed explanation of the AI chat system.

---

## Live Demo

<iframe
  src="/robots2026/chat/index.html"
  width="100%"
  height="600"
  style={{ border: "1px solid #1e293b", borderRadius: "8px" }}
/>

> ⚠️ **Note:** For live answers, the backend must be running locally:
> ```bash
> uvicorn rag.fastapi.main:app --port 8000
> ```

---

## Demo Flow (90 seconds — freeze)

**Goal:** Demonstrate end-to-end RAG in 90 seconds.

1. (0–10s) Open project landing page: `https://mshsheikh.github.io/robots2026/`
2. (10–30s) Navigate to **Tools & Resources → AI Chat Assistant**
3. (30–45s) Verify the UI loads; show the static chat box briefly
4. (45–75s) Ask a single question, e.g., _"What are the main modules in the Physical AI course?"_
5. (75–90s) Explain architecture quickly: "Retrieval → LLM → FastAPI → Frontend"
6. Optional fallback if backend not running: explain what the answer would look like using local `rag` folder

**Tips:** Show `uvicorn` terminal briefly to demonstrate backend health.

---

## How the AI Chat Works

### Architecture Overview

- **Frontend:** Docusaurus page with embedded chat iframe  
- **Chat Interface:** HTML/CSS/JS widget communicating via HTTP  
- **Backend:** FastAPI RAG service  
- **Vector DB:** Qdrant (stores document embeddings)  
- **Embedding System:** Converts text into vectors (OpenAI-compatible)  
- **Future:** Neon Postgres for conversation persistence  

### Retrieval-Augmented Generation (RAG) Flow

1. User submits question → backend `/query` endpoint  
2. Embedding generated for the question  
3. Qdrant returns top-K relevant document chunks  
4. Context assembled and sent to LLM (planned)  
5. JSON results returned to frontend  

### Selected-Text Question Answering

- Combines user question + selected text for more focused retrieval  
- Returns results contextualized to user-selected passage  

### Local vs Live Deployment

- **Local:** Full RAG functionality, including `/query` and `/query_selected` endpoints  
- **GitHub Pages:** Embedded UI is visible, but live queries blocked due to mixed content policy  

### Current Limitations & Future Improvements

- Response generation not fully implemented  
- Mixed content security limits live API calls  
- Backend Qdrant service required for local queries  
- Planned: LLM integration, session persistence, improved retrieval strategies

---

