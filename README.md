# Robots2026 — Physical AI & Humanoid Robotics (Hackathon entry)

**Live:** https://mshsheikh.github.io/robots2026/

**What this is**
A textbook (Docusaurus) + local RAG demo that demonstrates Retrieval-Augmented Generation on course content for Physical AI & Humanoid Robotics.

**Quick architecture (ASCII)**

```

Local browser (embedded UI)
│
└─ iframe → /robots2026/chat/index.html
│
└─ POST /ask → FastAPI (local)
│
├─ Retriever (Qdrant planned)
└─ Generator (LLM; Neon plan)

```

**What's done (Phase A)**
- Docusaurus book deployed to GitHub Pages (https://mshsheikh.github.io/robots2026/)
- Embedded static chat UI at `book/static/chat/index.html`
- Local FastAPI RAG backend (rag/fastapi/main.py) — runs via `uvicorn`
- Documentation & PHRs for Phase A

**Phase B (this commit)**
- Add "How the AI Chat Works" doc + Demo flow + README

**Planned (Phase C+)**
- Claude subagent skill(s) for chapter summarization
- Hosted vector DB + Neon + secure serverless deployment
- Signup/Signin + personalization (better-auth)

**How to demo (short)**
1. Start local backend: `uvicorn rag.fastapi.main:app --port 8000`
2. Open `https://mshsheikh.github.io/robots2026/` (or `http://localhost:3000` if running locally)
3. Visit Tools & Resources → AI Chat Assistant and ask a question