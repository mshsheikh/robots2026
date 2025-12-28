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

**Phase C+ (Hosted Services)**
- Hosted Qdrant vector database
- Neon Postgres for metadata storage
- Qwen embeddings API
- Claude subagent skill(s) for chapter summarization
- Signup/Signin + personalization (better-auth)

**How to demo (short)**
1. Start local backend: `uvicorn rag.fastapi.main:app --port 8000`
2. Open `https://mshsheikh.github.io/robots2026/` (or `http://localhost:3000` if running locally)
3. Visit Tools & Resources → AI Chat Assistant and ask a question

## Setup and Usage (Hosted Services)

This project includes a Retrieval-Augmented Generation (RAG) system with a Docusaurus frontend and FastAPI backend using hosted services.

### Environment Setup

First, set the required environment variables:

```bash
source set_env_vars.sh
```

### Starting the Services

1. **Start the backend server:**
   ```bash
   ./start_backend.sh
   ```
   This will start the FastAPI server on port 8000.

2. **Start the frontend:**
   ```bash
   ./start_frontend.sh
   ```
   This will start the Docusaurus frontend (typically on port 3000).

### Testing the System

Once both services are running, you can test the chat functionality:

```bash
./test_chat.sh
```

This sends a test query to the backend API.

### API Endpoints

- `/ask` - Main RAG endpoint for querying the knowledge base
- `/verify` - Health check for Qdrant and Neon DB connections

### Required Dependencies

Make sure you have the following installed:
- Python 3.8+
- Node.js and npm
- uvicorn
- FastAPI and related packages
- Qdrant client
- SQLAlchemy