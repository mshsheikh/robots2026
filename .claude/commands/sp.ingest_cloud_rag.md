---
name: sp.ingest_cloud_rag
description: One-time cloud RAG ingestion into Qdrant (Railway-safe)
outputs:
  - history/prompts/**/cloud_rag_ingestion.phr
---

## Purpose

Run a **one-time ingestion** of the Robots2026 book into **cloud Qdrant** using
Qwen embeddings. This command is safe to re-run and idempotent.

---

## Preconditions

Ensure the following environment variables are set:

- QDRANT_URL
- QDRANT_API_KEY
- QWEN_API_KEY
- NEON_DATABASE_URL

---

## Steps

### 1. Validate environment variables

```bash
for var in QDRANT_URL QDRANT_API_KEY QWEN_API_KEY NEON_DATABASE_URL; do
  if [ -z "${!var}" ]; then
    echo "❌ Missing env var: $var"
    exit 1
  fi
done

echo "✅ All required env vars present"
