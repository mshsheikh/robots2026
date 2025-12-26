from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
from qdrant_client import QdrantClient
import os
import requests
from sqlalchemy import create_engine, text as sql_text
from fastapi.responses import JSONResponse

# ----------------------------
# Qwen Embedding Helper
# ----------------------------
from rag.qwen_embeddings import get_qwen_embeddings as embed_text

# ----------------------------
# FastAPI Setup
# ----------------------------
app = FastAPI(title="robots2026 RAG API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # judge-safe
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------------
# Hosted Qdrant and Neon DB
# ----------------------------
# Qdrant hosted client with timeout
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=60
)
COLLECTION_NAME = "robots2026"

# Neon DB (Postgres) connection
engine = create_engine(os.getenv("NEON_DATABASE_URL"), echo=False)

# Optional: create a table for RAG metadata if it doesn't exist
try:
    with engine.connect() as conn:
        conn.execute(sql_text('''
            CREATE TABLE IF NOT EXISTS rag_metadata (
                id SERIAL PRIMARY KEY,
                chunk_id INT NOT NULL,
                title TEXT,
                payload_text TEXT,
                created_at TIMESTAMP DEFAULT NOW()
            )
        '''))
        conn.commit()
except Exception as e:
    print(f"Warning: failed to create metadata table: {e}")

# ----------------------------
# Models
# ----------------------------
class QueryRequest(BaseModel):
    query: str = None
    question: str = None
    top_k: int = 3

    @property
    def effective_query(self) -> str:
        # Use 'query' if available, otherwise use 'question' for compatibility
        return self.query or self.question

class Chunk(BaseModel):
    text: str
    score: float

class QueryResponse(BaseModel):
    results: List[Chunk]

# ----------------------------
# Retrieval Logic (FIXED)
# ----------------------------
def retrieve_chunks_from_qdrant(query: str, top_k: int) -> List[Chunk]:
    try:
        if not query:
            raise ValueError("Query cannot be empty")

        vector = embed_text(query)

        result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=vector,
            limit=top_k,
            with_payload=True,
        )

        hits = result[0] if isinstance(result, tuple) else result

        results = []
        for hit in hits:
            results.append(
                Chunk(
                    text=hit.payload.get("text", "") if hit.payload else "",
                    score=hit.score,
                )
            )

        return results

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ----------------------------
# Endpoints
# ----------------------------
@app.post("/ask", response_model=QueryResponse)
async def ask_endpoint(request: QueryRequest):
    results = retrieve_chunks_from_qdrant(request.effective_query, request.top_k)
    return QueryResponse(results=results)


@app.get("/verify")
async def verify_connections():
    result = {"qdrant": None, "neon": None}
    # Test Qdrant connection
    try:
        collections = qdrant_client.get_collections().collections
        result["qdrant"] = {"status": "ok", "collections": [c.name for c in collections]}
    except Exception as e:
        result["qdrant"] = {"status": "error", "error": str(e)}
    # Test Neon DB connection
    try:
        with engine.connect() as conn:
            test = conn.execute(sql_text("SELECT 1")).fetchone()
            if test and test[0] == 1:
                result["neon"] = {"status": "ok"}
            else:
                result["neon"] = {"status": "error", "error": "Unexpected result"}
    except Exception as e:
        result["neon"] = {"status": "error", "error": str(e)}
    return JSONResponse(result)
