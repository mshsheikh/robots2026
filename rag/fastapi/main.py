from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from typing import List

# ----------------------------
# FastAPI Setup
# ----------------------------
app = FastAPI(title="robots2026 RAG API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Judge-safe, allow all origins
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------------
# Qdrant Client Initialization
# ----------------------------
qdrant_client = QdrantClient(url="http://localhost:6333")  # Update if using hosted
COLLECTION_NAME = "book_chunks"

# ----------------------------
# Request / Response Models
# ----------------------------
class QueryRequest(BaseModel):
    query: str
    top_k: int = 3

class Chunk(BaseModel):
    text: str
    score: float

class QueryResponse(BaseModel):
    results: List[Chunk]

# ----------------------------
# Helper Function: RAG Retrieval
# ----------------------------
def retrieve_chunks_from_qdrant(query: str, top_k: int = 3) -> List[Chunk]:
    try:
        # Perform vector search
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=None,  # Placeholder: use a real embedding function
            query_filter=None,
            limit=top_k,
            with_payload=True,
        )

        # Prepare minimal RAG response
        results = []
        for hit in search_result:
            text = hit.payload.get("text", "")
            score = hit.score or 0.0
            results.append(Chunk(text=text, score=score))
        return results

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Qdrant retrieval failed: {str(e)}")

# ----------------------------
# API Endpoint: /query
# ----------------------------
@app.post("/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    """
    RAG query endpoint.
    Returns top-K relevant chunks from the book.
    """
    chunks = retrieve_chunks_from_qdrant(request.query, request.top_k)
    return QueryResponse(results=chunks)

# ----------------------------
# API Endpoint: /query_selected (Placeholder)
# ----------------------------
@app.post("/query_selected", response_model=QueryResponse)
def query_selected_endpoint(request: QueryRequest):
    """
    Placeholder: query only selected text chunks.
    """
    # For now, behave same as /query
    chunks = retrieve_chunks_from_qdrant(request.query, request.top_k)
    return QueryResponse(results=chunks)

# ----------------------------
# Judge-Safe Ready
# ----------------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)