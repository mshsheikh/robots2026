"""
Robotics RAG (Retrieval Augmented Generation) API
FastAPI application with /ask, /query, and /query_selected endpoints for question answering
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import uvicorn
import logging
from qdrant_client import QdrantClient
from qdrant_client.http.models import SearchRequest, PointStruct, VectorParams, Distance, Filter, FieldCondition, MatchText
import os


app = FastAPI(
    title="Robotics RAG API",
    description="Retrieval Augmented Generation API for robotics documentation",
    version="0.1.0"
)

# Add CORS middleware to allow all origins (for local + GitHub Pages iframe)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development and iframe usage
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)


# Initialize Qdrant client - assumes Qdrant runs locally at http://localhost:6333
# Using collection name 'robots2026_docs' as specified
try:
    qdrant_client = QdrantClient(
        url="http://localhost:6333",
        timeout=10  # 10 second timeout for Qdrant operations
    )
    # Verify connection by listing collections
    collections = qdrant_client.get_collections()
    logging.info(f"Successfully connected to Qdrant, found {len(collections.collections)} collections")
    qdrant_available = True
except Exception as e:
    logging.error(f"Could not connect to Qdrant: {e}")
    # Create a mock client for development purposes if Qdrant is not available
    qdrant_available = False
    qdrant_client = None


# Placeholder for Neon Postgres connection
# In a real implementation, you would initialize the connection here
# neon_db_url = os.getenv("NEON_DB_URL", "postgresql://user:password@localhost:5432/robotics_db")
# neon_connection = None  # Initialize connection when Neon credentials are available
NEON_AVAILABLE = False  # Set to True when Neon connection is established


# Placeholder for future OpenAI / Agents SDK integration
# openai_client = None  # Initialize when API keys are available
OPENAI_AVAILABLE = False  # Set to True when OpenAI integration is configured


class AskRequest(BaseModel):
    query: str
    max_results: Optional[int] = 5
    threshold: Optional[float] = 0.7


class QueryRequest(BaseModel):
    question: str
    max_results: Optional[int] = 5


class QuerySelectedRequest(BaseModel):
    question: str
    selected_text: str
    max_results: Optional[int] = 5


class AskResponse(BaseModel):
    query: str
    answer: str
    sources: list
    confidence: float


class QueryResponse(BaseModel):
    question: str
    results: List[Dict[str, Any]]
    total_found: int


class QuerySelectedResponse(BaseModel):
    question: str
    selected_text: str
    results: List[Dict[str, Any]]
    total_found: int


@app.get("/")
async def root():
    return {
        "message": "Robotics RAG API",
        "endpoints": [
            "GET /",
            "POST /ask",
            "POST /query",
            "POST /query_selected"
        ],
        "services": {
            "qdrant": qdrant_available,
            "neon_postgres": NEON_AVAILABLE,
            "openai": OPENAI_AVAILABLE
        }
    }


@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    """
    Main endpoint to ask questions about robotics documentation
    This is a skeleton implementation - actual RAG logic will be implemented here
    """
    try:
        # Placeholder for RAG logic
        # 1. Embed the query using embedding model
        # 2. Search Qdrant vector database for relevant chunks
        # 3. Format context from retrieved chunks
        # 4. Generate answer using LLM with context
        # 5. Return answer with sources and confidence

        # For now, return a placeholder response
        return AskResponse(
            query=request.query,
            answer="This is a placeholder response. Actual RAG implementation will go here.",
            sources=[],
            confidence=0.0
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/query", response_model=QueryResponse)
async def query_documents(request: QueryRequest):
    """
    Endpoint to query the RAG system for relevant documents/chunks
    Accepts a user question and returns relevant document chunks from Qdrant
    """
    if not qdrant_available:
        raise HTTPException(status_code=503, detail="Qdrant database not available")

    try:
        # In a real implementation, this would:
        # 1. Convert the question to an embedding using the same model as used for indexing
        # 2. Search the Qdrant collection for similar vectors
        # 3. Return the most relevant document chunks

        # For now, return mock results if Qdrant is not available, or search if it is
        if qdrant_client:
            try:
                # Perform vector search in Qdrant
                # In a real implementation, we would convert the query to an embedding first
                search_results = qdrant_client.search(
                    collection_name="robots2026_docs",
                    query_text=request.question,  # This would be an embedding in real implementation
                    limit=request.max_results or 5
                )

                results = []
                for hit in search_results:
                    results.append({
                        "id": hit.id,
                        "score": hit.score,
                        "content": hit.payload.get("content", ""),
                        "metadata": hit.payload.get("metadata", {})
                    })

                total_found = len(results)
            except Exception:
                # If vector search fails, return mock results for development
                results = [
                    {
                        "id": f"mock_chunk_{i}",
                        "score": 0.9 - (i * 0.1),
                        "content": f"Mock document chunk {i} containing information related to: {request.question}",
                        "metadata": {
                            "source": f"doc_part_{i}.md",
                            "section": f"section_{i}",
                            "similarity": 0.9 - (i * 0.1)
                        }
                    }
                    for i in range(min(request.max_results or 5, 5))
                ]
                total_found = len(results)
        else:
            # Return mock results for development when Qdrant is not available
            results = [
                {
                    "id": f"mock_chunk_{i}",
                    "score": 0.9 - (i * 0.1),
                    "content": f"Mock document chunk {i} containing information related to: {request.question}",
                    "metadata": {
                        "source": f"doc_part_{i}.md",
                        "section": f"section_{i}",
                        "similarity": 0.9 - (i * 0.1)
                    }
                }
                for i in range(min(request.max_results or 5, 5))
            ]
            total_found = len(results)

        return QueryResponse(
            question=request.question,
            results=results,
            total_found=total_found
        )
    except Exception as e:
        logging.error(f"Error querying documents: {e}")
        raise HTTPException(status_code=500, detail=f"Error querying documents: {str(e)}")


@app.post("/query_selected", response_model=QuerySelectedResponse)
async def query_documents_with_selection(request: QuerySelectedRequest):
    """
    Endpoint to query the RAG system with selected text context
    Accepts a user question plus selected text, returns relevant document chunks
    """
    if not qdrant_available:
        raise HTTPException(status_code=503, detail="Qdrant database not available")

    try:
        # In a real implementation, this would:
        # 1. Combine the question and selected text for better context
        # 2. Convert the combined query to an embedding
        # 3. Search the Qdrant collection for relevant chunks
        # 4. Potentially rerank results based on relevance to both question and selection

        # For now, return mock results that incorporate the selected text
        combined_query = f"{request.question} Context: {request.selected_text}"

        if qdrant_client:
            try:
                # Perform vector search in Qdrant using combined query
                # In a real implementation, we would convert the query to an embedding first
                search_results = qdrant_client.search(
                    collection_name="robots2026_docs",
                    query_text=combined_query,  # This would be an embedding in real implementation
                    limit=request.max_results or 5
                )

                results = []
                for hit in search_results:
                    results.append({
                        "id": hit.id,
                        "score": hit.score,
                        "content": hit.payload.get("content", ""),
                        "metadata": hit.payload.get("metadata", {})
                    })

                total_found = len(results)
            except Exception:
                # If vector search fails, return mock results for development
                results = [
                    {
                        "id": f"mock_chunk_{i}_selected",
                        "score": 0.85 - (i * 0.1),
                        "content": f"Mock document chunk {i} containing information related to: {request.question} in context of: {request.selected_text}",
                        "metadata": {
                            "source": f"doc_part_{i}_selected.md",
                            "section": f"section_{i}_context",
                            "similarity": 0.85 - (i * 0.1)
                        }
                    }
                    for i in range(min(request.max_results or 5, 5))
                ]
                total_found = len(results)
        else:
            # Return mock results for development when Qdrant is not available
            results = [
                {
                    "id": f"mock_chunk_{i}_selected",
                    "score": 0.85 - (i * 0.1),
                    "content": f"Mock document chunk {i} containing information related to: {request.question} in context of: {request.selected_text}",
                    "metadata": {
                        "source": f"doc_part_{i}_selected.md",
                        "section": f"section_{i}_context",
                        "similarity": 0.85 - (i * 0.1)
                    }
                }
                for i in range(min(request.max_results or 5, 5))
            ]
            total_found = len(results)

        return QuerySelectedResponse(
            question=request.question,
            selected_text=request.selected_text,
            results=results,
            total_found=total_found
        )
    except Exception as e:
        logging.error(f"Error querying documents with selection: {e}")
        raise HTTPException(status_code=500, detail=f"Error querying documents with selection: {str(e)}")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)