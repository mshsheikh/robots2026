"""
Robotics RAG (Retrieval Augmented Generation) Ingestion
Qdrant vector database ingestion script with document chunking
"""
import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import hashlib


# Configuration
QDRANT_URL = "http://localhost:6333"  # Update as needed
COLLECTION_NAME = "robotics_docs"


def chunk_document(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Simple document chunking function
    In a real implementation, you might want more sophisticated chunking strategies
    """
    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # Try to break at sentence boundary if possible
        if end < len(text):
            # Look for sentence endings near the chunk boundary
            for sep in ['.\n', '. ', '! ', '? ', '\n\n']:
                last_sep = text.rfind(sep, start, end)
                if last_sep != -1 and last_sep > start + chunk_size // 2:
                    end = last_sep + len(sep)
                    break

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        start = end - overlap if end < len(text) else end

    return chunks


def generate_document_id(content: str, source: str = "") -> str:
    """
    Generate a unique ID for a document chunk
    """
    content_hash = hashlib.md5(content.encode()).hexdigest()
    source_hash = hashlib.md5(source.encode()).hexdigest() if source else ""
    return f"{content_hash[:16]}_{source_hash[:8]}" if source else content_hash[:16]


async def upsert_document_chunks(
    client: QdrantClient,
    chunks: List[str],
    source: str = "",
    metadata: Dict[str, Any] = None
) -> bool:
    """
    Upsert document chunks to Qdrant collection
    Note: Where to plug embedding function - each chunk needs to be embedded before upsert
    """
    if metadata is None:
        metadata = {}

    points = []

    for i, chunk in enumerate(chunks):
        # NOTE: This is where you would plug in your embedding function
        # For example: embedding = embed_function(chunk)
        # For now, using a placeholder embedding (this needs to be replaced)
        embedding = [0.0] * 384  # Placeholder - replace with actual embedding

        point = models.PointStruct(
            id=generate_document_id(chunk, f"{source}_{i}"),
            vector=embedding,
            payload={
                "content": chunk,
                "source": source,
                "chunk_index": i,
                "metadata": metadata
            }
        )
        points.append(point)

    try:
        # Upsert the points to the collection
        await client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Successfully upserted {len(points)} chunks to collection '{COLLECTION_NAME}'")
        return True
    except Exception as e:
        print(f"Error upserting chunks: {e}")
        return False


async def setup_collection(client: QdrantClient, vector_size: int = 384):
    """
    Setup Qdrant collection for robotics documentation
    """
    try:
        # Check if collection exists
        collections = await client.get_collections()
        collection_exists = any(col.name == COLLECTION_NAME for col in collections.collections)

        if not collection_exists:
            # Create collection with appropriate vector size
            # NOTE: vector_size should match your embedding model output size
            await client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=vector_size,  # This should match your embedding dimension
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection '{COLLECTION_NAME}' with vector size {vector_size}")
        else:
            print(f"Collection '{COLLECTION_NAME}' already exists")

        return True
    except Exception as e:
        print(f"Error setting up collection: {e}")
        return False


async def main():
    """
    Main function to demonstrate ingestion workflow
    """
    # Initialize Qdrant client
    client = QdrantClient(url=QDRANT_URL)

    # Setup collection
    await setup_collection(client)

    # Example document to ingest (replace with actual document loading)
    sample_document = """
    Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering,
    electrical engineering, computer science, and others. Robotics deals with the design, construction,
    operation, and use of robots, as well as computer systems for their control, sensory feedback, and
    information processing.

    These technologies are used to develop machines that can substitute for humans. Robots can be used
    in any situation and environment meant for humans, though they may not have an anthropomorphic
    appearance. Robots can also be found in the form of insects.
    """

    # Chunk the document
    chunks = chunk_document(sample_document, chunk_size=200, overlap=50)
    print(f"Document chunked into {len(chunks)} pieces")

    # Upsert chunks to Qdrant
    # NOTE: Where to plug embedding function - each chunk needs to be embedded before upsert
    success = await upsert_document_chunks(
        client=client,
        chunks=chunks,
        source="sample_document.txt",
        metadata={"type": "robotics_introduction", "date": "2025-12-10"}
    )

    if success:
        print("Document ingestion completed successfully")
    else:
        print("Document ingestion failed")


if __name__ == "__main__":
    asyncio.run(main())