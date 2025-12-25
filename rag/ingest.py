"""
Ingestion module for processing book content into Qdrant.
"""
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from sqlalchemy import create_engine, text as sql_text
import os
import hashlib
import time
from rag.qwen_embeddings import get_qwen_embeddings
from qdrant_client.http.models import Batch


def chunk_text(text, chunk_size=512, overlap=64):
    """
    Split text into overlapping chunks.
    """
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunk = text[i:i + chunk_size]
        if chunk.strip():
            chunks.append(chunk)
        if i + chunk_size >= len(text):
            break
    return chunks


def get_sha256_id(content):
    """
    Generate a deterministic ID using SHA256 hash of content.
    """
    return hashlib.sha256(content.encode()).hexdigest()[:16]


def retry_with_backoff(func, *args, max_retries=3, base_delay=1, **kwargs):
    """
    Execute a function with exponential backoff retry logic.
    """
    for attempt in range(max_retries):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            if attempt == max_retries - 1:  # Last attempt
                raise e
            delay = base_delay * (2 ** attempt)  # Exponential backoff: 1s, 2s, 4s
            print(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
            time.sleep(delay)


def ingest_book(docs_dir="book/docs", collection="robots2026", chunk_size=512, overlap=64, id_strategy="sha256"):
    """
    Ingest book content into Qdrant with Neon metadata storage.
    """
    # Initialize Qdrant client with timeout
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=60
    )

    # Initialize Neon DB
    engine = create_engine(os.getenv("NEON_DATABASE_URL"), echo=False)

    # Create collection if it doesn't exist
    try:
        qdrant.get_collection(collection)
        print(f"Collection {collection} exists, clearing it...")
        qdrant.delete_collection(collection)
    except:
        print(f"Collection {collection} does not exist, creating it...")

    qdrant.create_collection(
        collection_name=collection,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )

    # Create metadata table if it doesn't exist
    try:
        with engine.connect() as conn:
            conn.execute(sql_text('''
                CREATE TABLE IF NOT EXISTS rag_metadata (
                    id SERIAL PRIMARY KEY,
                    chunk_id TEXT NOT NULL,
                    title TEXT,
                    payload_text TEXT,
                    created_at TIMESTAMP DEFAULT NOW()
                )
            '''))
            conn.commit()
    except Exception as e:
        print(f"Warning: failed to create metadata table: {e}")

    # Process all markdown files
    book_path = Path(docs_dir)
    chapters = list(book_path.glob('**/*.md'))

    points = []
    for chapter in chapters:
        print(f"Processing {chapter}...")
        payload_text = chapter.read_text(encoding='utf-8')
        text_chunks = chunk_text(payload_text, chunk_size, overlap)

        for chunk_idx, chunk in enumerate(text_chunks):
            # Generate embedding using Qwen
            embedding = get_qwen_embeddings(chunk)

            # Generate ID based on strategy
            if id_strategy == "sha256":
                point_id = get_sha256_id(chunk)
            else:
                point_id = len(points)  # fallback to sequential

            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'chapter': str(chapter),
                    'text': chunk,
                    'title': chapter.stem,
                    'chunk_idx': chunk_idx
                }
            ))

            # Save metadata to Neon DB
            try:
                with engine.connect() as conn:
                    conn.execute(
                        sql_text("INSERT INTO rag_metadata (chunk_id, title, payload_text) VALUES (:chunk_id, :title, :payload_text)"),
                        [{"chunk_id": str(point_id), "title": chapter.stem, "payload_text": chunk}]
                    )
                    conn.commit()
            except Exception as e:
                print(f"Warning: failed to save metadata to Neon DB: {e}")

        # Upload vectors to Qdrant using batching
    if points:
        # Process points in batches of 32
        batch_size = 32
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            retry_with_backoff(qdrant.upsert, collection_name=collection, points=batch)
            print(f'✅ Uploaded batch {i//batch_size + 1} ({len(batch)} points) to Qdrant collection {collection}')

        print(f'✅ Uploaded {len(points)} chunks from {len(chapters)} chapters to Qdrant collection {collection}')
    else:
        print('❌ No chapters found to upload')