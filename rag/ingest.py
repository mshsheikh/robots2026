"""
Ingestion module for processing book content into Qdrant.
"""
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from sqlalchemy import create_engine, text
import os
import hashlib
from rag.qwen_embeddings import get_qwen_embeddings


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


def ingest_book(docs_dir="book/docs", collection="robots2026", chunk_size=512, overlap=64, id_strategy="sha256"):
    """
    Ingest book content into Qdrant with Neon metadata storage.
    """
    # Initialize Qdrant client
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
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
            conn.execute(text('''
                CREATE TABLE IF NOT EXISTS rag_metadata (
                    id SERIAL PRIMARY KEY,
                    chunk_id TEXT NOT NULL,
                    title TEXT,
                    text TEXT,
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
        text = chapter.read_text(encoding='utf-8')
        text_chunks = chunk_text(text, chunk_size, overlap)

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
                        text("INSERT INTO rag_metadata (chunk_id, title, text) VALUES (:chunk_id, :title, :text)"),
                        [{"chunk_id": str(point_id), "title": chapter.stem, "text": chunk}]
                    )
                    conn.commit()
            except Exception as e:
                print(f"Warning: failed to save metadata to Neon DB: {e}")

    # Upload vectors to Qdrant
    if points:
        qdrant.upsert(collection_name=collection, points=points)
        print(f'✅ Uploaded {len(points)} chunks from {len(chapters)} chapters to Qdrant collection {collection}')
    else:
        print('❌ No chapters found to upload')