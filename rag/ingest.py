"""
Ingestion module for processing book content into Qdrant.
"""
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import os
import time
import uuid
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


def get_deterministic_uuid(content, source_file):
    """
    Generate a deterministic UUID5 based on content and source file.
    """
    namespace = uuid.uuid5(uuid.NAMESPACE_DNS, "robots2026.ai")
    combined = f"{source_file}::{content}"
    return str(uuid.uuid5(namespace, combined))


def retry_with_backoff(func, *args, max_retries=3, base_delay=1, **kwargs):
    """
    Execute a function with exponential backoff retry logic.
    """
    for attempt in range(max_retries):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            if attempt == max_retries - 1:  # Last attempt
                print(f"❌ Final attempt failed: {str(e)}")
                raise e
            delay = base_delay * (2 ** attempt)  # Exponential backoff: 1s, 2s, 4s
            print(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
            time.sleep(delay)


def ingest_book(docs_dir="book/docs", collection="robots2026", chunk_size=512, overlap=64):
    """
    Ingest book content into Qdrant with Qdrant as the single source of truth.
    All metadata is stored in Qdrant payloads.
    """
    # Initialize Qdrant client with timeout
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=60
    )

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

    # Process all markdown files
    book_path = Path(docs_dir)
    chapters = list(book_path.glob('**/*.md'))

    if not chapters:
        print('❌ No chapters found to upload')
        return

    total_chunks = 0
    total_chapters = len(chapters)

    for chapter_idx, chapter in enumerate(chapters):
        print(f"📖 Processing {chapter} ({chapter_idx + 1}/{total_chapters})...")
        payload_text = chapter.read_text(encoding='utf-8')
        text_chunks = chunk_text(payload_text, chunk_size, overlap)

        points = []
        for chunk_idx, chunk in enumerate(text_chunks):
            # Generate embedding using Qwen
            embedding = get_qwen_embeddings(chunk)

            # Generate deterministic UUID (Qdrant-safe)
            point_id = get_deterministic_uuid(chunk, str(chapter))


            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'source_file': str(chapter),
                    'text': chunk,
                    'title': chapter.stem,
                    'chunk_index': chunk_idx,
                    'source_type': 'book_document'
                }
            ))

        # Upload vectors to Qdrant using batching
        if points:
            # Process points in batches of 32
            batch_size = 32
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                retry_with_backoff(qdrant.upsert, collection_name=collection, points=batch)
                print(f'📦 Uploaded batch {i//batch_size + 1} ({len(batch)} points) to Qdrant collection {collection}')

            total_chunks += len(points)
            print(f'✅ Completed {chapter.stem}: {len(points)} chunks')

    print(f'🎉 Successfully uploaded {total_chunks} chunks from {total_chapters} chapters to Qdrant collection {collection}')