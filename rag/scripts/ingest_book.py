from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import os
from rag.qwen_embeddings import get_qwen_embeddings
import time
import uuid
from qdrant_client.http.models import Batch

def chunk_text(text, chunk_size=1000):
    """
    Split text into overlapping chunks
    """
    chunks = []
    for i in range(0, len(text), chunk_size // 2):  # 50% overlap
        chunk = text[i:i + chunk_size]
        if chunk.strip():
            chunks.append(chunk)
        if i + chunk_size >= len(text):
            break
    return chunks


def get_qwen_embedding(text):
    """
    Get embedding from Qwen API using the centralized module
    """
    return get_qwen_embeddings(text)


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

def main():
    # Qdrant hosted client with timeout
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=60
    )

    # Check if collection exists, create if it doesn't
    collection_name = 'book_chunks'  # Use the same collection name as in main.py
    try:
        qdrant.get_collection(collection_name)
        print(f"Collection {collection_name} exists, clearing it...")
        qdrant.delete_collection(collection_name)
    except:
        print(f"Collection {collection_name} does not exist, creating it...")

    # Create collection with vector parameters
    qdrant.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )

    # Collect all Markdown chapters from book/docs
    book_path = Path('book/docs')
    chapters = list(book_path.glob('**/*.md'))

    if not chapters:
        print('❌ No chapters found to upload')
        return

    total_chunks = 0
    total_chapters = len(chapters)

    for chapter_idx, chapter in enumerate(chapters):
        print(f"📖 Processing {chapter} ({chapter_idx + 1}/{total_chapters})...")
        payload_text = chapter.read_text(encoding='utf-8')

        # Split text into chunks
        text_chunks = chunk_text(payload_text)

        points = []
        for chunk_idx, chunk in enumerate(text_chunks):
            try:
                # Generate embedding using Qwen
                embedding = get_qwen_embedding(chunk)

                # Generate deterministic UUID based on content and source
                point_id = get_deterministic_uuid(chunk, str(chapter))

                points.append(PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        'source_file': str(chapter),
                        'text': chunk,  # Using 'text' to match the search expectation in main.py
                        'title': chapter.stem,
                        'chunk_index': chunk_idx,
                        'source_type': 'book_document'
                    }
                ))

            except Exception as e:
                print(f"❌ Error processing chunk {chunk_idx} of {chapter}: {str(e)}")
                continue  # Skip this chunk and continue with the next one

        # Upload vectors to Qdrant using batching
        if points:
            # Process points in batches of 32
            batch_size = 32
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                retry_with_backoff(qdrant.upsert, collection_name=collection_name, points=batch)
                print(f'📦 Uploaded batch {i//batch_size + 1} ({len(batch)} points) to Qdrant')

            total_chunks += len(points)
            print(f'✅ Completed {chapter.stem}: {len(points)} chunks')

    print(f'🎉 Successfully uploaded {total_chunks} chunks from {total_chapters} chapters to Qdrant using Qwen embeddings')

if __name__ == "__main__":
    main()