from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import os
from rag.qwen_embeddings import get_qwen_embeddings
from sqlalchemy import create_engine, text

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

def main():
    # Qdrant hosted client
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Neon DB (Postgres) connection
    engine = create_engine(os.getenv("NEON_DATABASE_URL"), echo=False)

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

    points = []
    point_id = 0
    for chapter in chapters:
        print(f"Processing {chapter}...")
        text = chapter.read_text(encoding='utf-8')

        # Split text into chunks
        text_chunks = chunk_text(text)

        for chunk_idx, chunk in enumerate(text_chunks):
            try:
                # Generate embedding using Qwen
                embedding = get_qwen_embedding(chunk)

                points.append(PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        'chapter': str(chapter),
                        'text': chunk,  # Using 'text' to match the search expectation in main.py
                        'title': chapter.stem,
                        'chunk_idx': chunk_idx
                    }
                ))

                # Save metadata to Neon DB
                try:
                    with engine.connect() as conn:
                        conn.execute(
                            text("INSERT INTO rag_metadata (chunk_id, title, text) VALUES (:chunk_id, :title, :text)"),
                            [{"chunk_id": point_id, "title": chapter.stem, "text": chunk}]
                        )
                        conn.commit()
                except Exception as e:
                    print(f"Warning: failed to save metadata to Neon DB: {e}")

                point_id += 1
            except Exception as e:
                print(f"Error processing chunk {chunk_idx} of {chapter}: {str(e)}")
                continue  # Skip this chunk and continue with the next one

    # Upload vectors to Qdrant
    if points:
        qdrant.upsert(collection_name=collection_name, points=points)
        print(f'✅ Uploaded {len(points)} chunks from {len(chapters)} chapters to Qdrant using Qwen embeddings')
    else:
        print('❌ No chapters found to upload')

if __name__ == "__main__":
    main()