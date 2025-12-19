"""
Robotics RAG (Retrieval Augmented Generation) Ingestion
Qdrant vector database ingestion script with document chunking from Docusaurus book
"""
import asyncio
import hashlib
import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, VectorParams, Distance

import openai  # For embeddings (placeholder-safe)
import dotenv  # For environment variables

# Load environment variables
dotenv.load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")  # Update as needed
COLLECTION_NAME = "robots2026_docs"  # Changed to match spec requirement
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")  # Configurable model
EMBEDDING_DIMENSION = int(os.getenv("EMBEDDING_DIMENSION", "1536"))  # Dimension of embedding model

# Placeholder for Neon Postgres connection (credentials not included)
# NEON_DB_URL = os.getenv("NEON_DB_URL")  # Uncomment when Neon credentials are available
# neon_connection = None  # Initialize connection when available


def load_markdown_files(base_path: str = "book/docs") -> List[Dict[str, Any]]:
    """
    Load markdown content from Docusaurus book
    - Source directory: book/docs/**/*.md
    - Ignore build artifacts, node_modules, and .docusaurus
    """
    markdown_files = []
    base_dir = Path(base_path)

    # Patterns to exclude
    exclude_patterns = [
        ".docusaurus",
        "node_modules",
        "__pycache__",
        ".git",
        "build",
        ".venv",
        "__pycache__"
    ]

    for md_file in base_dir.rglob("*.md"):
        # Check if the file path contains any exclude patterns
        if any(pattern in str(md_file) for pattern in exclude_patterns):
            continue

        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract frontmatter and content
            frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)$', content, re.DOTALL)
            frontmatter = {}
            text_content = content

            if frontmatter_match:
                frontmatter_str = frontmatter_match.group(1)
                text_content = frontmatter_match.group(2)

                # Parse simple frontmatter (key: value pairs)
                for line in frontmatter_str.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        frontmatter[key.strip()] = value.strip()

            markdown_files.append({
                'path': str(md_file),
                'content': text_content,
                'frontmatter': frontmatter
            })

        except Exception as e:
            logging.warning(f"Could not read file {md_file}: {e}")

    return markdown_files


def chunk_by_headings(content: str, max_tokens: int = 600) -> List[Dict[str, Any]]:
    """
    Chunk markdown content by headings with safe token size (500-800 tokens approx)
    Preserves metadata: file_path, week/module title, heading
    """
    chunks = []

    # Split content by headings (h1, h2, h3)
    heading_pattern = r'^(#+)\s+(.+)$[\s\n]*([^#]*)'
    lines = content.split('\n')

    current_chunk = {
        'heading': '',
        'content': '',
        'token_count': 0
    }

    for line in lines:
        # Check if line is a heading
        heading_match = re.match(r'^(#{1,3})\s+(.+)', line.strip())

        if heading_match:
            # If current chunk has content, save it before starting new chunk
            if current_chunk['content'].strip():
                chunks.append({
                    'heading': current_chunk['heading'],
                    'content': current_chunk['content'].strip(),
                    'token_count': len(current_chunk['content'].split())  # Approximate token count
                })

            # Start new chunk with this heading
            current_chunk = {
                'heading': heading_match.group(2).strip(),
                'content': line + '\n',
                'token_count': len(line.split())  # Approximate token count
            }
        else:
            # Add line to current chunk
            current_chunk['content'] += line + '\n'
            current_chunk['token_count'] += len(line.split())  # Approximate token count

            # If chunk is getting too large, split it
            if current_chunk['token_count'] > max_tokens:
                chunks.append({
                    'heading': current_chunk['heading'],
                    'content': current_chunk['content'].strip(),
                    'token_count': current_chunk['token_count']
                })

                # Start new chunk
                current_chunk = {
                    'heading': current_chunk['heading'],
                    'content': '',
                    'token_count': 0
                }

    # Add final chunk if it has content
    if current_chunk['content'].strip():
        chunks.append({
            'heading': current_chunk['heading'],
            'content': current_chunk['content'].strip(),
            'token_count': current_chunk['token_count']
        })

    return chunks


def generate_document_id(content: str, source: str = "") -> str:
    """
    Generate a unique ID for a document chunk
    """
    content_hash = hashlib.md5(content.encode()).hexdigest()
    source_hash = hashlib.md5(source.encode()).hexdigest() if source else ""
    return f"{content_hash[:16]}_{source_hash[:8]}" if source else content_hash[:16]


async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings using OpenAI API (placeholder-safe)
    In a real implementation, this would connect to OpenAI API
    """
    embeddings = []

    # Check if OpenAI API key is available
    openai_api_key = os.getenv("OPENAI_API_KEY")

    if not openai_api_key:
        # Placeholder: return random embeddings of correct dimension
        import random
        for _ in texts:
            embedding = [random.uniform(-0.1, 0.1) for _ in range(EMBEDDING_DIMENSION)]
            embeddings.append(embedding)
        logging.warning("Using placeholder embeddings - set OPENAI_API_KEY for real embeddings")
    else:
        # Real implementation would use OpenAI
        # openai.api_key = openai_api_key
        # response = openai.Embedding.create(input=texts, model=EMBEDDING_MODEL)
        # embeddings = [item['embedding'] for item in response['data']]
        pass

    return embeddings


async def upsert_document_chunks(
    client: QdrantClient,
    chunks: List[Dict[str, Any]],
    source: str = "",
    metadata: Dict[str, Any] = None
) -> bool:
    """
    Upsert document chunks to Qdrant collection with embeddings
    Makes ingestion idempotent (safe to re-run)
    """
    if metadata is None:
        metadata = {}

    points = []

    # Get embeddings for all chunks at once for efficiency
    texts_for_embedding = [chunk['content'] for chunk in chunks]
    embeddings = await get_embeddings(texts_for_embedding)

    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        # Generate unique ID based on content and source
        point_id = generate_document_id(chunk['content'], f"{source}_{i}")

        # Create point with vector and metadata
        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "content": chunk['content'],
                "source": source,
                "heading": chunk.get('heading', ''),
                "chunk_index": i,
                "metadata": metadata,
                "token_count": chunk.get('token_count', len(chunk['content'].split()))
            }
        )
        points.append(point)

    try:
        # Upsert the points to the collection
        # This operation is idempotent - same IDs will overwrite existing points
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Successfully upserted {len(points)} chunks to collection '{COLLECTION_NAME}'")
        return True
    except Exception as e:
        print(f"Error upserting chunks: {e}")
        return False


def setup_collection(client: QdrantClient, vector_size: int = 1536):
    """
    Setup Qdrant collection for robotics documentation
    Creates collection if it doesn't exist, makes it idempotent
    """
    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_exists = any(col.name == COLLECTION_NAME for col in collections.collections)

        if not collection_exists:
            # Create collection with appropriate vector size
            # NOTE: vector_size should match your embedding model output size
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=vector_size,  # This should match your embedding dimension
                    distance=Distance.COSINE
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
    Main ingestion function to load, chunk, and ingest Docusaurus book content
    """
    # Initialize Qdrant client
    print(f"Connecting to Qdrant at {QDRANT_URL}")
    client = QdrantClient(url=QDRANT_URL, timeout=10)

    # Setup collection (idempotent - safe to run multiple times)
    setup_collection(client, EMBEDDING_DIMENSION)

    # Load markdown files from book/docs
    print("Loading markdown files from book/docs...")
    markdown_files = load_markdown_files()
    print(f"Loaded {len(markdown_files)} markdown files")

    # Process each file
    total_chunks = 0
    for file_info in markdown_files:
        print(f"Processing {file_info['path']}...")

        # Extract week/module information from path
        path_parts = file_info['path'].split('/')
        week_info = next((part for part in path_parts if 'week' in part.lower()), 'unknown')

        # Chunk the content by headings
        chunks = chunk_by_headings(file_info['content'])
        print(f"  Chunked into {len(chunks)} segments")

        # Prepare metadata
        file_metadata = {
            'week': week_info,
            'file_path': file_info['path'],
            'original_title': file_info['frontmatter'].get('title', ''),
            'sidebar_position': file_info['frontmatter'].get('sidebar_position', '')
        }
        file_metadata.update(file_info['frontmatter'])  # Add any other frontmatter fields

        # Upsert chunks to Qdrant
        success = await upsert_document_chunks(
            client=client,
            chunks=chunks,
            source=file_info['path'],
            metadata=file_metadata
        )

        if success:
            total_chunks += len(chunks)
            print(f"  Successfully ingested {len(chunks)} chunks")
        else:
            print(f"  Failed to ingest chunks for {file_info['path']}")

    print(f"\nIngestion completed! Total chunks ingested: {total_chunks}")
    print(f"All content stored in Qdrant collection: {COLLECTION_NAME}")


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())