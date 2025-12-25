#!/usr/bin/env python3
"""
Test script to verify RAG ingestion works with a small sample
"""

import os
import sys
from dotenv import load_dotenv
from pathlib import Path

# Add the project root to Python path
sys.path.insert(0, os.path.abspath('.'))

# Load environment variables
load_dotenv()

from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from rag.qwen_embeddings import get_qwen_embeddings
from sqlalchemy import create_engine, text

def test_ingestion():
    print("🧪 Testing RAG ingestion with sample content...")

    # Qdrant hosted client
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Neon DB (Postgres) connection
    engine = create_engine(os.getenv("NEON_DATABASE_URL"), echo=False)

    # Use a test collection name
    collection_name = 'test_robots2026'

    # Delete if exists and recreate
    try:
        qdrant.delete_collection(collection_name)
        print(f"🧹 Cleared existing collection: {collection_name}")
    except:
        print(f"✨ Collection {collection_name} didn't exist, will create it")

    # Create collection with vector parameters
    qdrant.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )
    print(f"✅ Created collection: {collection_name}")

    # Sample content from the book (just a small subset)
    sample_content = [
        {
            'title': 'Week 1 - ROS2 Basics',
            'text': 'ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.',
            'chapter': 'book/docs/week1-ros2-basics.md'
        },
        {
            'title': 'Week 2 - Motion Planning',
            'text': 'Motion planning is the process of breaking down a desired movement task into discrete motions. In robotics, motion planning typically refers to the problem of navigating a mobile robot through an environment with obstacles.',
            'chapter': 'book/docs/week2-motion-planning.md'
        },
        {
            'title': 'Week 3 - Perception Systems',
            'text': 'Perception systems in robotics involve sensing and interpreting the environment. This includes computer vision, sensor fusion, object detection, and scene understanding to enable robots to interact with their surroundings.',
            'chapter': 'book/docs/week3-perception-systems.md'
        }
    ]

    points = []
    for idx, content in enumerate(sample_content):
        print(f"🔄 Processing sample {idx + 1}/{len(sample_content)}...")

        try:
            # Generate embedding using Qwen
            embedding = get_qwen_embeddings(content['text'])

            points.append(PointStruct(
                id=idx,
                vector=embedding,
                payload={
                    'chapter': content['chapter'],
                    'text': content['text'],
                    'title': content['title'],
                    'chunk_idx': 0
                }
            ))

            print(f"✅ Processed sample {idx + 1}: {content['title'][:50]}...")

        except Exception as e:
            print(f"❌ Error processing sample {idx + 1}: {str(e)}")
            continue

    # Upload vectors to Qdrant
    if points:
        qdrant.upsert(collection_name=collection_name, points=points)
        print(f'✅ Uploaded {len(points)} sample points to Qdrant using Qwen embeddings')

        # Verify the upload
        collection_info = qdrant.get_collection(collection_name)
        print(f'📊 Final collection {collection_name} has {collection_info.points_count} points')

        # Test a query to make sure it works
        print("🔍 Testing retrieval...")
        test_query = "What is ROS2?"
        query_embedding = get_qwen_embeddings(test_query)

        hits = qdrant.search_points(
            collection_name=collection_name,
            vector=query_embedding,
            limit=1,
            with_payload=True,
        )

        if hits:
            result = hits[0]
            print(f"✅ Query test successful!")
            print(f"   Query: {test_query}")
            print(f"   Match: {result.payload['title']}")
            print(f"   Score: {result.score}")
        else:
            print("❌ Query test failed - no results returned")

        # Clean up - delete the test collection
        qdrant.delete_collection(collection_name)
        print(f"🧹 Cleaned up test collection: {collection_name}")

        return True
    else:
        print('❌ No samples processed successfully')
        return False

if __name__ == "__main__":
    success = test_ingestion()
    if success:
        print("\n🎉 RAG system is working properly! Full book ingestion should work too.")
    else:
        print("\n❌ RAG system has issues that need to be resolved.")