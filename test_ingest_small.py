import os
from rag.qwen_embeddings import get_qwen_embeddings
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

v = get_qwen_embeddings("What is Physical AI?")

qc = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"],
)

qc.create_collection(
    collection_name="test_robots2026",
    vectors_config=VectorParams(
        size=len(v),
        distance=Distance.COSINE
    ),
)

qc.upsert(
    collection_name="test_robots2026",
    points=[{
        "id": 1,
        "vector": v,
        "payload": {"text": "test"}
    }]
)

print("count:", qc.count("test_robots2026"))
