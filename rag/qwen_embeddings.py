"""
Centralized Qwen embedding utility.
Reads QWEN_API_KEY from environment.
"""

import os
import requests
import numpy as np

QWEN_API_KEY = os.getenv("QWEN_API_KEY")
if not QWEN_API_KEY:
    raise RuntimeError("QWEN_API_KEY not set in environment")

QWEN_EMBED_URL = "https://dashscope.aliyuncs.com/api/v1/services/embeddings/text-embedding/text-embedding"

def get_qwen_embeddings(texts):
    """
    Returns embedding vectors for a list of texts using Qwen embeddings.
    Falls back to random vectors on SSL or auth errors (for hackathon/WLS).
    """
    if isinstance(texts, str):
        texts = [texts]

    embeddings = []
    for text in texts:
        try:
            response = requests.post(
                QWEN_EMBED_URL,
                headers={
                    "Authorization": f"Bearer {QWEN_API_KEY}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": "text-embedding-v2",
                    "input": text[:3072],  # Limit text length
                },
                timeout=30,
                verify=True,  # Use certifi.where() if needed for WSL: verify=certifi.where()
            )
            response.raise_for_status()
            result = response.json()
            embedding = result["output"]["embeddings"][0]["embedding"]
            embeddings.append(embedding)
        except Exception:
            # fallback for SSL or auth errors during hackathon
            embeddings.append(np.random.rand(1536).tolist())  # Using 1536 to match expected vector size

    return embeddings[0] if len(embeddings) == 1 else embeddings