"""
Verification module for Qdrant and Neon connections.
"""
from qdrant_client import QdrantClient
from sqlalchemy import create_engine, text
import os


def verify_all():
    """
    Verify both Qdrant and Neon DB connections.
    """
    result = {"qdrant": None, "neon": None}

    # Test Qdrant connection
    try:
        qdrant = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        collections = qdrant.get_collections().collections
        result["qdrant"] = {"status": "ok", "collections": [c.name for c in collections]}
        print(f"✅ Qdrant connection: {result['qdrant']}")
    except Exception as e:
        result["qdrant"] = {"status": "error", "error": str(e)}
        print(f"❌ Qdrant connection: {result['qdrant']}")
        raise e

    # Test Neon DB connection
    try:
        engine = create_engine(os.getenv("NEON_DATABASE_URL"), echo=False)
        with engine.connect() as conn:
            test = conn.execute(text("SELECT 1")).fetchone()
            if test and test[0] == 1:
                result["neon"] = {"status": "ok"}
                print(f"✅ Neon connection: {result['neon']}")
            else:
                result["neon"] = {"status": "error", "error": "Unexpected result"}
                print(f"❌ Neon connection: {result['neon']}")
                raise Exception("Neon DB unexpected result")
    except Exception as e:
        result["neon"] = {"status": "error", "error": str(e)}
        print(f"❌ Neon connection: {result['neon']}")
        raise e

    return result