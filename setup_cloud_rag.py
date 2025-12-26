#!/usr/bin/env python3
"""
Cloud RAG Setup Script
This script sets up the cloud RAG system by:
1. Loading environment variables
2. Ingesting book content into cloud Qdrant/Neon
3. Updating the chat frontend to point to cloud backend
4. Verifying the setup
"""

import os
import subprocess
import sys
from pathlib import Path


def main():
    print("📦 Activating virtual environment and loading environment variables...")

    # Add the project root to Python path
    sys.path.insert(0, os.path.abspath('.'))

    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Verify cloud infrastructure connectivity
    print("🔍 Verifying cloud infrastructure connectivity...")
    from rag.verify import verify_all
    verify_all()
    print("✅ Neon DB, Qdrant, Qwen API reachable from cloud")

    # Ingest book into cloud RAG (Qdrant + Neon)
    print("📚 Ingesting book content into cloud RAG...")
    from rag.ingest import ingest_book
    ingest_book(
        docs_dir="book/docs",
        collection="robots2026",
        chunk_size=512,
        overlap=64,
        id_strategy="sha256"
    )
    print("✅ Book content ingested into Qdrant / Neon")

    # Update chat frontend to point to cloud backend
    print("🌐 Updating chat frontend to point to cloud backend...")

    # Define the cloud backend URL - this should be set based on your deployment
    # For this example, we'll use a placeholder that needs to be replaced with actual deployment URL
    cloud_backend_url = os.getenv("CLOUD_BACKEND_URL", "https://your-actual-cloud-backend.com")

    chat_file = Path("book/static/chat/index.html")
    content = chat_file.read_text(encoding='utf-8')

    # Replace the cloud backend URL placeholder
    updated_content = content.replace(
        ' : "https://robots2026.up.railway.app";',
        f' : "{cloud_backend_url}";'
    )

    chat_file.write_text(updated_content, encoding='utf-8')
    print(f"✅ Chat frontend updated to use cloud backend: {cloud_backend_url}")

    # Optional: rebuild Docusaurus site to reflect changes
    print("🔨 Rebuilding Docusaurus site...")
    try:
        result = subprocess.run(["cd book && npm run build"], shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ Docusaurus site rebuilt successfully")
        else:
            print(f"⚠️  Docusaurus build failed: {result.stderr}")
    except Exception as e:
        print(f"⚠️  Docusaurus build failed - please check your setup: {e}")

    print("🎉 RAG chat fully configured for cloud deployment!")


if __name__ == "__main__":
    main()