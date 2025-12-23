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
from pathlib import Path
from dotenv import load_dotenv
from rag.ingest import ingest_book
from rag.verify import verify_all


def main():
    # Step 0: Load environment variables
    print("📦 Loading environment variables...")
    load_dotenv()

    # Step 1: Verify cloud infrastructure connectivity
    print("🔍 Verifying cloud infrastructure connectivity...")
    verify_all()
    print("✅ Neon DB, Qdrant, Qwen API reachable from cloud")

    # Step 2: Ingest book into cloud RAG (Qdrant + Neon)
    print("📚 Ingesting book content into cloud RAG...")
    ingest_book(
        docs_dir="book/docs",
        collection="robots2026",
        chunk_size=512,
        overlap=64,
        id_strategy="sha256"
    )
    print("✅ Book content ingested into Qdrant / Neon")

    # Step 3: Update chat frontend to point to cloud backend
    print("🌐 Updating chat frontend to point to cloud backend...")

    # Define the cloud backend URL - this should be set based on your deployment
    # For example, this could be a Railway, Render, or other cloud deployment URL
    cloud_backend_url = os.getenv("CLOUD_BACKEND_URL", "https://your-cloud-backend.onrender.com")

    chat_file = Path("book/static/chat/index.html")
    content = chat_file.read_text(encoding='utf-8')

    # Replace the cloud backend URL placeholder
    updated_content = content.replace(
        ' : "https://robots2026.up.railway.app";',
        f' : "{cloud_backend_url}";'
    )

    chat_file.write_text(updated_content, encoding='utf-8')
    print(f"✅ Chat frontend updated to use cloud backend: {cloud_backend_url}")

    # Step 4: Optional: rebuild Docusaurus site to reflect changes
    print("🔨 Rebuilding Docusaurus site...")
    try:
        subprocess.run(["cd book && npm run build"], shell=True, check=True)
        print("✅ Docusaurus site rebuilt successfully")
    except subprocess.CalledProcessError:
        print("⚠️  Docusaurus build failed - please check your setup")

    print("🎉 RAG chat fully configured for cloud deployment!")


if __name__ == "__main__":
    main()