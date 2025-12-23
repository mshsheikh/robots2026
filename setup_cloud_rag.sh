#!/bin/bash

# Cloud RAG Setup Script
set -e  # Exit on any error

echo "📦 Activating virtual environment..."
source venv/bin/activate

echo "🔑 Loading environment variables..."
set -a
source .env
set +a

echo "🔍 Verifying cloud infrastructure connectivity..."
python -c "
import os
from dotenv import load_dotenv
load_dotenv()
from rag.verify import verify_all
verify_all()
print('✅ Neon DB, Qdrant, Qwen API reachable from cloud')
"

echo "📚 Ingesting book content into cloud RAG..."
python -c "
import os
from dotenv import load_dotenv
load_dotenv()
from rag.ingest import ingest_book
ingest_book(
    docs_dir='book/docs',
    collection='robots2026',
    chunk_size=512,
    overlap=64,
    id_strategy='sha256'
)
print('✅ Book content ingested into Qdrant / Neon')
"

echo "🌐 Updating chat frontend to point to cloud backend..."

# Define the cloud backend URL - this should be set based on your deployment
# For this example, we'll use a placeholder that needs to be replaced with actual deployment URL
CLOUD_BACKEND_URL=\${CLOUD_BACKEND_URL:-\"https://your-actual-cloud-backend.com\"}

# Update the chat frontend
sed -i \"s| : \\\"https://<your-cloud-backend>\"| : \\\"$CLOUD_BACKEND_URL\"|g\" book/static/chat/index.html

echo \"✅ Chat frontend updated to use cloud backend: $CLOUD_BACKEND_URL\"

echo "🎉 RAG chat fully configured for cloud deployment!"