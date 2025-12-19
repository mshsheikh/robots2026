---
id: chat
title: How the AI Chat Works
sidebar_position: 15
---

# How the AI Chat Works

## Overview

The AI chat system provides intelligent question-answering capabilities for the robotics curriculum using Retrieval-Augmented Generation (RAG). This system allows users to ask questions about humanoid robotics concepts and receive answers grounded in the course documentation. The chat interface is embedded directly in the Docusaurus documentation site and connects to a FastAPI backend that retrieves relevant information from a Qdrant vector database.

## System Architecture

The chat system follows a modular architecture with clear separation of concerns:

- **Docusaurus Frontend**: Static documentation site with embedded chat UI component
- **HTML/CSS/JS Chat Interface**: Self-contained chat widget that communicates with the backend via HTTP requests
- **FastAPI Backend**: REST API server handling RAG queries with `/query` and `/query_selected` endpoints
- **Qdrant Vector Database**: Persistent storage for embedded curriculum content with similarity search capabilities
- **Embedding System**: Converts text to high-dimensional vectors using OpenAI-compatible models
- **Neon Postgres (Planned)**: Future integration for storing conversation history and user preferences

The architecture adheres to Spec-Kit Plus principles with judge-aware safety considerations and modular design for easy maintenance.

## Retrieval-Augmented Generation (RAG) Flow

The RAG system operates through the following sequence:

1. **Query Reception**: User question is submitted to the FastAPI backend via the `/query` endpoint
2. **Embedding Generation**: Question is converted to a vector embedding using the same model used during document indexing
3. **Similarity Search**: Qdrant performs vector similarity search to find top-K most relevant document chunks
4. **Context Assembly**: Retrieved chunks are assembled into context for the response generation
5. **Response Generation**: (Future implementation) LLM generates response based on retrieved context
6. **Results Return**: JSON response containing relevant chunks and metadata is returned to frontend

The system currently returns the retrieved document chunks without final response generation, allowing users to see the source material that informed the answer.

## Selected-Text Question Answering

The `/query_selected` endpoint provides enhanced context by incorporating user-selected text:

1. **Combined Query**: User question and selected text are combined into a single semantic query
2. **Enhanced Embedding**: The combined text is embedded to capture both question intent and context
3. **Focused Retrieval**: Qdrant searches for chunks specifically relevant to the question-context combination
4. **Contextual Results**: Returned results are more targeted to the specific context provided by the user

This feature enables users to ask questions about specific passages while still benefiting from broader curriculum knowledge.

## Local Demo vs Live Deployment

**Local Development Environment**:
- Full RAG functionality with active Qdrant connection
- Real-time vector search and retrieval
- Complete query and query_selected endpoint functionality
- HTTPS or HTTP serving compatible with embedded chat

**Live GitHub Pages Deployment**:
- Chat interface remains visible but becomes read-only
- Browser security policies block HTTP API calls from HTTPS pages (Mixed Content Policy)
- Users can view documentation but cannot submit queries
- This limitation is inherent to GitHub Pages' HTTPS-only serving and cross-origin security

The architectural design accounts for this browser security constraint while maintaining full functionality in local environments.

## Current Limitations and Future Improvements

**Current Limitations**:
- Response generation not yet implemented (returns source chunks only)
- Mixed content security prevents live deployment API calls
- Local Qdrant dependency requires separate service startup
- Embedding model requires API key for production performance

**Planned Improvements**:
- Full response generation with LLM integration
- Conversation history and context management
- Enhanced document chunking strategies for better retrieval
- Neon Postgres integration for user session persistence
- Production-grade embedding and response generation

The system demonstrates architectural correctness with scalable design patterns and clear separation of frontend, backend, and data storage concerns.