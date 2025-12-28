# Quickstart: RAG Chatbot Integration for Book Content

**Feature**: 003-rag-chatbot
**Date**: 2025-12-19

## Overview

This guide will help you set up and run the RAG chatbot that connects to your book embeddings pipeline. The chatbot enables conversational access to book content through semantic search and AI-powered responses.

## Prerequisites

- Python 3.11 or higher
- Completed book embeddings pipeline (002-book-embeddings-qdrant)
- Cohere API key
- Qdrant Cloud account and API key
- Redis server for session management
- Existing book content processed and stored in Qdrant

## Setup

### 1. Clone and Navigate to the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

If no requirements.txt exists yet, install the required packages:

```bash
pip install fastapi uvicorn cohere qdrant-client python-jose redis pytest
```

### 4. Configure Environment Variables

Update your `.env` file with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
REDIS_URL=redis://localhost:6379
SESSION_SECRET_KEY=your_session_secret_key_here
</environment>

## Running the Service

### 1. Start the API Server

```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

### 2. Using Docker (Alternative)

```bash
docker build -t rag-chatbot .
docker run -p 8000:8000 rag-chatbot
```

## API Usage

### 1. Create a New Chat Session

```bash
curl -X POST "http://localhost:8000/api/chat/new-session" \
  -H "Content-Type: application/json" \
  -d '{"book_id": "your-book-id"}'
```

### 2. Send a Query

```bash
curl -X POST "http://localhost:8000/api/chat/query" \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-id", 
    "query": "What is the main theme of this book?",
    "book_id": "your-book-id"
  }'
```

### 3. Get Conversation History

```bash
curl -X GET "http://localhost:8000/api/chat/history/{session_id}"
```

## Integration with Existing Pipeline

The RAG chatbot directly integrates with the existing book embeddings pipeline:

1. It uses the same Qdrant vector database populated by the 002-book-embeddings-qdrant pipeline
2. It uses the same Cohere embedding model for query encoding
3. It leverages the book_id organization established during the embedding process
4. It accesses the same metadata (URLs, headings) stored with the embeddings

## Testing

Run the test suite:

```bash
python -m pytest tests/
```

To run a specific test type:

```bash
python -m pytest tests/unit/
python -m pytest tests/integration/
python -m pytest tests/e2e/
```

## Next Steps

After setting up the API, consider:

1. Adding a web-based chat interface
2. Implementing rate limiting for production use
3. Adding analytics to track query patterns and user satisfaction
4. Implementing an admin panel to manage books and sessions