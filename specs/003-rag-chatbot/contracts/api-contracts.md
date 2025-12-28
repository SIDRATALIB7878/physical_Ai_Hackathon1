# API Contract: RAG Chatbot Integration for Book Content

**Feature**: 003-rag-chatbot
**Date**: 2025-12-19
**Version**: 1.0

## Overview

This document specifies the API contracts for the RAG chatbot integration that connects to the existing book embeddings pipeline.

## API Endpoints

### 1. Create New Session
```
POST /api/chat/new-session
```

#### Request
```json
{
  "book_id": "string"
}
```

#### Response (Success 200)
```json
{
  "session_id": "string",
  "created_at": "datetime"
}
```

#### Response (Error 400)
```json
{
  "error": "string",
  "message": "string"
}
```

#### Response (Error 404)
```json
{
  "error": "Book not found",
  "message": "The specified book_id does not exist"
}
```

---

### 2. Process User Query
```
POST /api/chat/query
```

#### Request
```json
{
  "session_id": "string",
  "query": "string",
  "book_id": "string"
}
```

#### Response (Success 200)
```json
{
  "response": "string",
  "sources": [
    {
      "chunk_id": "string",
      "content": "string",
      "relevance_score": "float",
      "source_url": "string",
      "section_heading": "string"
    }
  ],
  "timestamp": "string"
}
```

#### Response (Error 400)
```json
{
  "error": "Invalid request",
  "message": "string"
}
```

---

### 3. Get Conversation History
```
GET /api/chat/history/{session_id}
```

#### Response (Success 200)
```json
{
  "history": [
    {
      "turn_id": "string",
      "query": "string",
      "response": "string",
      "timestamp": "string"
    }
  ]
}
```

#### Response (Error 404)
```json
{
  "error": "Session not found",
  "message": "The specified session_id does not exist"
}
```

---

### 4. Switch Book in Session
```
POST /api/chat/switch-book
```

#### Request
```json
{
  "session_id": "string",
  "new_book_id": "string"
}
```

#### Response (Success 200)
```json
{
  "status": "Book switched successfully",
  "message": "Session now uses new book content"
}
```

#### Response (Error 404)
```json
{
  "error": "Book or Session not found",
  "message": "Either the session_id or new_book_id does not exist"
}
```

## Service Contracts

### Session Management Service

#### Interface
```python
def create_session(book_id: str) -> ChatSession:
    """
    Creates a new chat session for a specific book.
    
    Args:
        book_id (str): The identifier of the book to chat with
        
    Returns:
        ChatSession: The created session with a unique session_id
        
    Raises:
        ValueError: If the book_id is invalid or doesn't exist
    """
```

#### Interface
```python
def get_session(session_id: str) -> ChatSession:
    """
    Retrieves an existing chat session.
    
    Args:
        session_id (str): The session identifier
        
    Returns:
        ChatSession: The session data
        
    Raises:
        ValueError: If the session_id is invalid or doesn't exist
    """
```

---

### Vector Search Service

#### Interface
```python
def search(query: str, book_id: str, top_k: int = 5) -> List[RetrievedChunk]:
    """
    Performs semantic search in the Qdrant vector database.
    
    Args:
        query (str): The user's query to search for
        book_id (str): The book to search within
        top_k (int): Number of top results to return (default 5)
        
    Returns:
        List[RetrievedChunk]: List of relevant text chunks with metadata
    """
```

---

### Response Generation Service

#### Interface
```python
def generate_response(query: str, context: str) -> str:
    """
    Generates a response using the Cohere language model.
    
    Args:
        query (str): The user's question
        context (str): Retrieved context to base the response on
        
    Returns:
        str: The generated response
    """
```