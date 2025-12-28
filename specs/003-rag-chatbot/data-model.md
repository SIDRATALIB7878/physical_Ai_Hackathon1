# Data Model: RAG Chatbot Integration for Book Content

**Feature**: 003-rag-chatbot
**Date**: 2025-12-19
**Status**: Draft

## Entities

### ChatSession
- **Description**: Represents a user's conversation with a book, containing session ID, selected book, and conversation history
- **Fields**:
  - `session_id` (string): Unique identifier for the session
  - `user_id` (string): Identifier for the user (optional)
  - `book_id` (string): Identifier for the selected book
  - `created_at` (datetime): When the session was created
  - `last_accessed` (datetime): When the session was last used
  - `conversation_history` (list of ConversationTurn): History of exchanges
  - `active` (boolean): Whether the session is currently active

### QueryRequest
- **Description**: Represents a user's question with metadata like query text, session ID, book ID, and timestamp
- **Fields**:
  - `query_id` (string): Unique identifier for the query
  - `session_id` (string): Session this query belongs to
  - `query_text` (string): The user's question
  - `book_id` (string): Book ID to search within
  - `timestamp` (datetime): When the query was made
  - `user_context` (string): Additional context from conversation history

### RetrievedChunk
- **Description**: Represents relevant text chunks retrieved from vector search with relevance scores
- **Fields**:
  - `chunk_id` (string): ID of the retrieved chunk
  - `content` (string): The text content of the chunk
  - `relevance_score` (float): Similarity score from vector search
  - `source_url` (string): URL where the content originated
  - `section_heading` (string): Heading of the section containing the chunk
  - `page_number` (integer): Page number if applicable
  - `metadata` (dict): Additional metadata from Qdrant

### ChatResponse
- **Description**: Represents the system's response to a user query with generated text and source citations
- **Fields**:
  - `response_id` (string): Unique identifier for the response
  - `session_id` (string): Session this response belongs to
  - `query_id` (string): ID of the original query
  - `generated_text` (string): The AI-generated response
  - `sources` (list of RetrievedChunk): Sources used to generate the response
  - `timestamp` (datetime): When the response was generated
  - `response_time_ms` (integer): Time taken to generate the response

### ConversationTurn
- **Description**: Represents a single exchange (user query + system response) within a session
- **Fields**:
  - `turn_id` (string): Unique identifier for the turn
  - `session_id` (string): Session this turn belongs to
  - `query_request` (QueryRequest): The user's query
  - `chat_response` (ChatResponse): The system's response
  - `timestamp` (datetime): When the turn occurred

## Relationships

- ChatSession 1 --- * ConversationTurn (One session has many conversation turns)
- ConversationTurn 1 --- 1 QueryRequest (One conversation turn has one query)
- ConversationTurn 1 --- 1 ChatResponse (One conversation turn has one response)
- ChatResponse 1 --- * RetrievedChunk (One response uses many chunks as sources)

## Validation Rules

- ChatSession:
  - session_id must be unique
  - book_id must be valid and exist in the system
  - created_at must be set on creation

- QueryRequest:
  - query_text must not be empty
  - session_id and book_id must be valid
  - timestamp must be set on creation

- RetrievedChunk:
  - relevance_score must be between 0 and 1
  - content must not be empty

- ChatResponse:
  - generated_text must not be empty
  - sources list must not be empty
  - response_time_ms must be positive

- ConversationTurn:
  - Must have both a query and response
  - timestamp must be set on creation