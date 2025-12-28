# API Contract: Book Content Embeddings Pipeline

**Feature**: 002-book-embeddings-qdrant
**Date**: 2025-12-19
**Version**: 1.0

## Overview

This document specifies the API contracts for the book content embeddings pipeline. The pipeline is primarily a command-line utility, but this document outlines the internal service contracts and potential future REST endpoints.

## Service Contracts

### Content Extraction Service

#### Interface
```python
def extract_text_from_url(url: str) -> BookPage:
    """
    Extracts clean text content from a web page at the given URL.
    
    Args:
        url (str): The URL of the book page to extract content from
        
    Returns:
        BookPage: An object containing the extracted content and metadata
        
    Raises:
        ExtractionError: If content cannot be extracted from the URL
    """
```

#### Input
- `url`: A valid URL string pointing to a book page

#### Output
- `BookPage` object with:
  - `id`: Unique identifier for the page
  - `url`: The source URL
  - `title`: Page title
  - `extracted_text`: Clean text content
  - `headings`: List of headings found in the page

### Text Preprocessing Service

#### Interface
```python
def chunk_text(content: str, max_chunk_size: int = 768) -> List[TextChunk]:
    """
    Chunks a text content into smaller segments suitable for embedding.
    
    Args:
        content (str): The text content to chunk
        max_chunk_size (int): Maximum number of tokens per chunk (default 768)
        
    Returns:
        List[TextChunk]: List of text chunks derived from the content
    """
```

#### Input
- `content`: The text content to be chunked
- `max_chunk_size`: Maximum token count for each chunk

#### Output
- List of `TextChunk` objects with:
  - `id`: Unique identifier for the chunk
  - `content`: The chunked text
  - `start_pos`: Start position in original text
  - `end_pos`: End position in original text
  - `chunk_index`: Sequential index of the chunk
  - `tokens_count`: Number of tokens in the chunk

### Embedding Generation Service

#### Interface
```python
def generate_embedding(text_chunk: TextChunk, model_name: str = "embed-english-light-v2.0") -> Embedding:
    """
    Generates a vector embedding for a text chunk using the specified model.
    
    Args:
        text_chunk (TextChunk): The text chunk to embed
        model_name (str): Name of the embedding model to use
        
    Returns:
        Embedding: An object containing the generated vector
    """
```

#### Input
- `text_chunk`: The TextChunk object containing text to embed
- `model_name`: Name of the Cohere embedding model to use

#### Output
- `Embedding` object with:
  - `id`: Unique identifier for the embedding
  - `chunk_id`: Reference to the source TextChunk
  - `vector`: The embedding vector (array of floats)
  - `model_name`: Name of the model used

### Vector Storage Service

#### Interface
```python
def store_embedding(embedding: Embedding, metadata: Metadata) -> bool:
    """
    Stores an embedding in the vector database with associated metadata.
    
    Args:
        embedding (Embedding): The embedding to store
        metadata (Metadata): Associated metadata to store with the embedding
        
    Returns:
        bool: True if storage was successful, False otherwise
    """
```

#### Input
- `embedding`: The Embedding object to store
- `metadata`: The Metadata object to associate with the embedding

#### Output
- Boolean indicating success or failure of the storage operation

### Pipeline Orchestration Service

#### Interface
```python
def run_pipeline(book_urls: List[str], book_id: str) -> PipelineJob:
    """
    Runs the complete pipeline for a book with the given URLs.
    
    Args:
        book_urls (List[str]): List of URLs containing the book content
        book_id (str): Unique identifier for the book
        
    Returns:
        PipelineJob: An object containing job status and results
    """
```

#### Input
- `book_urls`: List of URLs containing the book content
- `book_id`: Unique identifier for the book being processed

#### Output
- `PipelineJob` object with:
  - `id`: Unique identifier for the job
  - `book_id`: Reference to the book being processed
  - `status`: Current status of the job
  - `pages_count`: Total number of pages to process
  - `pages_processed`: Number of pages processed
  - `pages_failed`: Number of pages that failed
  - `created_at`: Timestamp when job was created
  - `started_at`: Timestamp when job started
  - `completed_at`: Timestamp when job completed (if finished)
  - `error_message`: Error message if job failed