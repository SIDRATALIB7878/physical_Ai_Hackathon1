# Research: Book Content Embeddings Pipeline

**Feature**: 002-book-embeddings-qdrant
**Date**: 2025-12-19
**Status**: Complete

## Research Summary

This research addresses key unknowns from the feature specification and provides the technical foundation for implementing the book content embeddings pipeline.

## 1. Processing Rate Decision

**Decision**: Medium volume processing - 100 pages per hour
**Rationale**: Balances implementation complexity with realistic book processing needs. Most books contain fewer than 1000 pages, so this rate allows for processing a complete book within 10 hours. This rate respects Cohere API rate limits and Qdrant Cloud Free Tier constraints.
**Alternatives considered**: 
- Low volume (10 pages/hr): Too slow for practical use
- High volume (1000 pages/hr): Would require complex rate limiting and likely exceed free tier limits

## 2. Duplicate Content Handling

**Decision**: Flag duplicates but store separately (Option C)
**Rationale**: Maintains the original structure and URL mapping while identifying redundancies. This approach preserves the semantic relationship between content and its source location in the book, which is important for accurate retrieval in future RAG applications.
**Alternatives considered**:
- Store separately: Simplest but doesn't address storage efficiency
- Merge duplicates: More complex implementation and complicates URL mapping

## 3. Text Chunking Strategy

**Decision**: Medium chunks of 512-1024 tokens
**Rationale**: Provides a good balance of precision and efficiency. Most book paragraphs fall within this range, allowing for contextual understanding while maintaining search precision. This size works well with Cohere's embedding models and fits within typical context windows.
**Alternatives considered**:
- Small chunks (256-512): Could lose context within longer paragraphs
- Large chunks (1024-2048): May dilute semantic meaning and increase costs

## 4. Web Content Extraction Approach

**Decision**: Use Beautiful Soup for parsing HTML content, combined with custom preprocessing to extract clean text
**Rationale**: Beautiful Soup is a robust and widely-used Python library for parsing HTML. When combined with custom filtering to remove navigation, ads, and other non-content elements, it provides clean text extraction suitable for embeddings.
**Alternatives considered**:
- Selenium: Good for dynamic content but more complex and slower
- Newspaper3k: Good for articles but not necessarily optimized for book content
- Custom regex: Less reliable and harder to maintain

## 5. Qdrant Vector Database Configuration

**Decision**: Use Qdrant Cloud Free Tier with a single collection per book project
**Rationale**: The free tier provides 5 storage points and 1M vectors, which is adequate for most book projects. A collection per book allows for easy organization and retrieval while staying within free tier limits.
**Configuration details**:
- Collection name: book_embeddings_{book_id}
- Vector size: 1024 (for Cohere embeddings)
- Distance metric: Cosine

## 6. Cohere Embedding Model Selection

**Decision**: Use Cohere's embed-english-light-v2.0 model
**Rationale**: This model provides good performance for English text and is suitable for book content. It's also more cost-effective than the larger models while still providing high-quality embeddings for semantic search.
**Alternatives considered**:
- embed-english-v2.0: Higher quality but more expensive
- embed-multilingual-v2.0: Unnecessary for English-focused book content

## 7. Pipeline Architecture

**Decision**: Modular pipeline with separate components for each stage
**Rationale**: This approach makes the pipeline more testable, maintainable, and reusable. Each component can be tested independently and updated without affecting others.
**Architecture stages**:
1. URL crawler/collector: Gather all book page URLs
2. Content extractor: Extract clean text content from each page
3. Content preprocessor: Chunk text and prepare for embedding
4. Embedding generator: Create vector embeddings using Cohere
5. Vector storage: Store embeddings in Qdrant with metadata

## 8. Error Handling and Monitoring

**Decision**: Implement comprehensive error handling and logging
**Rationale**: The pipeline involves multiple external services (web scraping, Cohere API, Qdrant), each of which can fail independently. Proper error handling and logging are essential for debugging and monitoring.
**Approach**:
- Individual page failure shouldn't stop the entire pipeline
- Log all errors with sufficient details for debugging
- Provide summary of success/failure rates at the end

## 9. Configuration Management

**Decision**: Use environment variables for API keys and configuration parameters
**Rationale**: Environment variables provide a secure and flexible way to manage configuration across different environments without hardcoding sensitive information.
**Parameters to configure**:
- Cohere API key
- Qdrant API key and URL
- Processing rate limits
- Chunk size parameters