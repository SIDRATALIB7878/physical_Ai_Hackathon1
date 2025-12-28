# Quickstart: Book Content Embeddings Pipeline

**Feature**: 002-book-embeddings-qdrant
**Date**: 2025-12-19

## Overview

This guide will help you set up and run the book content embeddings pipeline. The pipeline takes book content from web-accessible URLs, extracts text, generates embeddings using Cohere, and stores them in Qdrant for future retrieval.

## Prerequisites

- Python 3.11 or higher
- Cohere API key
- Qdrant Cloud account and API key
- Access to book content via URLs

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
pip install requests beautifulsoup4 cohere qdrant-client python-dotenv pytest
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
BOOK_URLS=comma,separated,list,of,book,urls
CHUNK_SIZE=768  # Number of tokens per chunk
PROCESSING_RATE=100  # Pages per hour
```

## Usage

### 1. Run the Pipeline

```bash
python -m src.cli.main --book-urls "https://example-book.com/page1,https://example-book.com/page2" --book-id "example-book"
```

### 2. Using Configuration File

Alternatively, create a config file:

```bash
python -m src.cli.main --config config.json
```

Where `config.json` contains:

```json
{
  "book_urls": ["https://example-book.com/page1", "https://example-book.com/page2"],
  "book_id": "example-book",
  "chunk_size": 768,
  "processing_rate": 100
}
```

## Pipeline Stages

1. **Content Extraction**: Downloads and extracts text from book pages
2. **Text Preprocessing**: Splits content into appropriately sized chunks
3. **Embedding Generation**: Creates vector embeddings using Cohere
4. **Vector Storage**: Stores embeddings in Qdrant with metadata

## Monitoring

The pipeline outputs logs to indicate progress. At the end, you'll see a summary like:

```
Pipeline completed successfully:
- Total pages processed: 50
- Pages with errors: 0
- Embeddings generated: 120
- Time elapsed: 00:15:30
```

## Next Steps

After running the pipeline, you'll have embeddings stored in Qdrant that can be used for semantic search in RAG applications. Check out the advanced usage documentation for query examples.