# Book Content Embeddings Pipeline Guide

## Overview

The Book Content Embeddings Pipeline is a command-line tool that processes book content from web-accessible URLs, extracts text, generates embeddings using Cohere, and stores them in a Qdrant vector database for later retrieval. This enables semantic search capabilities for RAG (Retrieval-Augmented Generation) applications.

## Architecture

The pipeline consists of the following components:

1. **Content Extraction Service**: Downloads and extracts clean text from book pages
2. **Embedding Generation Service**: Creates vector embeddings using Cohere
3. **Vector Storage Service**: Stores embeddings in Qdrant with metadata
4. **Pipeline Orchestration Service**: Coordinates the entire process
5. **Command-Line Interface**: Provides user interaction

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant Cloud account and API key (or local instance)

## Installation

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Configuration

Create a `.env` file in the project root with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
CHUNK_SIZE=768  # Number of tokens per chunk
PROCESSING_RATE=100  # Pages per hour
```

## Usage

### Basic Usage

```bash
python -m src.cli.main --book-urls "https://example-book.com/page1,https://example-book.com/page2" --book-id "example-book"
```

### Using Configuration File

Create a `config.json` file:

```json
{
  "book_urls": [
    "https://example-book.com/page1",
    "https://example-book.com/page2"
  ],
  "book_id": "example-book",
  "chunk_size": 768,
  "processing_rate": 100
}
```

Then run:

```bash
python -m src.cli.main --config config.json --book-id "example-book"
```

### Additional Options

- `--verbose`: Enable verbose logging
- `--chunk-size`: Override the chunk size from config
- `--processing-rate`: Override the processing rate from config

## Pipeline Stages

1. **Content Extraction**: Downloads and extracts text from book pages
2. **Text Preprocessing**: Splits content into appropriately sized chunks (respecting the CHUNK_SIZE configuration)
3. **Embedding Generation**: Creates vector embeddings using Cohere's embed-english-light-v2.0 model
4. **Vector Storage**: Stores embeddings in Qdrant with metadata (URL, headings, etc.)

## Duplicate Content Handling

The pipeline detects duplicate content and flags it in the metadata, but stores each occurrence separately as specified in the research.

## Performance Metrics

The pipeline reports the following metrics:
- Pages processed per hour
- Embeddings generated per second
- Total processing time
- Number of duplicates found

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, reduce the PROCESSING_RATE in your configuration.
2. **Memory Issues**: For large books, consider processing in smaller batches.
3. **Content Extraction Issues**: Some websites may block automated requests. Ensure your requests respect robots.txt.

### Logging

The pipeline logs to stdout with different levels of verbosity. Use the `--verbose` flag for detailed logs.

## Development

### Running Tests

```bash
# Run all tests
python -m pytest

# Run validation script
python tests/test_pipeline_validation.py
```

### Code Structure

```
src/
├── models/           # Data models (BookPage, Embedding, etc.)
├── services/         # Core services (Content extraction, embedding, etc.)
├── cli/             # Command-line interface
└── lib/             # Utility libraries (configuration, logging, etc.)

tests/
├── unit/            # Unit tests
├── integration/     # Integration tests
├── contract/        # Contract tests
└── test_pipeline_validation.py  # Validation script
```