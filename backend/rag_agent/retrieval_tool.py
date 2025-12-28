"""Retrieval tool for the RAG agent that queries Qdrant for relevant book content."""
from typing import List, Dict, Any
import json
from src.services.vector_storage import VectorStorageService
from src.services.embedding_generator import EmbeddingGeneratorService
from src.models.text_chunk import TextChunk
from src.lib.config import Config


class RetrievalTool:
    """Tool for retrieving relevant book content from Qdrant vector database."""

    def __init__(self):
        """Initialize the retrieval tool with necessary services."""
        self.vector_storage = VectorStorageService()
        self.embedding_generator = EmbeddingGeneratorService()

    def retrieve_chunks(self, query: str, book_id: str = "default_book", limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant text chunks from Qdrant based on the query.

        Args:
            query: The query to search for in the vector database
            book_id: ID of the book to search within
            limit: Maximum number of results to return

        Returns:
            List of relevant chunks with their metadata
        """
        try:
            # Create a text chunk from the query with all required fields
            query_chunk = TextChunk(
                id="query_chunk",
                page_id="query_page",
                content=query,
                start_pos=0,
                end_pos=len(query),
                chunk_index=0,
                tokens_count=len(query.split())  # Simple token count
            )

            # Generate embedding for the query
            query_embedding = self.embedding_generator.generate_embedding(query_chunk)

            # Search for similar embeddings in Qdrant
            results = self.vector_storage.search_similar(
                query_vector=query_embedding.vector,
                book_id=book_id,
                limit=limit
            )

            # Format results to include relevant information
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'id': result['id'],
                    'score': result['score'],
                    'content': result['payload'].get('content', ''),
                    'page_url': result['payload'].get('page_url', ''),
                    'page_title': result['payload'].get('page_title', ''),
                    'section_heading': result['payload'].get('section_heading', ''),
                    'book_id': result['payload'].get('book_id', ''),
                    'page_number': result['payload'].get('page_number', ''),
                    'additional_tags': result['payload'].get('additional_tags', [])
                })

            return formatted_results
        except Exception as e:
            print(f"Error retrieving chunks: {str(e)}")
            return []


def retrieve_book_content(query: str, book_id: str = "default_book", limit: int = 5) -> str:
    """
    Function to retrieve book content that can be used as a tool for the OpenAI agent.

    Args:
        query: The query to search for in the vector database
        book_id: ID of the book to search within
        limit: Maximum number of results to return

    Returns:
        JSON string of relevant chunks
    """
    retrieval_tool = RetrievalTool()
    results = retrieval_tool.retrieve_chunks(query, book_id, limit)
    return json.dumps(results, indent=2)