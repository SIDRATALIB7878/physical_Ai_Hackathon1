"""Mock retrieval tool for testing the RAG agent without actual Qdrant connection."""
import json
from typing import List, Dict, Any


def retrieve_book_content(query: str, book_id: str = "default_book", limit: int = 5) -> str:
    """
    Mock function to simulate retrieving book content.
    This allows testing the agent without an actual Qdrant connection.
    
    Args:
        query: The query to search for in the vector database
        book_id: ID of the book to search within
        limit: Maximum number of results to return
        
    Returns:
        JSON string of relevant chunks
    """
    # Simulate search results based on the query
    if "artificial intelligence" in query.lower():
        mock_results = [
            {
                "id": "mock_chunk_1",
                "score": 0.95,
                "content": "Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
                "page_url": "http://example.com/book/page1",
                "page_title": "Introduction to AI",
                "section_heading": "What is AI?",
                "book_id": book_id,
                "page_number": 1,
                "additional_tags": ["AI", "machine learning", "cognitive computing"]
            },
            {
                "id": "mock_chunk_2", 
                "score": 0.89,
                "content": "The field of AI includes various subfields such as machine learning, natural language processing, robotics, computer vision, and expert systems. These technologies work together to enable machines to perform tasks that typically require human intelligence.",
                "page_url": "http://example.com/book/page5",
                "page_title": "AI Subfields",
                "section_heading": "Components of AI",
                "book_id": book_id,
                "page_number": 5,
                "additional_tags": ["AI", "subfields", "machine learning", "NLP"]
            }
        ]
    elif "machine learning" in query.lower():
        mock_results = [
            {
                "id": "mock_chunk_3",
                "score": 0.92,
                "content": "Machine learning is a subset of artificial intelligence that focuses on building systems that can learn from data, identify patterns, and make decisions with minimal human intervention. It uses algorithms to parse data, learn from it, and make informed decisions based on what it has learned.",
                "page_url": "http://example.com/book/page10",
                "page_title": "Understanding Machine Learning",
                "section_heading": "Machine Learning Basics",
                "book_id": book_id,
                "page_number": 10,
                "additional_tags": ["ML", "algorithms", "data", "learning"]
            }
        ]
    else:
        # Default mock result for other queries
        mock_results = [
            {
                "id": "mock_chunk_default",
                "score": 0.85,
                "content": f"This is a simulated result for the query: '{query}'. In a real implementation, this would come from searching the Qdrant vector database with actual book content embeddings.",
                "page_url": "http://example.com/book/default",
                "page_title": "Simulated Result",
                "section_heading": "Mock Data",
                "book_id": book_id,
                "page_number": 1,
                "additional_tags": ["mock", "test", "simulation"]
            }
        ]
    
    # Limit the results to the specified number
    mock_results = mock_results[:limit]
    
    return json.dumps(mock_results, indent=2)