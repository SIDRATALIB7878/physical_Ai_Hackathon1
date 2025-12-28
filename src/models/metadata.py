"""Data model for metadata associated with embeddings."""
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Metadata:
    """
    Information that accompanies each embedding, including the original page URL, 
    section headings, and other contextual information.
    """
    id: str
    embedding_id: str
    page_url: str
    page_title: str
    section_heading: str
    book_id: str
    page_number: int
    additional_tags: List[str]
    
    def __post_init__(self):
        """Validate the Metadata after initialization."""
        if not self.id:
            raise ValueError("Metadata.id cannot be empty")
        if not self.embedding_id:
            raise ValueError("Metadata.embedding_id cannot be empty")
        if not self.page_url:
            raise ValueError("Metadata.page_url cannot be empty")
        if not self.book_id:
            raise ValueError("Metadata.book_id cannot be empty")