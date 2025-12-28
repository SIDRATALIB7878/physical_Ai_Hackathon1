"""Data model for a text chunk."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class TextChunk:
    """
    A segment of text from a book page that will be converted into an embedding.
    """
    id: str
    page_id: str
    content: str
    start_pos: int
    end_pos: int
    chunk_index: int
    tokens_count: int
    
    def __post_init__(self):
        """Validate the TextChunk after initialization."""
        if not self.id:
            raise ValueError("TextChunk.id cannot be empty")
        if not self.page_id:
            raise ValueError("TextChunk.page_id cannot be empty")
        if not self.content:
            raise ValueError("TextChunk.content cannot be empty")
        if self.start_pos < 0:
            raise ValueError("TextChunk.start_pos must be non-negative")
        if self.end_pos < self.start_pos:
            raise ValueError("TextChunk.end_pos must be greater than or equal to start_pos")
        if self.chunk_index < 0:
            raise ValueError("TextChunk.chunk_index must be non-negative")
        if self.tokens_count <= 0:
            raise ValueError("TextChunk.tokens_count must be positive")