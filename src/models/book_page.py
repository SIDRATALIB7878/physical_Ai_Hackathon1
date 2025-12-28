"""Data model for a book page."""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime


@dataclass
class BookPage:
    """
    Represents an individual page of book content with properties like URL, 
    raw HTML content, extracted text, and structural elements.
    """
    id: str
    url: str
    title: str
    raw_html: str
    extracted_text: str
    headings: List[str]
    created_at: datetime
    updated_at: datetime
    
    def __post_init__(self):
        """Validate the BookPage after initialization."""
        if not self.id:
            raise ValueError("BookPage.id cannot be empty")
        if not self.url:
            raise ValueError("BookPage.url cannot be empty")
        if not self.extracted_text:
            raise ValueError("BookPage.extracted_text cannot be empty")