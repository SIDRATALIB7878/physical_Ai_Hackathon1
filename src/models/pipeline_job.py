"""Data model for a pipeline job."""
from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class PipelineJob:
    """
    Represents a single execution of the embedding pipeline.
    """
    id: str
    book_url: str
    status: str  # Enum: QUEUED, PROCESSING, COMPLETED, FAILED
    pages_count: int
    pages_processed: int
    pages_failed: int
    created_at: datetime
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    error_message: Optional[str] = None
    
    def __post_init__(self):
        """Validate the PipelineJob after initialization."""
        if not self.id:
            raise ValueError("PipelineJob.id cannot be empty")
        if not self.book_url:
            raise ValueError("PipelineJob.book_url cannot be empty")
        if self.status not in ["QUEUED", "PROCESSING", "COMPLETED", "FAILED"]:
            raise ValueError("PipelineJob.status must be one of: QUEUED, PROCESSING, COMPLETED, FAILED")
        if self.pages_count < 0:
            raise ValueError("PipelineJob.pages_count cannot be negative")
        if self.pages_processed < 0:
            raise ValueError("PipelineJob.pages_processed cannot be negative")
        if self.pages_failed < 0:
            raise ValueError("PipelineJob.pages_failed cannot be negative")
        if self.pages_processed + self.pages_failed > self.pages_count:
            raise ValueError("PipelineJob.pages_processed + pages_failed cannot exceed pages_count")