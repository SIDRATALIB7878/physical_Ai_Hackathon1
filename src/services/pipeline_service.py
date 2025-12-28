"""Service for orchestrating the complete book embeddings pipeline."""
from typing import List, Dict, Any, Optional
import logging
from datetime import datetime
import uuid
import time

from src.models.pipeline_job import PipelineJob
from src.models.book_page import BookPage
from src.models.text_chunk import TextChunk
from src.models.embedding import Embedding
from src.models.metadata import Metadata
from src.services.content_extractor import ContentExtractionService
from src.services.embedding_generator import EmbeddingGeneratorService
from src.services.vector_storage import VectorStorageService
from src.lib.config import Config
from src.lib.errors import PipelineError
from src.lib.logging import logger


class PipelineOrchestrationService:
    """Service for orchestrating the complete book embeddings pipeline."""
    
    def __init__(self):
        """Initialize the pipeline orchestration service."""
        self.content_extractor = ContentExtractionService()
        self.embedding_generator = EmbeddingGeneratorService()
        self.vector_storage = VectorStorageService()
        
        self.logger = logger
    
    def run_pipeline(self, book_urls: List[str], book_id: str) -> PipelineJob:
        """
        Runs the complete pipeline for a book with the given URLs.

        Args:
            book_urls: List of URLs containing the book content
            book_id: Unique identifier for the book

        Returns:
            PipelineJob: An object containing job status and results
        """
        # Create pipeline job record
        job_id = str(uuid.uuid4())
        pipeline_job = PipelineJob(
            id=job_id,
            book_url=book_urls[0] if book_urls else "",
            status="QUEUED",
            pages_count=len(book_urls),
            pages_processed=0,
            pages_failed=0,
            created_at=datetime.now()
        )

        # Performance tracking
        start_time = time.time()
        total_embeddings_generated = 0
        total_tokens_processed = 0

        try:
            self.logger.info(f"Starting pipeline for book {book_id} with {len(book_urls)} pages")

            # Update job status to processing
            pipeline_job.status = "PROCESSING"
            pipeline_job.started_at = datetime.now()

            # Process each URL
            for i, url in enumerate(book_urls):
                page_start_time = time.time()
                try:
                    self.logger.info(f"Processing page {i+1}/{len(book_urls)}: {url}")

                    # Step 1: Extract content from the page
                    book_page = self.content_extractor.extract_text_from_url(url)

                    # Step 2: Preprocess and chunk the text
                    text_chunks = self.embedding_generator.chunk_text(
                        book_page.extracted_text,
                        book_page.id
                    )

                    # Step 3: Generate embeddings for each chunk
                    for chunk in text_chunks:
                        embedding = self.embedding_generator.generate_embedding(chunk)
                        total_embeddings_generated += 1
                        total_tokens_processed += chunk.tokens_count

                        # Step 4: Create metadata for the embedding
                        metadata = Metadata(
                            id=f"meta_{embedding.id}",
                            embedding_id=embedding.id,
                            page_url=book_page.url,
                            page_title=book_page.title,
                            section_heading=self._get_section_heading_for_chunk(chunk, book_page.headings),
                            book_id=book_id,
                            page_number=i+1,  # Using index as page number
                            additional_tags=[]
                        )

                        # Step 5: Store the embedding with metadata
                        self.vector_storage.store_embedding(embedding, metadata)

                    # Calculate page processing time
                    page_processing_time = time.time() - page_start_time
                    self.logger.debug(f"Page {url} processed in {page_processing_time:.2f}s")

                    # Increment successful pages counter
                    pipeline_job.pages_processed += 1

                except Exception as e:
                    self.logger.error(f"Error processing page {url}: {str(e)}")
                    pipeline_job.pages_failed += 1

            # Set final job status
            if pipeline_job.pages_failed == 0:
                pipeline_job.status = "COMPLETED"
            elif pipeline_job.pages_failed == len(book_urls):
                pipeline_job.status = "FAILED"
            else:
                pipeline_job.status = "COMPLETED_WITH_ERRORS"

            pipeline_job.completed_at = datetime.now()

            # Calculate and log performance metrics
            total_time = time.time() - start_time
            pages_per_hour = (pipeline_job.pages_processed / total_time) * 3600 if total_time > 0 else 0
            embeddings_per_second = total_embeddings_generated / total_time if total_time > 0 else 0

            self.logger.info(
                f"Pipeline completed for book {book_id}. "
                f"Processed: {pipeline_job.pages_processed}, "
                f"Failed: {pipeline_job.pages_failed}, "
                f"Total time: {total_time:.2f}s, "
                f"Pages/hour: {pages_per_hour:.2f}, "
                f"Embeddings/sec: {embeddings_per_second:.2f}"
            )
            self.logger.info(
                f"Performance summary: {total_embeddings_generated} embeddings generated, "
                f"~{total_tokens_processed} tokens processed"
            )

            return pipeline_job

        except Exception as e:
            self.logger.error(f"Pipeline failed for book {book_id}: {str(e)}")
            pipeline_job.status = "FAILED"
            pipeline_job.error_message = str(e)
            pipeline_job.completed_at = datetime.now()
            return pipeline_job
    
    def _get_section_heading_for_chunk(self, chunk: TextChunk, page_headings: List[str]) -> str:
        """
        Determine the most relevant heading for a text chunk based on its position in the page.
        
        Args:
            chunk: The text chunk
            page_headings: List of headings from the page
            
        Returns:
            The most relevant heading for the chunk
        """
        if not page_headings:
            return "Uncategorized"
        
        # For simplicity, we'll return the first heading
        # In a more sophisticated implementation, we might analyze 
        # the chunk's position relative to heading positions
        return page_headings[0]
    
    def run_pipeline_with_duplicate_handling(self, book_urls: List[str], book_id: str,
                                           handle_duplicates: bool = True) -> PipelineJob:
        """
        Runs the complete pipeline with duplicate content handling as specified in research.md.

        Args:
            book_urls: List of URLs containing the book content
            book_id: Unique identifier for the book
            handle_duplicates: Whether to check for and handle duplicate content

        Returns:
            PipelineJob: An object containing job status and results
        """
        if handle_duplicates:
            self.logger.info("Running pipeline with duplicate content detection enabled")

        # Create pipeline job record
        job_id = str(uuid.uuid4())
        pipeline_job = PipelineJob(
            id=job_id,
            book_url=book_urls[0] if book_urls else "",
            status="QUEUED",
            pages_count=len(book_urls),
            pages_processed=0,
            pages_failed=0,
            created_at=datetime.now()
        )

        # Track content hashes to identify duplicates
        content_hashes = {}
        duplicate_count = 0

        # Performance tracking
        start_time = time.time()
        total_embeddings_generated = 0
        total_tokens_processed = 0

        try:
            self.logger.info(f"Starting pipeline for book {book_id} with {len(book_urls)} pages")

            # Update job status to processing
            pipeline_job.status = "PROCESSING"
            pipeline_job.started_at = datetime.now()

            # Process each URL
            for i, url in enumerate(book_urls):
                page_start_time = time.time()
                try:
                    self.logger.info(f"Processing page {i+1}/{len(book_urls)}: {url}")

                    # Step 1: Extract content from the page
                    book_page = self.content_extractor.extract_text_from_url(url)

                    # Check for duplicate content if enabled
                    if handle_duplicates:
                        import hashlib
                        content_hash = hashlib.md5(book_page.extracted_text.encode()).hexdigest()

                        if content_hash in content_hashes:
                            self.logger.info(f"Duplicate content detected for page {url}, "
                                           f"original at {content_hashes[content_hash]}")
                            # According to research.md, we flag duplicates but store separately
                            duplicate_count += 1

                        content_hashes[content_hash] = url

                    # Step 2: Preprocess and chunk the text
                    text_chunks = self.embedding_generator.chunk_text(
                        book_page.extracted_text,
                        book_page.id
                    )

                    # Step 3: Generate embeddings for each chunk
                    for chunk in text_chunks:
                        embedding = self.embedding_generator.generate_embedding(chunk)
                        total_embeddings_generated += 1
                        total_tokens_processed += chunk.tokens_count

                        # Step 4: Create metadata for the embedding
                        # Add duplicate flag to metadata if this content was a duplicate
                        additional_tags = ["duplicate"] if handle_duplicates and content_hash in content_hashes and content_hashes[content_hash] != url else []

                        metadata = Metadata(
                            id=f"meta_{embedding.id}",
                            embedding_id=embedding.id,
                            page_url=book_page.url,
                            page_title=book_page.title,
                            section_heading=self._get_section_heading_for_chunk(chunk, book_page.headings),
                            book_id=book_id,
                            page_number=i+1,  # Using index as page number
                            additional_tags=additional_tags
                        )

                        # Step 5: Store the embedding with metadata
                        self.vector_storage.store_embedding(embedding, metadata)

                    # Calculate page processing time
                    page_processing_time = time.time() - page_start_time
                    self.logger.debug(f"Page {url} processed in {page_processing_time:.2f}s")

                    # Increment successful pages counter
                    pipeline_job.pages_processed += 1

                except Exception as e:
                    self.logger.error(f"Error processing page {url}: {str(e)}")
                    pipeline_job.pages_failed += 1

            # Set final job status
            if pipeline_job.pages_failed == 0:
                pipeline_job.status = "COMPLETED"
            elif pipeline_job.pages_failed == len(book_urls):
                pipeline_job.status = "FAILED"
            else:
                pipeline_job.status = "COMPLETED_WITH_ERRORS"

            pipeline_job.completed_at = datetime.now()

            # Calculate and log performance metrics
            total_time = time.time() - start_time
            pages_per_hour = (pipeline_job.pages_processed / total_time) * 3600 if total_time > 0 else 0
            embeddings_per_second = total_embeddings_generated / total_time if total_time > 0 else 0

            self.logger.info(
                f"Pipeline completed for book {book_id}. "
                f"Processed: {pipeline_job.pages_processed}, "
                f"Failed: {pipeline_job.pages_failed}, "
                f"Duplicates found: {duplicate_count}, "
                f"Total time: {total_time:.2f}s, "
                f"Pages/hour: {pages_per_hour:.2f}, "
                f"Embeddings/sec: {embeddings_per_second:.2f}"
            )
            self.logger.info(
                f"Performance summary: {total_embeddings_generated} embeddings generated, "
                f"~{total_tokens_processed} tokens processed"
            )

            return pipeline_job

        except Exception as e:
            self.logger.error(f"Pipeline failed for book {book_id}: {str(e)}")
            pipeline_job.status = "FAILED"
            pipeline_job.error_message = str(e)
            pipeline_job.completed_at = datetime.now()
            return pipeline_job