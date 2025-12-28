"""Service for generating embeddings using Cohere API."""
import cohere
from typing import List
from datetime import datetime
import logging
import time
from collections import deque

from src.models.embedding import Embedding
from src.models.text_chunk import TextChunk
from src.lib.config import Config
from src.lib.errors import EmbeddingGenerationError, ConfigurationError, ProcessingRateLimitError
from src.services.text_preprocessor import TextPreprocessorService


class EmbeddingGeneratorService:
    """Service for generating vector embeddings for text chunks using Cohere."""

    def __init__(self, model_name: str = None, max_retries: int = 3):
        """
        Initialize the embedding generation service.

        Args:
            model_name: The Cohere model to use for embeddings
            max_retries: Maximum number of retry attempts for API calls
        """
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            raise ConfigurationError("Invalid configuration: " + "; ".join(config_errors))

        self.model_name = model_name or Config.COHERE_MODEL
        self.max_retries = max_retries
        self.co = cohere.Client(Config.COHERE_API_KEY)

        # Initialize text preprocessor with the same chunk size as configuration
        self.preprocessor = TextPreprocessorService(max_chunk_size=Config.CHUNK_SIZE)

        # Rate limiting: Track API calls to respect limits
        # Assuming a rate limit of 1000 requests per minute for Cohere API
        self.max_requests_per_minute = 900  # Stay under the limit
        self.requests_in_window = deque()
        self.window_duration = 60  # 60 seconds

        # Setup logging
        self.logger = logging.getLogger("book_embeddings_pipeline.embedding_generator")

    def _check_rate_limit(self):
        """
        Check if we're within the rate limit for API calls.

        Raises:
            ProcessingRateLimitError: If rate limit would be exceeded
        """
        current_time = time.time()

        # Remove requests that are outside the current window
        while (self.requests_in_window and
               current_time - self.requests_in_window[0] > self.window_duration):
            self.requests_in_window.popleft()

        # Check if we're at the limit
        if len(self.requests_in_window) >= self.max_requests_per_minute:
            raise ProcessingRateLimitError(
                f"Rate limit exceeded: {self.max_requests_per_minute} requests per {self.window_duration} seconds"
            )

        # Add current request to the window
        self.requests_in_window.append(current_time)

    def generate_embedding(self, text_chunk: TextChunk) -> Embedding:
        """
        Generates a vector embedding for a text chunk using the specified model.

        Args:
            text_chunk: The text chunk to embed

        Returns:
            Embedding: An object containing the generated vector
        """
        if not text_chunk.content.strip():
            raise EmbeddingGenerationError("Cannot generate embedding for empty text content")

        # Check rate limits before making the API call
        self._check_rate_limit()

        # Attempt to generate embedding with retry logic
        last_exception = None

        for attempt in range(self.max_retries):
            try:
                # Generate the embedding
                response = self.co.embed(
                    texts=[text_chunk.content],
                    model=self.model_name,
                    input_type="search_document"  # Using search_document as input type for book content
                )

                # Extract the embedding from the response
                if response.embeddings and len(response.embeddings) > 0:
                    embedding_vector = response.embeddings[0]  # Get first embedding

                    embedding = Embedding(
                        id=f"emb_{text_chunk.id}_{int(datetime.now().timestamp())}",
                        chunk_id=text_chunk.id,
                        vector=embedding_vector,
                        model_name=self.model_name,
                        created_at=datetime.now()
                    )

                    self.logger.info(f"Successfully generated embedding for chunk {text_chunk.id}")
                    return embedding
                else:
                    raise EmbeddingGenerationError(f"No embeddings returned for chunk {text_chunk.id}")

            except Exception as e:
                # If it's a rate limit error, wait longer before retrying
                if "rate limit" in str(e).lower():
                    self.logger.warning(f"Rate limit hit on attempt {attempt + 1}, waiting longer...")
                    time.sleep(30 * (attempt + 1))  # Longer wait for rate limit
                else:
                    self.logger.warning(f"Attempt {attempt + 1} failed for chunk {text_chunk.id}: {str(e)}")

                last_exception = e
                if attempt < self.max_retries - 1:
                    # Exponential backoff: wait 2^attempt seconds
                    time.sleep(2 ** attempt)

        # If all retries failed, raise an error
        raise EmbeddingGenerationError(
            f"Failed to generate embedding for chunk {text_chunk.id} after {self.max_retries} attempts: {str(last_exception)}"
        )
    
    def generate_embeddings_batch(self, text_chunks: List[TextChunk]) -> List[Embedding]:
        """
        Generates embeddings for a batch of text chunks.
        
        Args:
            text_chunks: List of text chunks to embed
            
        Returns:
            List of Embedding objects
        """
        embeddings = []
        
        for chunk in text_chunks:
            try:
                embedding = self.generate_embedding(chunk)
                embeddings.append(embedding)
            except EmbeddingGenerationError as e:
                self.logger.error(f"Failed to generate embedding for chunk {chunk.id}: {str(e)}")
                # Continue with other chunks even if one fails
                continue
        
        return embeddings
    
    def chunk_text(self, content: str, page_id: str) -> List[TextChunk]:
        """
        Wrapper method to chunk text using the preprocessor.
        
        Args:
            content: The text content to chunk
            page_id: The ID of the page this content comes from
            
        Returns:
            List of TextChunk objects
        """
        return self.preprocessor.chunk_text(content, page_id)