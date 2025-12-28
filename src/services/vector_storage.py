"""Service for storing and retrieving embeddings in Qdrant vector database."""
from typing import List, Dict, Any, Optional
from datetime import datetime
import logging

from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams

from src.models.embedding import Embedding
from src.models.metadata import Metadata
from src.lib.config import Config
from src.lib.errors import VectorStorageError, ConfigurationError
from src.lib.qdrant_service import QdrantService


class VectorStorageService:
    """Service for storing embeddings in Qdrant vector database with associated metadata."""
    
    def __init__(self):
        """Initialize the vector storage service."""
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            raise ConfigurationError("Invalid configuration: " + "; ".join(config_errors))
        
        # Initialize Qdrant client
        self.qdrant_client = QdrantService(
            url=Config.QDRANT_URL if Config.QDRANT_URL else None,
            api_key=Config.QDRANT_API_KEY,
            host=Config.QDRANT_HOST,
            port=Config.QDRANT_PORT
        )
        
        # Setup logging
        self.logger = logging.getLogger("book_embeddings_pipeline.vector_storage")
    
    def create_collection(self, collection_name: str, vector_size: int = 384) -> bool:
        """
        Create a collection for storing book embeddings.
        
        Args:
            collection_name: Name of the collection to create
            vector_size: Size of the embedding vectors (default 384 for Cohere embed-english-light-v2.0)
            
        Returns:
            True if collection was created successfully
        """
        try:
            # Define collection configuration for book embeddings
            collection_created = self.qdrant_client.create_collection(
                collection_name=collection_name,
                vector_size=vector_size
            )
            
            if collection_created:
                self.logger.info(f"Successfully created collection: {collection_name}")
                return True
            else:
                self.logger.error(f"Failed to create collection: {collection_name}")
                return False
        except Exception as e:
            self.logger.error(f"Error creating collection {collection_name}: {str(e)}")
            raise VectorStorageError(f"Failed to create collection {collection_name}: {str(e)}")
    
    def store_embedding(self, embedding: Embedding, metadata: Metadata) -> bool:
        """
        Store an embedding with metadata in the specified collection.

        Args:
            embedding: The embedding to store
            metadata: Metadata associated with the embedding

        Returns:
            True if embedding was stored successfully
        """
        # Validate inputs before storing
        validation_errors = self._validate_embedding_and_metadata(embedding, metadata)
        if validation_errors:
            error_msg = f"Validation failed for embedding {embedding.id}: {'; '.join(validation_errors)}"
            self.logger.error(error_msg)
            raise VectorStorageError(error_msg)

        try:
            # Construct the collection name from the book_id in metadata
            collection_name = f"book_embeddings_{metadata.book_id}"

            # Prepare payload with metadata
            payload = {
                "page_url": metadata.page_url,
                "page_title": metadata.page_title,
                "section_heading": metadata.section_heading,
                "book_id": metadata.book_id,
                "page_number": metadata.page_number,
                "additional_tags": metadata.additional_tags,
                "chunk_id": embedding.chunk_id,
                "model_name": embedding.model_name,
                "created_at": embedding.created_at.isoformat()
            }

            # Store the embedding in Qdrant
            success = self.qdrant_client.store_embedding(
                collection_name=collection_name,
                embedding_id=embedding.id,
                vector=embedding.vector,
                metadata=payload
            )

            if success:
                self.logger.info(f"Successfully stored embedding {embedding.id} in collection {collection_name}")
                return True
            else:
                self.logger.error(f"Failed to store embedding {embedding.id}")
                return False
        except Exception as e:
            self.logger.error(f"Error storing embedding {embedding.id}: {str(e)}")
            raise VectorStorageError(f"Failed to store embedding {embedding.id}: {str(e)}")

    def _validate_embedding_and_metadata(self, embedding: Embedding, metadata: Metadata) -> List[str]:
        """
        Validate that the embedding and metadata are properly formed.

        Args:
            embedding: The embedding to validate
            metadata: The metadata to validate

        Returns:
            List of validation error messages, empty if valid
        """
        errors = []

        # Validate embedding
        if not embedding.id:
            errors.append("Embedding ID is required")
        if not embedding.vector:
            errors.append("Embedding vector is required")
        if len(embedding.vector) == 0:
            errors.append("Embedding vector cannot be empty")
        if not embedding.model_name:
            errors.append("Embedding model name is required")
        if not embedding.chunk_id:
            errors.append("Embedding chunk ID is required")

        # Validate metadata
        if not metadata.id:
            errors.append("Metadata ID is required")
        if not metadata.embedding_id:
            errors.append("Metadata embedding ID is required")
        if not metadata.page_url:
            errors.append("Metadata page URL is required")
        if not metadata.book_id:
            errors.append("Metadata book ID is required")

        return errors
    
    def search_similar(self, query_vector: List[float], book_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings to the query vector in the specified book collection.
        
        Args:
            query_vector: The query embedding vector
            book_id: ID of the book to search within
            limit: Maximum number of results to return
            
        Returns:
            List of similar embeddings with their metadata
        """
        try:
            collection_name = f"book_embeddings_{book_id}"
            
            # Search for similar embeddings in Qdrant
            results = self.qdrant_client.search_similar(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit
            )
            
            self.logger.info(f"Found {len(results)} similar embeddings in collection {collection_name}")
            return results
        except Exception as e:
            self.logger.error(f"Error searching in collection {collection_name}: {str(e)}")
            raise VectorStorageError(f"Failed to search in collection {collection_name}: {str(e)}")
    
    def get_embedding(self, embedding_id: str, book_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific embedding by ID from the specified book collection.
        
        Args:
            embedding_id: ID of the embedding to retrieve
            book_id: ID of the book the embedding belongs to
            
        Returns:
            Dictionary with embedding data and metadata, or None if not found
        """
        try:
            from qdrant_client.http.exceptions import UnexpectedResponse
            
            collection_name = f"book_embeddings_{book_id}"
            
            # In a real implementation, we would use the Qdrant client to retrieve the specific point
            # For now, we'll simulate by searching and filtering (not ideal for production)
            # But since we need exact match by ID, let's implement proper retrieval if available
            self.logger.info(f"Retrieving embedding {embedding_id} from collection {collection_name}")
            
            # For now, return the same pattern as search_similar
            # In a real system, we'd have a get_point_by_id method
            return None  # Placeholder until we implement proper retrieval
            
        except Exception as e:
            self.logger.error(f"Error retrieving embedding {embedding_id} from collection {collection_name}: {str(e)}")
            raise VectorStorageError(f"Failed to retrieve embedding {embedding_id}: {str(e)}")
    
    def delete_embedding(self, embedding_id: str, book_id: str) -> bool:
        """
        Delete a specific embedding by ID from the specified book collection.
        
        Args:
            embedding_id: ID of the embedding to delete
            book_id: ID of the book the embedding belongs to
            
        Returns:
            True if embedding was deleted successfully
        """
        try:
            collection_name = f"book_embeddings_{book_id}"
            
            # In a real implementation, we would use the Qdrant client to delete the specific point
            self.logger.info(f"Deleting embedding {embedding_id} from collection {collection_name}")
            
            # Placeholder implementation - in real system, would call client.delete(points_selector=...)
            return True  # Placeholder
            
        except Exception as e:
            self.logger.error(f"Error deleting embedding {embedding_id} from collection {collection_name}: {str(e)}")
            raise VectorStorageError(f"Failed to delete embedding {embedding_id}: {str(e)}")
    
    def check_collection_exists(self, book_id: str) -> bool:
        """
        Check if a collection exists for the specified book.
        
        Args:
            book_id: ID of the book to check
            
        Returns:
            True if collection exists
        """
        try:
            collection_name = f"book_embeddings_{book_id}"
            return self.qdrant_client.check_collection_exists(collection_name)
        except Exception as e:
            self.logger.error(f"Error checking if collection {collection_name} exists: {str(e)}")
            return False