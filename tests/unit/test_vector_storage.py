"""Unit tests for the Vector Storage Service."""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.vector_storage import VectorStorageService
from src.models.embedding import Embedding
from src.models.metadata import Metadata
from src.models.text_chunk import TextChunk
from src.lib.errors import VectorStorageError


class TestVectorStorageService:
    """Test class for VectorStorageService."""
    
    def test_store_embedding_success(self):
        """Test successful storage of an embedding."""
        service = VectorStorageService()
        
        # Create a test embedding and metadata
        embedding = Embedding(
            id="test-emb-1",
            chunk_id="test-chunk-1",
            vector=[0.1, 0.2, 0.3],
            model_name="test-model",
            created_at=MagicMock()
        )
        
        metadata = Metadata(
            id="test-meta-1",
            embedding_id="test-emb-1",
            page_url="https://example.com/page1",
            page_title="Test Page",
            section_heading="Introduction",
            book_id="test-book",
            page_number=1,
            additional_tags=["tag1", "tag2"]
        )
        
        # Mock the Qdrant client operations
        with patch.object(service.qdrant_client, 'upsert', return_value=True):
            result = service.store_embedding(embedding, metadata)
        
        assert result is True
    
    def test_store_embedding_qdrant_error(self):
        """Test handling of Qdrant errors during storage."""
        service = VectorStorageService()
        
        # Create a test embedding and metadata
        embedding = Embedding(
            id="test-emb-2",
            chunk_id="test-chunk-2",
            vector=[0.4, 0.5, 0.6],
            model_name="test-model",
            created_at=MagicMock()
        )
        
        metadata = Metadata(
            id="test-meta-2",
            embedding_id="test-emb-2",
            page_url="https://example.com/page2",
            page_title="Test Page 2",
            section_heading="Conclusion",
            book_id="test-book",
            page_number=2,
            additional_tags=[]
        )
        
        # Mock the Qdrant client to raise an exception
        with patch.object(service.qdrant_client, 'upsert', side_effect=Exception("Qdrant error")):
            with pytest.raises(VectorStorageError):
                service.store_embedding(embedding, metadata)
    
    def test_retrieve_embedding_success(self):
        """Test successful retrieval of embeddings."""
        service = VectorStorageService()
        
        # Mock the Qdrant search response
        mock_result = Mock()
        mock_result.id = "test-emb-3"
        mock_result.score = 0.95
        mock_result.payload = {
            "page_url": "https://example.com/page3",
            "page_title": "Retrieved Page",
            "section_heading": "Section",
            "book_id": "test-book",
            "page_number": 3
        }
        
        with patch.object(service.qdrant_client, 'search', return_value=[mock_result]):
            results = service.search_similar([0.7, 0.8, 0.9], "test-book", limit=5)
        
        assert len(results) == 1
        assert results[0]['id'] == "test-emb-3"
        assert results[0]['score'] == 0.95
        assert results[0]['payload']['page_url'] == "https://example.com/page3"
    
    def test_retrieve_embedding_qdrant_error(self):
        """Test handling of Qdrant errors during retrieval."""
        service = VectorStorageService()
        
        # Mock the Qdrant client to raise an exception
        with patch.object(service.qdrant_client, 'search', side_effect=Exception("Qdrant search error")):
            with pytest.raises(VectorStorageError):
                service.search_similar([0.1, 0.2, 0.3], "test-book", limit=5)
    
    def test_create_collection_success(self):
        """Test successful creation of a collection."""
        service = VectorStorageService()
        
        # Mock the Qdrant client collection creation
        with patch.object(service.qdrant_client, 'recreate_collection', return_value=True):
            result = service.create_collection("test-collection", 384)
        
        assert result is True
    
    def test_create_collection_qdrant_error(self):
        """Test handling of Qdrant errors during collection creation."""
        service = VectorStorageService()
        
        # Mock the Qdrant client to raise an exception
        with patch.object(service.qdrant_client, 'recreate_collection', side_effect=Exception("Qdrant create error")):
            with pytest.raises(VectorStorageError):
                service.create_collection("test-collection", 384)