"""Contract tests for the vector storage service."""
import pytest
from src.services.vector_storage import VectorStorageService
from src.models.embedding import Embedding
from src.models.metadata import Metadata
from src.lib.errors import VectorStorageError
from datetime import datetime


class TestVectorStorageContract:
    """Contract tests to ensure the vector storage service meets the API contract."""
    
    def test_store_embedding_returns_boolean(self):
        """Test that store_embedding returns a boolean as specified in the contract."""
        service = VectorStorageService()
        
        # Create a test embedding and metadata
        embedding = Embedding(
            id="contract-test-emb",
            chunk_id="contract-test-chunk",
            vector=[0.1, 0.2, 0.3],
            model_name="test-model",
            created_at=datetime.now()
        )
        
        metadata = Metadata(
            id="contract-test-meta",
            embedding_id="contract-test-emb",
            page_url="https://example.com/test",
            page_title="Contract Test Page",
            section_heading="Test Section",
            book_id="contract-test-book",
            page_number=1,
            additional_tags=[]
        )
        
        # We'll mock the Qdrant client for contract testing
        from unittest.mock import patch
        
        with patch.object(service.qdrant_client, 'upsert', return_value=True):
            result = service.store_embedding(embedding, metadata)
        
        # Contract specifies it should return boolean
        assert isinstance(result, bool)
        assert result is True
    
    def test_store_embedding_signature(self):
        """Test that store_embedding has the correct signature as per contract."""
        service = VectorStorageService()
        
        # Check the method exists
        assert hasattr(service, 'store_embedding')
        
        # Check the method is callable
        assert callable(getattr(service, 'store_embedding'))
    
    def test_store_embedding_error_handling(self):
        """Test that store_embedding raises VectorStorageError on failure."""
        service = VectorStorageService()
        
        # Create a test embedding and metadata
        embedding = Embedding(
            id="error-test-emb",
            chunk_id="error-test-chunk",
            vector=[0.1, 0.2, 0.3],
            model_name="test-model",
            created_at=datetime.now()
        )
        
        metadata = Metadata(
            id="error-test-meta",
            embedding_id="error-test-emb",
            page_url="https://example.com/test",
            page_title="Error Test Page",
            section_heading="Test Section",
            book_id="error-test-book",
            page_number=1,
            additional_tags=[]
        )
        
        # Mock Qdrant client to raise an exception
        from unittest.mock import patch
        
        with patch.object(service.qdrant_client, 'upsert', side_effect=Exception("Storage error")):
            with pytest.raises(VectorStorageError):
                service.store_embedding(embedding, metadata)
    
    def test_search_similar_signature(self):
        """Test that search_similar has the correct signature as per contract."""
        service = VectorStorageService()
        
        # Check the method exists
        assert hasattr(service, 'search_similar')
        
        # Check the method is callable
        assert callable(getattr(service, 'search_similar'))
    
    def test_embedding_storage_contract_attributes(self):
        """Test that required models have the required attributes as per contract."""
        # Import the models
        from src.models.embedding import Embedding
        from src.models.metadata import Metadata
        
        # Check that Embedding has all required attributes
        embedding_required_attrs = [
            'id', 'chunk_id', 'vector', 'model_name', 'created_at'
        ]
        
        for attr in embedding_required_attrs:
            assert hasattr(Embedding, attr) or attr in Embedding.__annotations__, \
                f"Embedding is missing required attribute: {attr}"
        
        # Check that Metadata has all required attributes
        metadata_required_attrs = [
            'id', 'embedding_id', 'page_url', 'page_title', 'section_heading',
            'book_id', 'page_number', 'additional_tags'
        ]
        
        for attr in metadata_required_attrs:
            assert hasattr(Metadata, attr) or attr in Metadata.__annotations__, \
                f"Metadata is missing required attribute: {attr}"
    
    def test_create_collection_signature(self):
        """Test that create_collection has the correct signature as per contract."""
        service = VectorStorageService()
        
        # Check the method exists
        assert hasattr(service, 'create_collection')
        
        # Check the method is callable
        assert callable(getattr(service, 'create_collection'))
    
    def test_create_collection_contract(self):
        """Test that create_collection works according to contract."""
        service = VectorStorageService()
        
        # Mock the successful collection creation
        from unittest.mock import patch
        
        with patch.object(service.qdrant_client, 'recreate_collection', return_value=True):
            result = service.create_collection("test-book-embeddings", 384)
        
        # Should return boolean indicating success
        assert isinstance(result, bool)
        assert result is True