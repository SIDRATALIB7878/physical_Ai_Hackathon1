"""Integration tests for vector storage process."""
import pytest
from src.services.vector_storage import VectorStorageService
from src.services.embedding_generator import EmbeddingGeneratorService
from src.models.embedding import Embedding
from src.models.metadata import Metadata
from src.models.text_chunk import TextChunk
from datetime import datetime


class TestVectorStorageIntegration:
    """Integration tests for vector storage process."""
    
    def test_full_storage_process(self):
        """Test the complete process from embedding to storage in Qdrant."""
        # Since we can't connect to a real Qdrant instance in tests, we'll use mocks
        from unittest.mock import patch, Mock
        
        service = VectorStorageService()
        
        # Create a test embedding and metadata
        embedding = Embedding(
            id="integration-test-emb",
            chunk_id="integration-test-chunk",
            vector=[0.1, 0.2, 0.3] * 128,  # Simulate 384-dim embedding
            model_name="embed-english-light-v2.0",
            created_at=datetime.now()
        )
        
        metadata = Metadata(
            id="integration-test-meta",
            embedding_id="integration-test-emb",
            page_url="https://example.com/integration-test",
            page_title="Integration Test Page",
            section_heading="Integration Test Section",
            book_id="integration-test-book",
            page_number=1,
            additional_tags=["integration", "test"]
        )
        
        # Mock the successful upsert operation
        with patch.object(service.qdrant_client, 'upsert', return_value=True):
            store_result = service.store_embedding(embedding, metadata)
        
        assert store_result is True
        
        # Test the search capability
        query_vector = [0.15, 0.25, 0.35] * 128  # Similar to our stored embedding
        
        mock_result = Mock()
        mock_result.id = "integration-test-emb"
        mock_result.score = 0.92
        mock_result.payload = {
            "page_url": "https://example.com/integration-test",
            "page_title": "Integration Test Page",
            "section_heading": "Integration Test Section",
            "book_id": "integration-test-book",
            "page_number": 1,
            "additional_tags": ["integration", "test"]
        }
        
        with patch.object(service.qdrant_client, 'search', return_value=[mock_result]):
            search_results = service.search_similar(query_vector, "integration-test-book", limit=5)
        
        assert len(search_results) == 1
        assert search_results[0]['id'] == "integration-test-emb"
        assert search_results[0]['score'] == 0.92
        assert search_results[0]['payload']['page_title'] == "Integration Test Page"
    
    def test_multiple_embeddings_storage(self):
        """Test storage and retrieval of multiple embeddings."""
        from unittest.mock import patch, Mock
        
        service = VectorStorageService()
        
        # Create multiple test embeddings and metadata
        embeddings = []
        metadatas = []
        
        for i in range(3):
            embedding = Embedding(
                id=f"multi-test-emb-{i}",
                chunk_id=f"multi-test-chunk-{i}",
                vector=[float(i+1) * 0.1, float(i+2) * 0.1, float(i+3) * 0.1] * 128,  # 384-dim
                model_name="embed-english-light-v2.0",
                created_at=datetime.now()
            )
            
            metadata = Metadata(
                id=f"multi-test-meta-{i}",
                embedding_id=f"multi-test-emb-{i}",
                page_url=f"https://example.com/page-{i}",
                page_title=f"Test Page {i}",
                section_heading=f"Section {i}",
                book_id="multi-test-book",
                page_number=i+1,
                additional_tags=[f"tag-{i}"]
            )
            
            embeddings.append(embedding)
            metadatas.append(metadata)
        
        # Mock storing all embeddings
        with patch.object(service.qdrant_client, 'upsert', return_value=True):
            for embedding, metadata in zip(embeddings, metadatas):
                result = service.store_embedding(embedding, metadata)
                assert result is True
        
        # Test retrieval of similar embeddings
        query_vector = [0.11, 0.21, 0.31] * 128  # Similar to the first embedding
        
        mock_results = []
        for i in range(2):  # Expect 2 results
            mock_result = Mock()
            mock_result.id = f"multi-test-emb-{i}"
            mock_result.score = 0.9 - (i * 0.1)  # Decreasing scores
            mock_result.payload = {
                "page_url": f"https://example.com/page-{i}",
                "page_title": f"Test Page {i}",
                "section_heading": f"Section {i}",
                "book_id": "multi-test-book",
                "page_number": i+1,
                "additional_tags": [f"tag-{i}"]
            }
            mock_results.append(mock_result)
        
        with patch.object(service.qdrant_client, 'search', return_value=mock_results):
            search_results = service.search_similar(query_vector, "multi-test-book", limit=5)
        
        assert len(search_results) == 2
        assert search_results[0]['id'] == "multi-test-emb-0"
        assert search_results[1]['id'] == "multi-test-emb-1"
        assert search_results[0]['score'] > search_results[1]['score']  # First should be more similar