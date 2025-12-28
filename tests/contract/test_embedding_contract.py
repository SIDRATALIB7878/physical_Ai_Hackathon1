"""Contract tests for the embedding generation service."""
import pytest
from src.services.embedding_generator import EmbeddingGeneratorService
from src.models.embedding import Embedding
from src.models.text_chunk import TextChunk
from src.lib.errors import EmbeddingGenerationError


class TestEmbeddingGenerationContract:
    """Contract tests to ensure the embedding generation service meets the API contract."""
    
    def test_generate_embedding_returns_embedding(self):
        """Test that generate_embedding returns an Embedding object as specified in the contract."""
        service = EmbeddingGeneratorService()
        
        # Create a test text chunk
        text_chunk = TextChunk(
            id="contract-test-chunk",
            page_id="contract-test-page",
            content="This is a test sentence for contract verification.",
            start_pos=0,
            end_pos=50,
            chunk_index=0,
            tokens_count=10
        )
        
        # We'll mock the Cohere API call for contract testing
        from unittest.mock import patch, Mock
        
        mock_embeddings = Mock()
        # Mock embedding vector for embed-english-light-v2.0 (384 dimensions)
        mock_embeddings.embeddings = [list(range(384))]
        
        with patch.object(service.co, 'embed', return_value=mock_embeddings):
            result = service.generate_embedding(text_chunk)
        
        # Contract specifies it should return Embedding
        assert isinstance(result, Embedding)
        
        # Check that required fields are present
        assert hasattr(result, 'id')
        assert hasattr(result, 'chunk_id')
        assert hasattr(result, 'vector')
        assert hasattr(result, 'model_name')
        assert hasattr(result, 'created_at')
    
    def test_generate_embedding_signature(self):
        """Test that generate_embedding has the correct signature as per contract."""
        service = EmbeddingGeneratorService()
        
        # Check the method exists
        assert hasattr(service, 'generate_embedding')
        
        # Check the method is callable
        assert callable(getattr(service, 'generate_embedding'))
    
    def test_generate_embedding_error_handling(self):
        """Test that generate_embedding raises EmbeddingGenerationError on failure."""
        service = EmbeddingGeneratorService()
        
        # Create a test text chunk
        text_chunk = TextChunk(
            id="error-test-chunk",
            page_id="error-test-page",
            content="",
            start_pos=0,
            end_pos=0,
            chunk_index=0,
            tokens_count=0
        )
        
        with pytest.raises(EmbeddingGenerationError):
            service.generate_embedding(text_chunk)
    
    def test_chunk_text_returns_text_chunks(self):
        """Test that chunk_text returns a list of TextChunk objects."""
        service = EmbeddingGeneratorService(max_chunk_size=100)
        
        sample_text = "This is a sample text that will be chunked."
        
        result = service.chunk_text(sample_text, "test-page")
        
        # Should return a list
        assert isinstance(result, list)
        
        # Each item should be a TextChunk
        for item in result:
            assert isinstance(item, TextChunk)
    
    def test_embedding_contract_attributes(self):
        """Test that Embedding has the required attributes as per contract."""
        # Import the model
        from src.models.embedding import Embedding
        
        # Check that Embedding has all required attributes
        required_attrs = [
            'id', 'chunk_id', 'vector', 'model_name', 'created_at'
        ]
        
        # Check that all attributes exist
        for attr in required_attrs:
            assert hasattr(Embedding, attr) or attr in Embedding.__annotations__, \
                f"Embedding is missing required attribute: {attr}"
    
    def test_text_preprocessor_contract(self):
        """Test that TextPreprocessorService meets its contract."""
        from src.services.text_preprocessor import TextPreprocessorService
        
        service = TextPreprocessorService(max_chunk_size=100)
        
        # Check that the service has the required method
        assert hasattr(service, 'chunk_text')
        assert callable(getattr(service, 'chunk_text'))
        
        # Test that it returns TextChunk objects
        sample_text = "Sample text for testing." * 10  # Make it longer to ensure chunking
        chunks = service.chunk_text(sample_text, "test-page")
        
        assert isinstance(chunks, list)
        assert all(isinstance(chunk, TextChunk) for chunk in chunks)