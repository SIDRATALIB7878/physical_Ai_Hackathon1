"""Unit tests for the Embedding Generation Service."""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.embedding_generator import EmbeddingGeneratorService
from src.models.embedding import Embedding
from src.models.text_chunk import TextChunk
from src.lib.errors import EmbeddingGenerationError


class TestEmbeddingGeneratorService:
    """Test class for EmbeddingGeneratorService."""
    
    def test_generate_embedding_success(self):
        """Test successful embedding generation."""
        service = EmbeddingGeneratorService()
        
        # Create a test text chunk
        text_chunk = TextChunk(
            id="test-chunk-1",
            page_id="test-page-1",
            content="This is a test sentence for embedding generation.",
            start_pos=0,
            end_pos=50,
            chunk_index=0,
            tokens_count=10
        )
        
        # Mock the Cohere API response
        mock_embeddings = Mock()
        mock_embeddings.embeddings = [[0.1, 0.2, 0.3, 0.4]]  # Simplified embedding
        
        with patch.object(service.co, 'embed', return_value=mock_embeddings):
            embedding = service.generate_embedding(text_chunk)
        
        assert isinstance(embedding, Embedding)
        assert embedding.chunk_id == "test-chunk-1"
        assert len(embedding.vector) == 4  # Based on our mock
        assert embedding.model_name == "embed-english-light-v2.0"
    
    def test_generate_embedding_empty_content(self):
        """Test handling of empty content."""
        service = EmbeddingGeneratorService()
        
        # Create a text chunk with empty content
        text_chunk = TextChunk(
            id="test-chunk-2",
            page_id="test-page-1",
            content="",
            start_pos=0,
            end_pos=0,
            chunk_index=0,
            tokens_count=0
        )
        
        with pytest.raises(EmbeddingGenerationError):
            service.generate_embedding(text_chunk)
    
    def test_generate_embedding_cohere_api_error(self):
        """Test handling of Cohere API errors."""
        service = EmbeddingGeneratorService()
        
        # Create a test text chunk
        text_chunk = TextChunk(
            id="test-chunk-3",
            page_id="test-page-1",
            content="This is a test sentence for embedding generation.",
            start_pos=0,
            end_pos=50,
            chunk_index=0,
            tokens_count=10
        )
        
        # Mock the Cohere API to raise an exception
        with patch.object(service.co, 'embed', side_effect=Exception("API Error")):
            with pytest.raises(EmbeddingGenerationError):
                service.generate_embedding(text_chunk)
                
    def test_chunk_text_success(self):
        """Test successful text chunking."""
        service = EmbeddingGeneratorService(max_chunk_size=100)
        
        long_text = "This is a sentence. " * 50  # Create text longer than chunk size
        
        text_chunks = service.chunk_text(long_text, "test-page-1")
        
        assert len(text_chunks) > 1  # Should be split into multiple chunks
        assert all(isinstance(chunk, TextChunk) for chunk in text_chunks)
        assert all(len(chunk.content) <= 2000 for chunk in text_chunks)  # Approximate check (100 tokens is roughly 4-5 chars each = ~2000 chars)
        
    def test_chunk_text_short_text(self):
        """Test chunking of text shorter than max size."""
        service = EmbeddingGeneratorService(max_chunk_size=1000)
        
        short_text = "This is a short sentence."
        
        text_chunks = service.chunk_text(short_text, "test-page-1")
        
        assert len(text_chunks) == 1
        assert text_chunks[0].content == short_text
        assert text_chunks[0].page_id == "test-page-1"