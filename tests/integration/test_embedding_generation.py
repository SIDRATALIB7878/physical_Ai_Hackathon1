"""Integration tests for embedding generation process."""
import pytest
from src.services.embedding_generator import EmbeddingGeneratorService
from src.services.text_preprocessor import TextPreprocessorService
from src.models.text_chunk import TextChunk


class TestEmbeddingGenerationIntegration:
    """Integration tests for embedding generation process."""
    
    def test_full_embedding_generation_process(self):
        """Test the complete process from text chunk to embedding."""
        # Since we can't make real API calls in tests, we'll use a mock
        from unittest.mock import patch, Mock
        
        # Initialize services
        preprocessor = TextPreprocessorService(max_chunk_size=100)
        generator = EmbeddingGeneratorService()
        
        # Sample text to process
        sample_text = "This is a sample sentence for testing the embedding generation process."
        
        # Test the text preprocessing
        chunks = preprocessor.chunk_text(sample_text, "test-page-1")
        
        assert len(chunks) == 1
        assert isinstance(chunks[0], TextChunk)
        assert "sample sentence" in chunks[0].content
        
        # Test embedding generation (with mocked API call)
        mock_embeddings = Mock()
        # Simulate Cohere embeddings response - 384 dimensions for embed-english-light-v2.0
        mock_embeddings.embeddings = [list(range(384))]  # Mock embedding vector
        
        with patch.object(generator.co, 'embed', return_value=mock_embeddings):
            embedding = generator.generate_embedding(chunks[0])
        
        # Verify the embedding was created correctly
        assert embedding.chunk_id == chunks[0].id
        assert len(embedding.vector) == 384  # Expected size for embed-english-light-v2.0
        assert embedding.model_name == "embed-english-light-v2.0"
    
    def test_chunking_and_embedding_integration(self):
        """Test the integration between text chunking and embedding generation."""
        from unittest.mock import patch, Mock
        
        preprocessor = TextPreprocessorService(max_chunk_size=50)
        generator = EmbeddingGeneratorService()
        
        # Create a longer text that should be split into multiple chunks
        long_text = "This is the first sentence. " * 20  # Multiple sentences
        
        # Preprocess the text into chunks
        chunks = preprocessor.chunk_text(long_text, "test-page-2")
        
        assert len(chunks) > 1  # Should be multiple chunks
        
        # Generate embeddings for each chunk
        mock_embeddings = Mock()
        # Mock embedding vector (384 dimensions)
        mock_embeddings.embeddings = [list(range(384))]
        
        embeddings = []
        with patch.object(generator.co, 'embed', return_value=mock_embeddings):
            for chunk in chunks:
                embedding = generator.generate_embedding(chunk)
                embeddings.append(embedding)
        
        # Verify all chunks have embeddings
        assert len(embeddings) == len(chunks)
        for embedding in embeddings:
            assert len(embedding.vector) == 384  # Expected size
            assert embedding.model_name == "embed-english-light-v2.0"
    
    def test_embedding_generation_with_special_chars(self):
        """Test embedding generation with text containing special characters."""
        from unittest.mock import patch, Mock
        
        generator = EmbeddingGeneratorService()
        
        # Create a text chunk with special characters
        text_chunk = TextChunk(
            id="special-chars-test",
            page_id="test-page-3",
            content="Text with special chars: émojis, punctuation! numbers: 123456789.",
            start_pos=0,
            end_pos=70,
            chunk_index=0,
            tokens_count=20
        )
        
        # Mock the Cohere API response
        mock_embeddings = Mock()
        mock_embeddings.embeddings = [list(range(384))]  # Mock embedding vector
        
        with patch.object(generator.co, 'embed', return_value=mock_embeddings):
            embedding = generator.generate_embedding(text_chunk)
        
        # Verify the embedding was created correctly
        assert embedding.chunk_id == "special-chars-test"
        assert len(embedding.vector) == 384  # Expected size
        assert embedding.model_name == "embed-english-light-v2.0"