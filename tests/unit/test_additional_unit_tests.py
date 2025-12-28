"""Additional unit tests for utility functions and edge cases."""
import pytest
from unittest.mock import Mock, patch
from src.lib.config import Config
from src.services.text_preprocessor import TextPreprocessorService
from src.models.book_page import BookPage
from src.models.text_chunk import TextChunk
from datetime import datetime


class TestConfigValidation:
    """Test configuration validation."""
    
    def test_config_validation_with_all_required_values(self):
        """Test config validation when all required values are present."""
        # Temporarily modify config values to valid ones
        original_api_key = Config.COHERE_API_KEY
        original_qdrant_key = Config.QDRANT_API_KEY
        original_qdrant_url = Config.QDRANT_URL
        original_qdrant_host = Config.QDRANT_HOST
        
        Config.COHERE_API_KEY = "test-key"
        Config.QDRANT_API_KEY = "test-key"
        Config.QDRANT_URL = "https://test.qdrant.com"
        Config.QDRANT_HOST = "localhost"
        
        errors = Config.validate()
        
        # Should have no errors with valid config
        assert len(errors) == 0
        
        # Restore original values
        Config.COHERE_API_KEY = original_api_key
        Config.QDRANT_API_KEY = original_qdrant_key
        Config.QDRANT_URL = original_qdrant_url
        Config.QDRANT_HOST = original_qdrant_host
    
    def test_config_validation_missing_cohere_key(self):
        """Test config validation when Cohere API key is missing."""
        original_api_key = Config.COHERE_API_KEY
        Config.COHERE_API_KEY = ""
        
        errors = Config.validate()
        
        assert "COHERE_API_KEY is required" in errors
        
        # Restore original value
        Config.COHERE_API_KEY = original_api_key
    
    def test_config_validation_missing_qdrant_key(self):
        """Test config validation when Qdrant API key is missing."""
        original_qdrant_key = Config.QDRANT_API_KEY
        Config.QDRANT_API_KEY = ""
        original_url = Config.QDRANT_URL
        Config.QDRANT_URL = ""
        original_host = Config.QDRANT_HOST
        Config.QDRANT_HOST = ""
        
        errors = Config.validate()
        
        assert "QDRANT_API_KEY is required" in errors
        assert "Either QDRANT_URL or QDRANT_HOST must be set" in errors
        
        # Restore original values
        Config.QDRANT_API_KEY = original_qdrant_key
        Config.QDRANT_URL = original_url
        Config.QDRANT_HOST = original_host


class TestTextPreprocessor:
    """Test text preprocessing functionality."""
    
    def test_chunk_text_with_short_content(self):
        """Test chunking of content shorter than max size."""
        preprocessor = TextPreprocessorService(max_chunk_size=100)
        
        short_text = "This is a short sentence."
        
        chunks = preprocessor.chunk_text(short_text, "test-page")
        
        assert len(chunks) == 1
        assert chunks[0].content == short_text
        assert chunks[0].page_id == "test-page"
    
    def test_chunk_text_with_long_content(self):
        """Test chunking of content longer than max size."""
        preprocessor = TextPreprocessorService(max_chunk_size=10)  # Small size for testing
        
        long_text = "This is sentence one. This is sentence two. This is sentence three. This is sentence four."
        
        chunks = preprocessor.chunk_text(long_text, "test-page")
        
        assert len(chunks) > 1  # Should be split into multiple chunks
        assert all(isinstance(chunk, TextChunk) for chunk in chunks)
        
        # Verify that the entire content is preserved across chunks
        total_content = "".join(chunk.content for chunk in chunks)
        # Remove extra spaces that may result from processing
        total_content = " ".join(total_content.split())
        original_clean = " ".join(long_text.split())
        assert original_clean in total_content or total_content in original_clean
    
    def test_preprocess_text_removes_extra_whitespace(self):
        """Test that preprocessing removes extra whitespace."""
        preprocessor = TextPreprocessorService(max_chunk_size=100)
        
        text_with_extra_spaces = "This   has    extra   spaces."
        
        result = preprocessor.preprocess_text(text_with_extra_spaces)
        
        # Should have normalized spaces
        assert "  " not in result  # No double spaces
        assert result == "This has extra spaces."


class TestModelValidation:
    """Test model validation."""
    
    def test_bookpage_validation(self):
        """Test BookPage model validation."""
        with pytest.raises(ValueError, match="BookPage.id cannot be empty"):
            BookPage(
                id="",
                url="https://example.com",
                title="Test",
                raw_html="<p>test</p>",
                extracted_text="test",
                headings=[],
                created_at=datetime.now(),
                updated_at=datetime.now()
            )
        
        with pytest.raises(ValueError, match="BookPage.url cannot be empty"):
            BookPage(
                id="test-id",
                url="",
                title="Test",
                raw_html="<p>test</p>",
                extracted_text="test",
                headings=[],
                created_at=datetime.now(),
                updated_at=datetime.now()
            )
        
        with pytest.raises(ValueError, match="BookPage.extracted_text cannot be empty"):
            BookPage(
                id="test-id",
                url="https://example.com",
                title="Test",
                raw_html="<p>test</p>",
                extracted_text="",
                headings=[],
                created_at=datetime.now(),
                updated_at=datetime.now()
            )
    
    def test_textchunk_validation(self):
        """Test TextChunk model validation."""
        with pytest.raises(ValueError, match="TextChunk.id cannot be empty"):
            TextChunk(
                id="",
                page_id="test-page",
                content="test content",
                start_pos=0,
                end_pos=11,
                chunk_index=0,
                tokens_count=10
            )
        
        with pytest.raises(ValueError, match="TextChunk.start_pos must be non-negative"):
            TextChunk(
                id="test-chunk",
                page_id="test-page",
                content="test content",
                start_pos=-1,
                end_pos=11,
                chunk_index=0,
                tokens_count=10
            )
        
        with pytest.raises(ValueError, match="TextChunk.tokens_count must be positive"):
            TextChunk(
                id="test-chunk",
                page_id="test-page",
                content="test content",
                start_pos=0,
                end_pos=11,
                chunk_index=0,
                tokens_count=0
            )