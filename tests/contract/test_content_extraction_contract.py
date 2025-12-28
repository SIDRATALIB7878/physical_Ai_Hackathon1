"""Contract tests for the content extraction service."""
import pytest
from src.services.content_extractor import ContentExtractionService
from src.models.book_page import BookPage
from src.lib.errors import ContentExtractionError


class TestContentExtractionContract:
    """Contract tests to ensure the content extraction service meets the API contract."""
    
    def test_extract_text_from_url_returns_book_page(self):
        """Test that extract_text_from_url returns a BookPage object as specified in the contract."""
        service = ContentExtractionService()
        
        # We'll mock the request as we're testing the contract, not the actual HTTP call
        with pytest.MonkeyPatch().context() as m:
            import requests
            import unittest.mock
            
            mock_response = unittest.mock.Mock()
            mock_response.text = '<html><head><title>Test</title></head><body><p>Test content</p></body></html>'
            mock_response.status_code = 200
            
            m.setattr(requests, 'get', lambda *args, **kwargs: mock_response)
            
            result = service.extract_text_from_url("https://example.com/test")
            
            # Contract specifies it should return BookPage
            assert isinstance(result, BookPage)
            
            # Check that required fields are present
            assert hasattr(result, 'id')
            assert hasattr(result, 'url')
            assert hasattr(result, 'title')
            assert hasattr(result, 'extracted_text')
            assert hasattr(result, 'headings')
    
    def test_extract_text_from_url_signature(self):
        """Test that extract_text_from_url has the correct signature as per contract."""
        service = ContentExtractionService()
        
        # Check the method exists
        assert hasattr(service, 'extract_text_from_url')
        
        # Check the method is callable
        assert callable(getattr(service, 'extract_text_from_url'))
    
    def test_extract_text_from_url_error_handling(self):
        """Test that extract_text_from_url raises ContentExtractionError on failure."""
        service = ContentExtractionService()
        
        # Mock a failed request
        with pytest.MonkeyPatch().context() as m:
            import requests
            import unittest.mock
            
            def failing_get(*args, **kwargs):
                raise Exception("Network error")
            
            m.setattr(requests, 'get', failing_get)
            
            with pytest.raises(ContentExtractionError):
                service.extract_text_from_url("https://example.com/test")
    
    def test_book_page_contract_attributes(self):
        """Test that BookPage has the required attributes as per contract."""
        # Import the model
        from src.models.book_page import BookPage
        
        # Check that BookPage has all required attributes
        required_attrs = [
            'id', 'url', 'title', 'raw_html', 'extracted_text', 'headings', 
            'created_at', 'updated_at'
        ]
        
        # Check that all attributes exist
        for attr in required_attrs:
            assert hasattr(BookPage, attr) or attr in BookPage.__annotations__, \
                f"BookPage is missing required attribute: {attr}"
    
    def test_book_page_creation(self):
        """Test that BookPage can be instantiated with the expected parameters."""
        from src.models.book_page import BookPage
        import datetime
        
        # Create a BookPage instance
        book_page = BookPage(
            id="test-id",
            url="https://example.com/test",
            title="Test Page",
            raw_html="<html><head><title>Test</title></head><body><p>Test content</p></body></html>",
            extracted_text="Test content",
            headings=["Test"],
            created_at=datetime.datetime.now(),
            updated_at=datetime.datetime.now()
        )
        
        # Verify all attributes are set correctly
        assert book_page.id == "test-id"
        assert book_page.url == "https://example.com/test"
        assert book_page.title == "Test Page"
        assert "Test content" in book_page.raw_html
        assert book_page.extracted_text == "Test content"
        assert book_page.headings == ["Test"]