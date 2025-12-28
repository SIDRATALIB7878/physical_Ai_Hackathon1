"""Unit tests for the Content Extraction Service."""
import pytest
from unittest.mock import Mock, patch, mock_open
from src.services.content_extractor import ContentExtractionService
from src.models.book_page import BookPage
from src.lib.errors import ContentExtractionError


class TestContentExtractionService:
    """Test class for ContentExtractionService."""
    
    def test_extract_text_from_url_success(self):
        """Test successful text extraction from a URL."""
        service = ContentExtractionService()
        
        # Mock the requests.get response
        mock_response = Mock()
        mock_response.text = '''
        <html>
            <head>
                <title>Sample Page</title>
            </head>
            <body>
                <h1>Main Title</h1>
                <p>First paragraph with some text.</p>
                <p>Second paragraph with more text.</p>
                <h2>Subheading</h2>
                <p>Paragraph under subheading.</p>
            </body>
        </html>
        '''
        mock_response.status_code = 200
        
        with patch('requests.get', return_value=mock_response):
            book_page = service.extract_text_from_url("https://example.com/page1")
            
        assert isinstance(book_page, BookPage)
        assert book_page.url == "https://example.com/page1"
        assert book_page.title == "Sample Page"
        assert "First paragraph with some text" in book_page.extracted_text
        assert "Subheading" in book_page.headings
    
    def test_extract_text_from_url_with_special_chars(self):
        """Test text extraction with special characters."""
        service = ContentExtractionService()
        
        # Mock the requests.get response with special characters
        mock_response = Mock()
        mock_response.text = '''
        <html>
            <head>
                <title>Page with &lt;special&gt; chars</title>
            </head>
            <body>
                <h1>Special Chars Test</h1>
                <p>Text with &amp; symbols and other entities.</p>
            </body>
        </html>
        '''
        mock_response.status_code = 200
        
        with patch('requests.get', return_value=mock_response):
            book_page = service.extract_text_from_url("https://example.com/page2")
            
        assert "special" in book_page.title.lower()
        assert "symbols and other entities" in book_page.extracted_text
    
    def test_extract_text_from_url_connection_error(self):
        """Test handling of connection errors."""
        service = ContentExtractionService()
        
        with patch('requests.get', side_effect=Exception("Connection failed")):
            with pytest.raises(ContentExtractionError):
                service.extract_text_from_url("https://example.com/bad_url")
    
    def test_extract_text_from_url_http_error(self):
        """Test handling of HTTP errors."""
        service = ContentExtractionService()
        
        mock_response = Mock()
        mock_response.status_code = 404
        
        with patch('requests.get', return_value=mock_response):
            with pytest.raises(ContentExtractionError):
                service.extract_text_from_url("https://example.com/not_found")
    
    def test_extract_text_from_url_empty_content(self):
        """Test handling of empty content."""
        service = ContentExtractionService()
        
        mock_response = Mock()
        mock_response.text = ""
        mock_response.status_code = 200
        
        with patch('requests.get', return_value=mock_response):
            with pytest.raises(ContentExtractionError):
                service.extract_text_from_url("https://example.com/empty")