"""Integration tests for content extraction process."""
import pytest
from src.services.content_extractor import ContentExtractionService
from src.models.book_page import BookPage


class TestContentExtractionIntegration:
    """Integration tests for content extraction process."""
    
    def test_full_extraction_process(self):
        """Test the complete extraction process from URL to clean text."""
        service = ContentExtractionService()
        
        # We'll use a mock response for testing purposes
        # In real integration tests, we would use actual URLs
        import unittest.mock
        from unittest.mock import patch
        import requests
        
        # Example HTML content for testing
        html_content = '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>Test Book Page</title>
        </head>
        <body>
            <nav>Navigation content to be ignored</nav>
            <header>Header content to be ignored</header>
            <main>
                <h1>Chapter 1: Introduction</h1>
                <p>This is the first paragraph of the chapter.</p>
                <p>This is the second paragraph.</p>
                <h2>Subtopic</h2>
                <p>Content under subtopic heading.</p>
                <footer>Footer content to be ignored</footer>
            </main>
        </body>
        </html>
        '''
        
        mock_response = unittest.mock.Mock()
        mock_response.text = html_content
        mock_response.status_code = 200
        
        with patch.object(requests, 'get', return_value=mock_response):
            book_page = service.extract_text_from_url("https://example.com/test-page")
        
        # Assertions
        assert isinstance(book_page, BookPage)
        assert book_page.url == "https://example.com/test-page"
        assert "Introduction" in book_page.title
        assert "first paragraph of the chapter" in book_page.extracted_text
        assert "Subtopic" in book_page.headings
        assert len(book_page.headings) >= 1
        
        # Verify non-content elements are removed
        assert "Navigation content to be ignored" not in book_page.extracted_text
        assert "Header content to be ignored" not in book_page.extracted_text
        assert "Footer content to be ignored" not in book_page.extracted_text
    
    def test_multiple_headings_extraction(self):
        """Test extraction of multiple headings of different levels."""
        service = ContentExtractionService()
        
        import unittest.mock
        from unittest.mock import patch
        import requests
        
        html_content = '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>Multi-heading Test</title>
        </head>
        <body>
            <main>
                <h1>Main Title</h1>
                <h2>Section 1</h2>
                <p>Content for section 1.</p>
                <h3>Subsection 1.1</h3>
                <p>Content for subsection 1.1.</p>
                <h2>Section 2</h2>
                <p>Content for section 2.</p>
            </main>
        </body>
        </html>
        '''
        
        mock_response = unittest.mock.Mock()
        mock_response.text = html_content
        mock_response.status_code = 200
        
        with patch.object(requests, 'get', return_value=mock_response):
            book_page = service.extract_text_from_url("https://example.com/multi-heading")
        
        # Check that all headings are captured
        assert "Main Title" in book_page.headings
        assert "Section 1" in book_page.headings
        assert "Subsection 1.1" in book_page.headings
        assert "Section 2" in book_page.headings
        assert len(book_page.headings) == 4  # All headings should be captured