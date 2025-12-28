"""Service for extracting text content from web pages."""
import requests
from bs4 import BeautifulSoup
from typing import List
from urllib.parse import urljoin, urlparse
import time
from datetime import datetime
import logging

from src.models.book_page import BookPage
from src.lib.errors import ContentExtractionError
from src.lib.config import Config


class ContentExtractionService:
    """Service for extracting clean text content from web pages."""
    
    def __init__(self, timeout: int = None, max_retries: int = 3):
        """
        Initialize the content extraction service.
        
        Args:
            timeout: Request timeout in seconds
            max_retries: Maximum number of retry attempts
        """
        self.timeout = timeout or Config.REQUEST_TIMEOUT
        self.max_retries = max_retries
        self.session = requests.Session()
        # Set a user agent to avoid being blocked by some sites
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })
        
        # Setup logging
        self.logger = logging.getLogger("book_embeddings_pipeline.content_extraction")
        
    def extract_text_from_url(self, url: str) -> BookPage:
        """
        Extracts clean text content from a web page at the given URL.
        
        Args:
            url: The URL of the book page to extract content from
            
        Returns:
            BookPage: An object containing the extracted content and metadata
            
        Raises:
            ContentExtractionError: If content cannot be extracted from the URL
        """
        # Validate URL
        if not self._is_valid_url(url):
            raise ContentExtractionError(f"Invalid URL format: {url}")
        
        # Attempt to retrieve the page content with retries
        response = self._fetch_with_retry(url)
        
        if response.status_code != 200:
            raise ContentExtractionError(
                f"Failed to retrieve page {url}. Status code: {response.status_code}"
            )
        
        # Parse the HTML content
        soup = BeautifulSoup(response.text, 'html.parser')
        
        # Extract the title
        title = self._extract_title(soup)
        
        # Extract headings
        headings = self._extract_headings(soup)
        
        # Extract clean text content
        clean_text = self._extract_clean_text(soup)
        
        if not clean_text.strip():
            raise ContentExtractionError(f"No content extracted from page: {url}")
        
        # Create and return BookPage instance
        book_page = BookPage(
            id=url.split('/')[-1].replace('.html', '').replace('.htm', '') or url.split('/')[-1][:10],
            url=url,
            title=title,
            raw_html=response.text,
            extracted_text=clean_text,
            headings=headings,
            created_at=datetime.now(),
            updated_at=datetime.now()
        )
        
        self.logger.info(f"Successfully extracted content from {url}")
        return book_page
    
    def _fetch_with_retry(self, url: str) -> requests.Response:
        """
        Fetch URL content with retry logic.
        
        Args:
            url: The URL to fetch
            
        Returns:
            Response object from the request
        """
        last_exception = None
        
        for attempt in range(self.max_retries):
            try:
                response = self.session.get(url, timeout=self.timeout)
                if response.status_code == 200:
                    return response
                elif response.status_code < 500:
                    # Client error, no point retrying
                    break
            except requests.RequestException as e:
                self.logger.warning(f"Attempt {attempt + 1} failed for {url}: {str(e)}")
                last_exception = e
                if attempt < self.max_retries - 1:
                    time.sleep(2 ** attempt)  # Exponential backoff
        
        # If we've exhausted retries, raise an error
        if last_exception:
            raise ContentExtractionError(f"Failed to fetch {url} after {self.max_retries} attempts: {str(last_exception)}")
        else:
            raise ContentExtractionError(f"Failed to fetch {url} after {self.max_retries} attempts, status: {response.status_code}")
    
    def _is_valid_url(self, url: str) -> bool:
        """
        Validate if the provided string is a valid URL.
        
        Args:
            url: The URL to validate
            
        Returns:
            True if the URL is valid
        """
        try:
            result = urlparse(url)
            return all([result.scheme, result.netloc])
        except Exception:
            return False
    
    def _extract_title(self, soup: BeautifulSoup) -> str:
        """
        Extract the title from the HTML soup.
        
        Args:
            soup: BeautifulSoup object of the page
            
        Returns:
            The extracted title or empty string if not found
        """
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()
        return ""
    
    def _extract_headings(self, soup: BeautifulSoup) -> List[str]:
        """
        Extract all headings (h1-h6) from the HTML soup.
        
        Args:
            soup: BeautifulSoup object of the page
            
        Returns:
            List of heading texts
        """
        headings = []
        for i in range(1, 7):  # h1 through h6
            heading_tags = soup.find_all(f'h{i}')
            for tag in heading_tags:
                heading_text = tag.get_text().strip()
                if heading_text:
                    headings.append(heading_text)
        return headings
    
    def _extract_clean_text(self, soup: BeautifulSoup) -> str:
        """
        Extract clean text content from the HTML soup.
        Removes navigation, ads, and other non-content elements.
        
        Args:
            soup: BeautifulSoup object of the page
            
        Returns:
            Clean text content
        """
        # Remove unwanted tags
        for tag_name in ['nav', 'header', 'footer', 'aside', 'script', 'style', 'noscript', 'meta']:
            for tag in soup.find_all(tag_name):
                tag.decompose()
        
        # Try to find the main content area first
        main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=lambda x: x and ('content' in x or 'main' in x or 'body' in x))
        
        if main_content:
            # If a main content area is found, extract text from it
            text = main_content.get_text(separator=' ')
        else:
            # Otherwise, extract from the body
            body = soup.find('body')
            if body:
                text = body.get_text(separator=' ')
            else:
                text = soup.get_text(separator=' ')
        
        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        clean_text = ' '.join(chunk for chunk in chunks if chunk)
        
        return clean_text