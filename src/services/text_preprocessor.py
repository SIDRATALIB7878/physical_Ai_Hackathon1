"""Service for preprocessing text before embedding generation."""
import re
from typing import List
from src.models.text_chunk import TextChunk
from src.lib.config import Config
import uuid


class TextPreprocessorService:
    """Service for preprocessing text content before embedding generation."""
    
    def __init__(self, max_chunk_size: int = None):
        """
        Initialize the text preprocessing service.
        
        Args:
            max_chunk_size: Maximum number of tokens per chunk (default from config)
        """
        self.max_chunk_size = max_chunk_size or Config.CHUNK_SIZE
        # Approximate characters per token (typically 4-5 characters per token)
        self.chars_per_token = 4
        
    def chunk_text(self, content: str, page_id: str) -> List[TextChunk]:
        """
        Chunks a text content into smaller segments suitable for embedding.
        
        Args:
            content: The text content to chunk
            page_id: The ID of the page this content comes from
            
        Returns:
            List of TextChunk objects derived from the content
        """
        if not content:
            return []
            
        # Estimate max characters based on token count
        max_chars = self.max_chunk_size * self.chars_per_token
        
        # Split content into sentences or paragraphs to maintain semantic meaning
        sentences = self._split_into_sentences(content)
        
        chunks = []
        current_chunk_content = ""
        start_pos = 0
        chunk_index = 0
        
        for sentence in sentences:
            # Check if adding this sentence would exceed the chunk size
            if len(current_chunk_content + sentence) <= max_chars:
                current_chunk_content += sentence
            else:
                # If the current chunk has content, save it
                if current_chunk_content.strip():
                    chunk = self._create_text_chunk(
                        page_id, current_chunk_content.strip(), start_pos, chunk_index
                    )
                    chunks.append(chunk)
                    
                    # Update position tracking
                    start_pos += len(current_chunk_content)
                    chunk_index += 1
                    
                    # Start a new chunk with the current sentence
                    current_chunk_content = sentence
                else:
                    # Handle the case where a single sentence is longer than max_chars
                    # Split it into smaller pieces
                    sub_chunks = self._split_long_sentence(sentence, max_chars)
                    for sub_chunk in sub_chunks[:-1]:  # All but the last
                        chunk = self._create_text_chunk(
                            page_id, sub_chunk.strip(), start_pos, chunk_index
                        )
                        chunks.append(chunk)
                        start_pos += len(sub_chunk)
                        chunk_index += 1
                    
                    # The last sub-chunk becomes the current chunk content
                    current_chunk_content = sub_chunks[-1]
        
        # Add the last chunk if it has content
        if current_chunk_content.strip():
            chunk = self._create_text_chunk(
                page_id, current_chunk_content.strip(), start_pos, chunk_index
            )
            chunks.append(chunk)
        
        return chunks
    
    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences preserving the sentence endings.
        
        Args:
            text: The text to split into sentences
            
        Returns:
            List of sentences
        """
        # Use regex to split on sentence endings, keeping the delimiters
        sentence_endings = r'([.!?]+["\']?)\s+'
        parts = re.split(sentence_endings, text)
        
        sentences = []
        current_sentence = ""
        
        for part in parts:
            current_sentence += part
            # If part matches sentence endings, it's time to save the sentence
            if re.match(sentence_endings.strip() + r'$', part):
                sentences.append(current_sentence)
                current_sentence = ""
        
        # Add any remaining text as a sentence if it's not empty
        if current_sentence.strip():
            sentences.append(current_sentence)
        
        return sentences
    
    def _split_long_sentence(self, sentence: str, max_chars: int) -> List[str]:
        """
        Split a sentence that's too long into smaller chunks.
        
        Args:
            sentence: The long sentence to split
            max_chars: Maximum characters per chunk
            
        Returns:
            List of text chunks
        """
        if len(sentence) <= max_chars:
            return [sentence]
            
        chunks = []
        words = sentence.split()
        
        current_chunk = ""
        for word in words:
            if len(current_chunk + " " + word) <= max_chars:
                if current_chunk:
                    current_chunk += " " + word
                else:
                    current_chunk = word
            else:
                if current_chunk:  # If there's anything accumulated, save it
                    chunks.append(current_chunk)
                    current_chunk = word
                else:  # If a single word is longer than max_chars, split it
                    while len(word) > max_chars:
                        chunks.append(word[:max_chars])
                        word = word[max_chars:]
                    if word:  # Add the remainder
                        current_chunk = word
        
        if current_chunk:
            chunks.append(current_chunk)
            
        return chunks
    
    def _create_text_chunk(self, page_id: str, content: str, start_pos: int, 
                          chunk_index: int) -> TextChunk:
        """
        Create a TextChunk object with appropriate fields.
        
        Args:
            page_id: The ID of the parent page
            content: The chunk content
            start_pos: Start position in the original text
            chunk_index: Sequential index of this chunk
            
        Returns:
            TextChunk object
        """
        chunk_id = f"{page_id}_chunk_{chunk_index}"
        
        # Approximate token count (4-5 characters per token)
        tokens_count = len(content) // self.chars_per_token
        
        return TextChunk(
            id=chunk_id,
            page_id=page_id,
            content=content,
            start_pos=start_pos,
            end_pos=start_pos + len(content),
            chunk_index=chunk_index,
            tokens_count=tokens_count
        )
    
    def preprocess_text(self, content: str) -> str:
        """
        Apply preprocessing transformations to the text before chunking.
        
        Args:
            content: The raw text content to preprocess
            
        Returns:
            Clean, preprocessed text
        """
        # Remove extra whitespace
        content = re.sub(r'\s+', ' ', content)
        
        # Remove special characters or normalize them if necessary
        # (specific preprocessing depends on the use case)
        
        return content.strip()