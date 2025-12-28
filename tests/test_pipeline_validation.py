#!/usr/bin/env python3
"""Pipeline validation and testing script for the book embeddings pipeline."""

import sys
import os
from typing import List, Dict, Any
import tempfile
import json

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.pipeline_service import PipelineOrchestrationService
from src.services.content_extractor import ContentExtractionService
from src.services.embedding_generator import EmbeddingGeneratorService
from src.services.vector_storage import VectorStorageService
from src.lib.config import Config
from src.lib.logging import setup_logging


def validate_config() -> List[str]:
    """Validate the configuration for the pipeline."""
    print("Validating configuration...")
    errors = Config.validate()
    
    if errors:
        print("❌ Configuration validation failed:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("✅ Configuration validation passed")
    
    return errors


def validate_services() -> bool:
    """Validate that all services can be initialized."""
    print("\nValidating services initialization...")
    
    try:
        # Try to initialize each service
        content_extractor = ContentExtractionService()
        embedding_generator = EmbeddingGeneratorService()
        vector_storage = VectorStorageService()
        
        print("✅ All services initialized successfully")
        return True
    except Exception as e:
        print(f"❌ Service initialization failed: {str(e)}")
        return False


def test_content_extraction() -> bool:
    """Test content extraction functionality."""
    print("\nTesting content extraction...")
    
    # Use a simple HTML string for testing
    test_html = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <h1>Main Title</h1>
            <p>This is a test paragraph.</p>
            <h2>Subtitle</h2>
            <p>Another paragraph for testing.</p>
        </body>
    </html>
    """
    
    try:
        from unittest.mock import Mock, patch
        import requests
        
        # Mock the requests.get response
        mock_response = Mock()
        mock_response.text = test_html
        mock_response.status_code = 200
        
        content_extractor = ContentExtractionService()
        
        with patch('requests.get', return_value=mock_response):
            book_page = content_extractor.extract_text_from_url("https://example.com/test")
        
        # Validate the extracted content
        assert book_page is not None
        assert book_page.title == "Test Page"
        assert "test paragraph" in book_page.extracted_text
        assert "Main Title" in book_page.headings
        
        print("✅ Content extraction test passed")
        return True
    except Exception as e:
        print(f"❌ Content extraction test failed: {str(e)}")
        return False


def test_embedding_generation() -> bool:
    """Test embedding generation functionality."""
    print("\nTesting embedding generation...")
    
    try:
        from src.models.text_chunk import TextChunk
        from datetime import datetime
        
        # Create a test text chunk
        test_chunk = TextChunk(
            id="test-chunk",
            page_id="test-page",
            content="This is a test sentence for embedding generation.",
            start_pos=0,
            end_pos=50,
            chunk_index=0,
            tokens_count=10
        )
        
        embedding_generator = EmbeddingGeneratorService()
        
        # Since we can't call the real API in tests, we'll test the preprocessor
        # which is part of the embedding generator
        chunks = embedding_generator.chunk_text(
            "This is a test sentence. " * 20,  # Create longer text to ensure chunking
            "test-page"
        )
        
        # Check that text was properly chunked
        assert len(chunks) > 0
        assert all(len(chunk.content) > 0 for chunk in chunks)
        
        print("✅ Embedding generation test (preprocessing) passed")
        return True
    except Exception as e:
        print(f"❌ Embedding generation test failed: {str(e)}")
        return False


def test_vector_storage() -> bool:
    """Test vector storage functionality."""
    print("\nTesting vector storage...")
    
    try:
        # We can't test actual storage without a Qdrant instance,
        # but we can test the initialization and basic functionality
        vector_storage = VectorStorageService()
        
        # Check that the service initialized properly
        assert vector_storage.qdrant_client is not None
        
        print("✅ Vector storage initialization test passed")
        return True
    except Exception as e:
        print(f"❌ Vector storage test failed: {str(e)}")
        return False


def test_pipeline_integration() -> bool:
    """Test pipeline integration (without actual API calls)."""
    print("\nTesting pipeline integration...")
    
    try:
        # Create a simple pipeline service
        # We'll test that it can be initialized and has the expected methods
        pipeline_service = PipelineOrchestrationService()
        
        # Check that required methods exist
        assert hasattr(pipeline_service, 'run_pipeline')
        assert hasattr(pipeline_service, 'run_pipeline_with_duplicate_handling')
        
        print("✅ Pipeline integration test passed")
        return True
    except Exception as e:
        print(f"❌ Pipeline integration test failed: {str(e)}")
        return False


def run_pipeline_with_test_data() -> bool:
    """Run the pipeline with test data (mocked to avoid real API calls)."""
    print("\nTesting pipeline execution with mocked services...")
    
    try:
        from unittest.mock import Mock, patch
        import requests
        from src.models.book_page import BookPage
        from datetime import datetime
        import uuid
        
        # Create pipeline service
        pipeline_service = PipelineOrchestrationService()
        
        # Mock the content extraction
        book_page = BookPage(
            id=str(uuid.uuid4()),
            url="https://example.com/test",
            title="Test Page",
            raw_html="<html><head><title>Test</title></head><body><p>Test content</p></body></html>",
            extracted_text="Test content",
            headings=["Test"],
            created_at=datetime.now(),
            updated_at=datetime.now()
        )
        
        # Mock all the methods that would make external calls
        with patch.object(pipeline_service.content_extractor, 'extract_text_from_url', return_value=book_page), \
             patch.object(pipeline_service.embedding_generator, 'chunk_text', return_value=[
                 Mock(id="chunk1", content="Test content", tokens_count=10, start_pos=0, end_pos=12, chunk_index=0)
             ]), \
             patch.object(pipeline_service.embedding_generator, 'generate_embedding', return_value=Mock(
                 id="emb1", chunk_id="chunk1", vector=[0.1]*384, model_name="test-model", created_at=datetime.now()
             )), \
             patch.object(pipeline_service.vector_storage, 'store_embedding', return_value=True):
            
            # Run the pipeline with mocked services
            job_result = pipeline_service.run_pipeline(
                book_urls=["https://example.com/test"],
                book_id="test-book"
            )
        
        # Check that pipeline completed
        assert job_result.status in ["COMPLETED", "COMPLETED_WITH_ERRORS"]
        
        print("✅ Pipeline execution test with mocked services passed")
        return True
    except Exception as e:
        print(f"❌ Pipeline execution test failed: {str(e)}")
        return False


def main():
    """Run all validation tests."""
    print("🔍 Running pipeline validation and testing script...\n")
    
    # Setup logging
    logger = setup_logging()
    
    # Run all validation tests
    validation_results = [
        ("Configuration Validation", validate_config, lambda r: len(r) == 0),
        ("Services Initialization", validate_services, lambda r: r),
        ("Content Extraction", test_content_extraction, lambda r: r),
        ("Embedding Generation", test_embedding_generation, lambda r: r),
        ("Vector Storage", test_vector_storage, lambda r: r),
        ("Pipeline Integration", test_pipeline_integration, lambda r: r),
        ("Pipeline Execution", run_pipeline_with_test_data, lambda r: r)
    ]
    
    passed_tests = 0
    total_tests = len(validation_results)
    
    for test_name, test_func, check_result in validation_results:
        try:
            result = test_func()
            if check_result(result):
                print(f"✅ {test_name}: PASSED")
                passed_tests += 1
            else:
                print(f"❌ {test_name}: FAILED")
        except Exception as e:
            print(f"❌ {test_name}: ERROR - {str(e)}")
    
    # Print summary
    print(f"\n📊 Test Summary: {passed_tests}/{total_tests} tests passed")
    
    if passed_tests == total_tests:
        print("🎉 All validation tests passed!")
        return 0
    else:
        print(f"⚠️ {total_tests - passed_tests} test(s) failed.")
        return 1 if passed_tests < total_tests else 0


if __name__ == "__main__":
    sys.exit(main())