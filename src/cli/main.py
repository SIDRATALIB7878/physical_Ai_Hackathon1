"""Command-line interface for the book embeddings pipeline."""
import argparse
import sys
import json
from typing import List, Dict, Any

from src.services.pipeline_service import PipelineOrchestrationService
from src.lib.config import Config
from src.lib.logging import setup_logging


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Book Content Embeddings Pipeline CLI"
    )
    
    parser.add_argument(
        "--book-urls",
        type=str,
        help="Comma-separated list of book page URLs to process"
    )
    
    parser.add_argument(
        "--book-id",
        type=str,
        required=True,
        help="Unique identifier for the book being processed"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        help="Path to JSON configuration file"
    )
    
    parser.add_argument(
        "--chunk-size",
        type=int,
        help="Maximum number of tokens per chunk"
    )
    
    parser.add_argument(
        "--processing-rate",
        type=int,
        help="Processing rate limit (pages per hour)"
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )
    
    return parser.parse_args()


def load_config_from_file(config_path: str) -> Dict[str, Any]:
    """Load configuration from a JSON file."""
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config file {config_path}: {str(e)}")
        sys.exit(1)


def main():
    """Main CLI entry point."""
    args = parse_arguments()
    
    # Setup logging
    log_level = "DEBUG" if args.verbose else "INFO"
    logger = setup_logging(log_level)
    
    logger.info("Starting Book Content Embeddings Pipeline")
    
    # Validate configuration
    config_errors = Config.validate()
    if config_errors:
        logger.error("Configuration errors found:")
        for error in config_errors:
            logger.error(f"  - {error}")
        sys.exit(1)
    
    # Load configuration from file if provided
    if args.config:
        file_config = load_config_from_file(args.config)
        
        # Override config values from file
        if "book_urls" in file_config and not args.book_urls:
            args.book_urls = ",".join(file_config["book_urls"])
        if "book_id" in file_config:
            args.book_id = file_config["book_id"]
        if "chunk_size" in file_config:
            args.chunk_size = file_config["chunk_size"]
        if "processing_rate" in file_config:
            args.processing_rate = file_config["processing_rate"]
    
    # Process book URLs
    if args.book_urls:
        book_urls = [url.strip() for url in args.book_urls.split(',')]
    else:
        logger.error("No book URLs provided. Use --book-urls or specify in config file.")
        sys.exit(1)
    
    # Override config values if provided via command line
    if args.chunk_size:
        Config.CHUNK_SIZE = args.chunk_size
    if args.processing_rate:
        Config.PROCESSING_RATE = args.processing_rate
    
    # Create pipeline service and run
    try:
        pipeline_service = PipelineOrchestrationService()
        
        logger.info(f"Processing book '{args.book_id}' with {len(book_urls)} pages")
        logger.info(f"Using chunk size: {Config.CHUNK_SIZE}, processing rate: {Config.PROCESSING_RATE}")
        
        # Run the pipeline
        job_result = pipeline_service.run_pipeline(
            book_urls=book_urls,
            book_id=args.book_id
        )
        
        # Print results
        print(f"\nPipeline completed with status: {job_result.status}")
        print(f"Pages processed: {job_result.pages_processed}")
        print(f"Pages failed: {job_result.pages_failed}")
        print(f"Total pages: {job_result.pages_count}")
        
        if job_result.error_message:
            print(f"Error: {job_result.error_message}")
        
        logger.info(f"Pipeline completed with status: {job_result.status}")
        
        # Exit with error code if pipeline failed completely
        if job_result.status == "FAILED" and job_result.pages_failed == job_result.pages_count:
            sys.exit(1)
        
    except Exception as e:
        logger.error(f"Pipeline execution failed: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()