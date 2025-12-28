"""Test script for the RAG agent."""
import os
import sys
import json
from dotenv import load_dotenv

# Add the project root to the path so we can import modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

# Load environment variables
load_dotenv()

try:
    from backend.rag_agent.rag_agent import RAGAgentSimple
    from backend.rag_agent.retrieval_tool import retrieve_book_content
    print("Successfully imported RAG agent modules")
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)


def test_retrieval_tool():
    """Test the retrieval tool directly."""
    print("Testing retrieval tool...")

    # Test with a sample query
    query = "What is the main concept of artificial intelligence?"
    try:
        results = retrieve_book_content(query, "default_book", 3)

        print(f"Query: {query}")
        print(f"Results: {results}")

        # Parse and display the results nicely
        try:
            parsed_results = json.loads(results)
            print("\nFormatted results:")
            for i, result in enumerate(parsed_results):
                print(f"Result {i+1}:")
                print(f"  Content: {result['content'][:100]}...")  # First 100 chars
                print(f"  Score: {result['score']}")
                print(f"  Page: {result['page_title']}")
                print()
        except Exception as e:
            print(f"Error parsing results: {e}")
    except Exception as e:
        print(f"Error calling retrieval tool: {e}")
        print("This might be expected if Qdrant is not configured or no embeddings exist yet.")


def test_rag_agent():
    """Test the RAG agent."""
    print("Testing RAG agent...")

    # Check if OpenAI API key is available
    openai_key = os.getenv("OPENAI_API_KEY")
    if not openai_key:
        print("OPENAI_API_KEY not found in environment variables. Please set it before running this test.")
        return

    try:
        # Create the agent
        agent = RAGAgentSimple()

        # Test with a sample question
        question = "What is the main concept of artificial intelligence?"

        print(f"Question: {question}")
        response = agent.ask(question, "default_book", 3)
        print(f"Response: {response}")
    except Exception as e:
        print(f"Error testing RAG agent: {e}")
        print("Note: This might be due to Python 3.14 compatibility issues with the OpenAI library.")


if __name__ == "__main__":
    print("Running RAG agent tests...\n")

    test_retrieval_tool()
    print("\n" + "="*50 + "\n")
    test_rag_agent()