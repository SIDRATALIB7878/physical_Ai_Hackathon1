"""Test script for the RAG agent using mock retrieval tool."""
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
    from backend.rag_agent.mock_retrieval_tool import retrieve_book_content
    print("Successfully imported RAG agent modules with mock retrieval")
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)


def test_retrieval_tool():
    """Test the mock retrieval tool."""
    print("Testing mock retrieval tool...")
    
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


def test_agent_logic():
    """Test the agent logic with mock data (without calling OpenAI API)."""
    print("Testing agent logic with mock data...")
    
    # Show how the agent would format the prompt with retrieved content
    question = "What is the main concept of artificial intelligence?"
    retrieved_content = retrieve_book_content(question, "default_book", 3)
    
    # Format the retrieved content for the prompt (this is what the agent would do)
    context = f"Based on the following book content, answer the question: '{question}'\n\n"
    context += f"Relevant book content:\n{retrieved_content}\n\n"
    context += "Please provide a detailed answer based on the provided content, citing specific sections when possible."
    
    print("Formatted context that would be sent to OpenAI:")
    print("="*50)
    print(context)
    print("="*50)
    
    print("\nThis is the context that would be sent to the OpenAI API in a real implementation.")
    print("The agent would then return the AI-generated response based on this context.")


if __name__ == "__main__":
    print("Running RAG agent tests with mock retrieval...\n")
    
    test_retrieval_tool()
    print("\n" + "="*50 + "\n")
    test_agent_logic()