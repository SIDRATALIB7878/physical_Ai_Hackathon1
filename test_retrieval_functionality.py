#!/usr/bin/env python3
"""
Test to check if the RAG agent properly uses the retrieval tool
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_rag_agent_with_book_query():
    """Test the RAG agent with a book-related query."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        
        print("Creating RAG Agent...")
        agent = RAGAgent()
        print("RAG Agent created successfully")
        
        print("\nTesting with a general greeting (might not trigger retrieval):")
        try:
            response = agent.ask("Hello")
            print(f"Response: {response}")
        except Exception as e:
            print(f"Error during ask(): {e}")
        
        print("\nTesting with a book-related query (should trigger retrieval):")
        try:
            response = agent.ask("What does the book say about artificial intelligence?")
            print(f"Response: {response}")
        except Exception as e:
            print(f"Error during ask(): {e}")
        
        print("\nTesting with another book-related query:")
        try:
            response = agent.ask("Explain quantum computing as described in the book")
            print(f"Response: {response}")
        except Exception as e:
            print(f"Error during ask(): {e}")
        
        print("\nTesting with a specific concept query:")
        try:
            response = agent.ask("What is the Perception-Action Cycle of Physical AI?")
            print(f"Response: {response}")
        except Exception as e:
            print(f"Error during ask(): {e}")
        
        return True
        
    except Exception as e:
        print(f"Error creating RAG Agent: {e}")
        print(f"Error type: {type(e).__name__}")
        return False

def test_tool_directly():
    """Test the retrieval tool directly to see if it works."""
    try:
        from backend.rag_agent.rag_agent import retrieve_book_content_tool
        
        print("\nTesting retrieval tool directly:")
        try:
            result = retrieve_book_content_tool("artificial intelligence", 3)
            print(f"Direct tool result: {result}")
            print(f"Content: {result.content[:200]}...")  # First 200 chars
        except Exception as e:
            print(f"Error calling tool directly: {e}")
            print(f"Error type: {type(e).__name__}")
        
        return True
    except Exception as e:
        print(f"Error testing tool directly: {e}")
        return False

def main():
    print("Testing RAG Agent retrieval functionality...")
    print()
    
    test_rag_agent_with_book_query()
    test_tool_directly()
    
    print()
    print("Note: If the agent is not calling the retrieval tool, it might be")
    print("ignoring the instructions to use the tool before answering.")

if __name__ == "__main__":
    main()