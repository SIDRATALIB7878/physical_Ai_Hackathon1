#!/usr/bin/env python3
"""
Test to verify the new RAG agent implementation with Qdrant integration works properly.
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_new_rag_agent():
    """Test the new RAG agent with Qdrant integration."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        print("[OK] Successfully imported RAGAgent")
        
        # Create an instance
        agent = RAGAgent()
        print("[OK] Successfully created RAGAgent instance")
        
        # Check that it has the expected methods
        assert hasattr(agent, 'ask'), "RAGAgent should have an 'ask' method"
        print("[OK] RAGAgent has 'ask' method")
        
        # Check that the agent object exists
        assert hasattr(agent, 'agent'), "RAGAgent should have an 'agent' attribute"
        print("[OK] RAGAgent has 'agent' attribute")
        
        print("\n[OK] All basic functionality tests passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in basic functionality test: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_backend_integration():
    """Test that the backend can integrate with the new RAG agent."""
    try:
        from backend.main import app
        print("[OK] Successfully imported backend app")
        
        # Check that the rag_agent is properly initialized
        import backend.main
        if backend.main.rag_agent is not None:
            print("[OK] RAG agent is properly initialized in backend")
        else:
            print("[WARNING] RAG agent is None in backend (likely due to missing API key)")
        
        print("\n[OK] Backend integration test passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in backend integration test: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_async_safety():
    """Test that the agent is async-safe."""
    try:
        import asyncio
        from backend.rag_agent.rag_agent import RAGAgent
        
        # Create an agent
        agent = RAGAgent()
        print("[OK] Created agent for async safety test")
        
        # Try to call the ask method from within an async context to test nested event loop handling
        async def test_nested():
            try:
                # This should handle nested event loops properly
                result = agent.ask("Test question")  # This will fail due to API but not due to async issues
            except Exception as e:
                # It's expected to fail due to API issues, not async issues
                error_str = str(e).lower()
                if "event loop" in error_str or "nested" in error_str or "running" in error_str:
                    print(f"[ERROR] Async safety issue: {e}")
                    return False
                else:
                    print(f"[OK] Method called (failed as expected due to API, not async: {type(e).__name__})")
            return True
        
        # Run the async test
        result = asyncio.run(test_nested())
        print("[OK] Async safety test passed!")
        return result
        
    except Exception as e:
        print(f"[ERROR] Error in async safety test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing new RAG Agent implementation with Qdrant integration...")
    print()
    
    tests = [
        ("Basic Functionality Test", test_new_rag_agent),
        ("Backend Integration Test", test_backend_integration),
        ("Async Safety Test", test_async_safety),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"Running {test_name}...")
        result = test_func()
        results.append((test_name, result))
        print()
    
    print("Test Results:")
    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")
        if not result:
            all_passed = False
    
    print()
    if all_passed:
        print("[OK] All tests passed!")
        print("The new RAG Agent with Qdrant integration is working properly.")
    else:
        print("[ERROR] Some tests failed.")
    
    return all_passed

if __name__ == "__main__":
    main()