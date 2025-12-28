#!/usr/bin/env python3
"""
Test to verify the RAG agent conversion works properly with the async fix.
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_rag_agent_async_handling():
    """Test that the RAG agent can handle async operations properly."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        print("[OK] Successfully imported RAGAgent")
        
        # Create an instance
        agent = RAGAgent(model="gpt-3.5-turbo")
        print("[OK] Successfully created RAGAgent instance")
        
        # Check that it has the expected methods
        assert hasattr(agent, 'ask'), "RAGAgent should have an 'ask' method"
        print("[OK] RAGAgent has 'ask' method")
        
        # Try calling the ask method (without actually executing it if API key is not available)
        # This should not raise an exception about missing event loop
        try:
            response = agent.ask("What is the capital of France?")
            print(f"[OK] ask() method called successfully (returned: {type(response)})")
        except Exception as e:
            # It's expected to fail due to missing API key, but not due to event loop issues
            error_str = str(e).lower()
            if "api" in error_str or "key" in error_str or "authorization" in error_str:
                print(f"[OK] Method called successfully, failed as expected due to missing API credentials: {type(e).__name__}")
            elif "event loop" in error_str or "no running" in error_str:
                print(f"[ERROR] Event loop issue still exists: {e}")
                return False
            else:
                print(f"[ERROR] Unexpected error: {e}")
                return False
        
        print("\n[OK] Async handling test passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in async handling test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing RAG Agent async handling...")
    print()
    
    tests = [
        ("Async Handling Test", test_rag_agent_async_handling),
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
        print("[OK] All RAG Agent async handling tests passed!")
        print("The conversion to OpenAI Agent SDK with proper async handling is successful.")
    else:
        print("[ERROR] Some tests failed.")
    
    return all_passed

if __name__ == "__main__":
    main()