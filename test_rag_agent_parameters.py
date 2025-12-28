#!/usr/bin/env python3
"""
Comprehensive test to verify the RAG agent conversion works properly with parameters.
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_rag_agent_with_parameters():
    """Test that the RAG agent can be called with parameters."""
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
        # This should not raise an exception about missing parameters
        try:
            # This will fail due to missing API key, but should not fail due to method signature issues
            response = agent.ask("What is the capital of France?")
            print(f"[OK] ask() method accepts question parameter: {type(response)}")
        except Exception as e:
            # It's expected to fail due to missing API key, but not due to parameter issues
            if "API key" in str(e) or "Authorization" in str(e) or "api_key" in str(e).lower():
                print(f"[OK] Method signature is correct, failed as expected due to missing API key: {type(e).__name__}")
            else:
                print(f"[ERROR] Unexpected error: {e}")
                return False
        
        print("\n[OK] Parameter handling test passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in parameter handling test: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_simple_rag_agent_with_parameters():
    """Test the simple RAG agent with parameters."""
    try:
        from backend.rag_agent.rag_agent import RAGAgentSimple
        # Only create the agent if API key is available
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("[SKIP] OPENAI_API_KEY not set, skipping simple agent test")
            return True
            
        simple_agent = RAGAgentSimple()
        print("[OK] Successfully created simple RAG agent")
        
        # Try calling the ask method (might fail due to API key, but not due to parameters)
        try:
            response = simple_agent.ask("What is the capital of France?")
            print(f"[OK] Simple agent ask() method works: {type(response)}")
        except Exception as e:
            # It's expected to fail due to missing API key, but not due to parameter issues
            if "API key" in str(e) or "Authorization" in str(e) or "api_key" in str(e).lower():
                print(f"[OK] Method signature is correct, failed as expected due to missing API key: {type(e).__name__}")
            else:
                print(f"[ERROR] Unexpected error: {e}")
                return False
        
        print("\n[OK] Simple agent parameter handling test passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in simple agent parameter test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing RAG Agent parameter handling...")
    print()
    
    tests = [
        ("Parameter Handling Test", test_rag_agent_with_parameters),
        ("Simple Agent Parameter Test", test_simple_rag_agent_with_parameters),
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
        print("[OK] All RAG Agent parameter handling tests passed!")
        print("The conversion to OpenAI Agent SDK appears to be working correctly.")
    else:
        print("[ERROR] Some tests failed.")
    
    return all_passed

if __name__ == "__main__":
    main()