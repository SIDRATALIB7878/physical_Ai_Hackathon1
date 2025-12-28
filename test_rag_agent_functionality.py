#!/usr/bin/env python3
"""
Simple test to verify the RAG agent conversion works properly.
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_rag_agent_creation_and_basic_functionality():
    """Test that the RAG agent can be created and has the expected interface."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        print("[OK] Successfully imported RAGAgent")
        
        # Create an instance
        agent = RAGAgent(model="gpt-4-1106-preview")
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

def test_simple_rag_agent():
    """Test the simple RAG agent functionality."""
    try:
        from backend.rag_agent.rag_agent import RAGAgentSimple
        simple_agent = RAGAgentSimple()
        print("[OK] Successfully created simple RAG agent")
        
        print("\n[OK] Simple agent creation test passed!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Error in simple agent test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing RAG Agent conversion functionality...")
    print()
    
    tests = [
        ("Basic Functionality Test", test_rag_agent_creation_and_basic_functionality),
        ("Simple Agent Test", test_simple_rag_agent),
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
        print("[OK] All RAG Agent functionality tests passed!")
        print("The conversion to OpenAI Agent SDK appears to be successful.")
    else:
        print("[ERROR] Some tests failed.")
    
    return all_passed

if __name__ == "__main__":
    main()