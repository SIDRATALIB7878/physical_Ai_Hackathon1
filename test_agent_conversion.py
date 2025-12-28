#!/usr/bin/env python3
"""
Test script to verify the OpenAI Agent SDK conversion works properly.
"""
import os
import sys

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

def test_agent_import():
    """Test that we can import the converted RAG agent."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        print("[OK] Successfully imported RAGAgent from converted code")
        return True
    except ImportError as e:
        print(f"[ERROR] Failed to import RAGAgent: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Error importing RAGAgent: {e}")
        return False

def test_agent_creation():
    """Test that we can create an instance of the RAG agent."""
    try:
        from backend.rag_agent.rag_agent import RAGAgent
        agent = RAGAgent(model="gpt-4-1106-preview")
        print("[OK] Successfully created RAGAgent instance")
        return True
    except Exception as e:
        print(f"[ERROR] Error creating RAGAgent instance: {e}")
        return False

def test_syntax():
    """Test that the file has valid Python syntax."""
    try:
        import ast
        with open("backend/rag_agent/rag_agent.py", "r", encoding="utf-8") as f:
            source = f.read()
        ast.parse(source)
        print("[OK] File has valid Python syntax")
        return True
    except SyntaxError as e:
        print(f"[ERROR] Syntax error in file: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Error checking syntax: {e}")
        return False

def main():
    print("Testing OpenAI Agent SDK conversion...")
    print()
    
    tests = [
        ("Syntax Check", test_syntax),
        ("Import Test", test_agent_import),
        ("Creation Test", test_agent_creation),
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
        print("[OK] All tests passed! The conversion appears to be successful.")
    else:
        print("[ERROR] Some tests failed. Please review the output above.")

    return all_passed

if __name__ == "__main__":
    main()