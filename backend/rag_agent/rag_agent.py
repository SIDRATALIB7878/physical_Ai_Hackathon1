"""
RAG Agent using OpenAI Agents SDK + Qdrant
Answers ONLY from retrieved book content
"""

import os
import asyncio
from typing_extensions import Annotated
from pydantic import BaseModel, Field
from dotenv import load_dotenv

from agents import Agent, Runner, function_tool
from qdrant_client import QdrantClient
from openai import OpenAI

# Load env vars
load_dotenv()

# -------------------------------
# ENV CHECK
# -------------------------------
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION")

if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable is not set")

# For development, provide defaults if QDRANT env vars are not set
if not QDRANT_URL:
    QDRANT_URL = "http://localhost:6333"  # Default Qdrant URL

if not QDRANT_COLLECTION:
    QDRANT_COLLECTION = "book_embeddings_default"  # Default collection name

# -------------------------------
# CLIENTS
# -------------------------------
openai_client = OpenAI(api_key=OPENAI_API_KEY)

qdrant = QdrantClient(url=QDRANT_URL)

# -------------------------------
# MODELS
# -------------------------------
class RetrievedContent(BaseModel):
    content: str = Field(description="Retrieved book text")
    source: str = Field(description="Source info")

# -------------------------------
# RETRIEVAL TOOL
# -------------------------------
@function_tool
def retrieve_book_content_tool(
    query: Annotated[str, "User question"],
    limit: Annotated[int, "Number of chunks"] = 5
) -> RetrievedContent:
    """
    Searches Qdrant and returns book chunks
    """

    # Create embedding
    embedding = openai_client.embeddings.create(
        model="text-embedding-3-small",
        input=query
    ).data[0].embedding

    # Search Qdrant
    results = qdrant.search(
        collection_name=QDRANT_COLLECTION,
        query_vector=embedding,
        limit=limit
    )

    if not results:
        return RetrievedContent(
            content="No relevant content found in the book.",
            source="Qdrant"
        )

    text = "\n\n".join(
        hit.payload.get("text", "")
        for hit in results
    )

    return RetrievedContent(
        content=text,
        source="Book / Qdrant"
    )

# -------------------------------
# AGENT
# -------------------------------
class RAGAgent:
    def __init__(self):
        self.agent = Agent(
            name="Book RAG Agent",
            model="gpt-4.1-mini",
            instructions="""
You are a Retrieval-Augmented Generation agent.

RULES:
1. You MUST call retrieve_book_content_tool before answering.
2. You may ONLY answer using retrieved content.
3. If no content is retrieved, say: "No relevant information found in the book."
4. Do NOT use prior knowledge.
""",
            tools=[retrieve_book_content_tool],
        )

    def ask(self, question: str) -> str:
        """
        Ask a question to the RAG agent.
        Ensures the retrieval tool is called before generating a response.
        """
        # First, manually retrieve content using the Qdrant client directly
        retrieved_content = self._retrieve_content_sync(question)

        # Format the question with the retrieved context
        if retrieved_content and "[API_LIMIT_REACHED]" in retrieved_content:
            # If API limits were reached, return a message indicating this
            return "I'm sorry, but I couldn't retrieve relevant book content because the API quota/rate limit was reached. Please try again later."
        elif retrieved_content and "No relevant content found" not in retrieved_content:
            context_question = f"Based on this content: {retrieved_content}\n\nAnswer this question: {question}"
        else:
            context_question = question  # Just ask the original question if no content found

        async def run():
            try:
                result = await Runner.run(self.agent, context_question)
                return result.final_output
            except Exception as e:
                # Check if it's an API quota/rate limit error during agent run
                error_str = str(e).lower()
                if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
                    return "I'm sorry, but I'm currently experiencing API quota/rate limit issues. Please try again later."
                else:
                    # Re-raise other exceptions
                    raise e

        # Check if there's already a running event loop
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            # No event loop running, safe to use asyncio.run
            return asyncio.run(run())
        else:
            # Event loop is running, use nested event loop approach
            import concurrent.futures

            # Create a new thread to run the asyncio operation
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(lambda: asyncio.run(run()))
                return future.result()

    def _retrieve_content_sync(self, query: str) -> str:
        """Helper method to retrieve content synchronously."""
        # This calls the same logic as the function tool but directly
        try:
            # Create embedding
            embedding = openai_client.embeddings.create(
                model="text-embedding-3-small",
                input=query
            ).data[0].embedding

            # Search Qdrant
            results = qdrant.search(
                collection_name=QDRANT_COLLECTION,
                query_vector=embedding,
                limit=5
            )

            if not results:
                return "No relevant content found in the book."

            text = "\n\n".join(
                hit.payload.get("text", "")
                for hit in results
            )

            return text
        except Exception as e:
            # Check if it's an API quota/rate limit error
            error_str = str(e).lower()
            if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
                print(f"API limit reached: {e}")
                # Return a special indicator that we know API limits were hit
                return "[API_LIMIT_REACHED]"
            else:
                print(f"Error retrieving content: {e}")
                return "No relevant content found in the book."

# -------------------------------
# SIMPLE AGENT (for testing without OpenAI Agents SDK)
# -------------------------------
class RAGAgentSimple:
    def __init__(self, model="gpt-3.5-turbo"):
        self.model = model
        # Note: This simplified version doesn't use the OpenAI Agents SDK
        # It's designed for testing and simpler implementations

    def ask(self, question: str, book_id: str = "default_book", limit: int = 5) -> str:
        """
        Ask a question to the simple RAG agent.
        This version uses the mock_retrieval_tool by default for testing purposes.
        """
        try:
            # Try to import and use the mock retrieval tool
            from .mock_retrieval_tool import retrieve_book_content
        except ImportError:
            try:
                # If relative import fails, try absolute import
                from backend.rag_agent.mock_retrieval_tool import retrieve_book_content
            except ImportError:
                try:
                    # If both imports fail, try direct import
                    import mock_retrieval_tool
                    retrieve_book_content = mock_retrieval_tool.retrieve_book_content
                except ImportError:
                    # If all imports fail, return an error message
                    return "Error: Could not import mock retrieval tool. Please ensure mock_retrieval_tool.py exists."

        # Retrieve relevant content based on the question
        retrieved_content = retrieve_book_content(question, book_id, limit)

        # Process the retrieved content and generate a response
        # In a real implementation, you would send this to an LLM
        try:
            import json
            results = json.loads(retrieved_content)

            if not results:
                return "No relevant information found in the book."

            # Format the response based on the retrieved content
            response_parts = ["Based on the book content:"]
            for result in results:
                content = result.get("content", "")
                if content:
                    response_parts.append(f"- {content}")

            response_parts.append(f"\nQuestion: {question}")
            return "\n\n".join(response_parts)
        except json.JSONDecodeError:
            # If the retrieved content is not valid JSON, return it as is
            return f"Retrieved content: {retrieved_content}\n\nQuestion: {question}"


# -------------------------------
# TEST
# -------------------------------
if __name__ == "__main__":
    agent = RAGAgent()
    answer = agent.ask("What is the Perception-Action Cycle of Physical AI?")
    print(answer)
