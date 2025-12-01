import os
from dotenv import load_dotenv
from pathlib import Path
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from agents import AsyncOpenAI
from qdrant_client import AsyncQdrantClient, models
from typing import List, Dict, Optional

# Import components from openai-agents
from agents import Agent, Runner, OpenAIChatCompletionsModel
from agents.run import RunConfig

# Load environment variables (relative to the backend directory)
load_dotenv(Path(__file__).resolve().parent.parent / ".env")

# --- Configuration ---
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("api_key") # Using 'api_key' as per user's instruction
COLLECTION_NAME = "textbook_content"
EMBEDDING_MODEL_NAME = "text-embedding-004" # Or other suitable Gemini embedding model for embeddings
GENERATION_MODEL_NAME = "gemini-2.0-flash"

# --- Initialize Clients ---
if not GEMINI_API_KEY:
    raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="GEMINI_API_KEY is not set.")

gemini_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)


if not QDRANT_HOST:
    raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="QDRANT_HOST is not set.")

qdrant_client = AsyncQdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

# --- Router Setup ---
router = APIRouter(
    prefix="/chat",
    tags=["chat"],
)

# --- Pydantic Models ---
class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    query: str
    history: List[ChatMessage] = [] # For conversation history
    top_k: int = 5

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []

# --- Helper Function for Embeddings (duplicated from ingest.py for now) ---
async def get_query_embedding(text: str) -> List[float]:
    """Generates embedding for a single text using the Gemini embedding model."""
    try:
        response = await gemini_client.embeddings.create(
            model=EMBEDDING_MODEL_NAME,
            input=[text]
        )
        return response.data[0].embedding
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error generating query embedding: {e}"
        )

# --- RAG Endpoint ---
@router.post("/", response_model=ChatResponse)
async def chat_with_rag(request: ChatRequest):
    """
    Performs RAG to answer the user's query using retrieved context and Gemini.
    """
    query_embedding = await get_query_embedding(request.query)

    search_result = await qdrant_client.query_points(
        collection_name=COLLECTION_NAME,
        query=query_embedding, # Note: `query_vector` becomes `query`
        limit=request.top_k,
        query_filter=None,
        with_payload=True, # Ensure payload is returned
    )

    context_snippets = []
    sources = set()
    for hit in search_result.points:
        context_snippets.append(hit.payload["content"])
        sources.add(hit.payload["source"])
    
    context_str = "\n\n".join(context_snippets)

    # Prepare messages for the agent, including history and context
    messages_for_agent = []
    # Add history
    for msg in request.history:
        messages_for_agent.append({"role": msg.role, "content": msg.content})

    # Add system instruction with context
    system_instruction = (
        "You are a helpful assistant providing information based on the provided context. "
        "Answer the user's question accurately and concisely. "
        "If the answer is not in the context, state that you don't know. "
        f"Context:\n\n{context_str}"
    )
    messages_for_agent.append({"role": "system", "content": system_instruction})

    # Add current user query
    messages_for_agent.append({"role": "user", "content": request.query})

    try:
        # Initialize the OpenAI Agents model, configured for Gemini
        model = OpenAIChatCompletionsModel(
            model=GENERATION_MODEL_NAME,
            openai_client=gemini_client
        )

        # Initialize the Agent
        agent: Agent = Agent(
            name="RAG Assistant",
            instructions="You are an expert RAG assistant. "
                         "You only use the provided context to answer questions. "
                         "If the information is not in the context, you state 'I cannot answer based on the provided context.'"
                         "if the user greeting so say user please ask me question only related this book",
            model=model
        )

        # Configure the runner to use our Gemini client
        run_config = RunConfig(
            model=model, # Specify model for generation
            model_provider=gemini_client, # Crucially, pass the AsyncOpenAI client for Gemini
            tracing_disabled=True # Disable tracing for now
        )

        # Run the agent
        # The 'messages' parameter for agent.run expects a list of dicts: [{"role": "user", "content": "..."}]
        # The history and context are handled by the system instruction.
        result = await Runner.run(agent, messages_for_agent, run_config=run_config)
        
        return ChatResponse(answer=result.final_output, sources=list(sources))

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error generating RAG response: {e}"
        )
