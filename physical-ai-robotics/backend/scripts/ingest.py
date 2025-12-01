import os
import asyncio
from dotenv import load_dotenv
from pathlib import Path
from openai import AsyncOpenAI
from qdrant_client import QdrantClient, models
from typing import List, Dict
import uuid

# Load environment variables from .env file
load_dotenv(Path(__file__).resolve().parent.parent / ".env")

# --- Configuration ---
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("api_key") # Using 'api_key' as per user's instruction
COLLECTION_NAME = "textbook_content"
EMBEDDING_MODEL_NAME = "text-embedding-004" # Or other suitable Gemini embedding model
DOCS_PATH = Path(__file__).resolve().parent.parent.parent / "docs" # Path to physical-ai-robotics/docs

# --- Initialize Clients ---
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY (or api_key) is not set. Please ensure it is defined in your .env file.")

# Initialize OpenAI-compatible client for Gemini
gemini_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Initialize Qdrant Client
if not QDRANT_HOST:
    raise ValueError("QDRANT_HOST is not set. Please define it in your .env file.")

qdrant_client = QdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY, # Optional, if Qdrant is publicly exposed
)

# --- Helper Functions ---
def read_markdown_files(directory: Path) -> List[Dict]:
    """Reads all markdown files from the specified directory and returns their content."""
    documents = []
    for md_file in directory.rglob("*.md"):
        if md_file.is_file():
            try:
                content = md_file.read_text(encoding="utf-8")
                documents.append({"id": str(md_file), "content": content, "source": str(md_file.relative_to(DOCS_PATH))})
            except Exception as e:
                print(f"Error reading {md_file}: {e}")
    return documents

def chunk_text(document: Dict, chunk_size: int = 500, chunk_overlap: int = 50) -> List[Dict]:
    """
    Splits document content into smaller chunks.
    This is a simple character-based splitter.
    A more advanced splitter might consider Markdown structure.
    """
    text = document["content"]
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append({
            "content": chunk,
            "source": document["source"],
            "start_char": start,
            "end_char": end,
            "original_id": document["id"]
        })
        start += chunk_size - chunk_overlap
        if chunk_size - chunk_overlap <= 0 and start < len(text): # Avoid infinite loop if overlap >= chunk_size
             start += chunk_size # Move by full chunk size if overlap is too large

    return chunks

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """Generates embeddings for a list of texts using the Gemini embedding model."""
    try:
        response = await gemini_client.embeddings.create(
            model=EMBEDDING_MODEL_NAME,
            input=texts
        )
        return [embedding.embedding for embedding in response.data]
    except Exception as e:
        print(f"Error generating embeddings: {e}")
        return []

async def upload_to_qdrant(chunks: List[Dict]):
    """Uploads text chunks and their embeddings to Qdrant."""
    print(f"Ensuring collection '{COLLECTION_NAME}' exists...")
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE), # Gemini's text-embedding-004 is 768 dims
    )

    print(f"Generating embeddings for {len(chunks)} chunks and uploading to Qdrant...")
    batch_size = 100 # Adjust based on API limits and performance
    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i:i+batch_size]
        texts_to_embed = [chunk["content"] for chunk in batch_chunks]
        embeddings = await get_embeddings(texts_to_embed)

        if not embeddings:
            print(f"Skipping batch {i//batch_size} due to embedding failure.")
            continue

        points = []
        for j, chunk in enumerate(batch_chunks):
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()), # Generate a UUID for the Qdrant point ID
                    vector=embeddings[j],
                    payload={
                        "content": chunk["content"],
                        "source": chunk["source"],
                        "start_char": chunk["start_char"],
                        "end_char": chunk["end_char"]
                    },
                )
            )
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True,
        )
        print(f"Uploaded {len(points)} points to Qdrant (Batch {i//batch_size})")

async def main():
    print(f"Starting ingestion process from {DOCS_PATH}...")
    documents = read_markdown_files(DOCS_PATH)
    all_chunks = []
    for doc in documents:
        all_chunks.extend(chunk_text(doc))
    
    print(f"Total {len(documents)} documents read, resulting in {len(all_chunks)} chunks.")

    if all_chunks:
        await upload_to_qdrant(all_chunks)
        print("Ingestion complete!")
    else:
        print("No chunks to upload.")

if __name__ == "__main__":
    asyncio.run(main())
