import os
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient, models

# Load environment variables
load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(host=QDRANT_URL, api_key=QDRANT_API_KEY)

COLLECTION_NAME = "book_embeddings"
BATCH_SIZE = 100

def get_embedding(text: str) -> list[float]:
    """Generates an embedding for the given text using OpenAI."""
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-ada-002"
    )
    return response.data[0].embedding

def load_book_content_placeholder():
    """
    Placeholder function to load book content.
    In a real scenario, this would read from markdown files, database, etc.
    Returns a list of (text_chunk, metadata) tuples.
    """
    print("Loading placeholder book content...")
    # Example content
    return [
        ("This is the first chapter of the book about AI.", {"chapter": 1, "page": 1}),
        ("Artificial intelligence is a rapidly evolving field.", {"chapter": 1, "page": 2}),
        ("Robotics is a branch of engineering and science that deals with the design, construction, operation, and use of robots.", {"chapter": 2, "page": 1}),
        ("Machine learning is a subset of AI that focuses on enabling systems to learn from data.", {"chapter": 1, "page": 3}),
        ("Humanoid robots are designed to resemble the human body.", {"chapter": 2, "page": 2})
    ]

def ingest_book_content():
    """
    Ingests book content into Qdrant after generating embeddings.
    """
    print(f"Ensuring collection '{COLLECTION_NAME}' exists...")
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE), # OpenAI embedding size
    )

    content_chunks = load_book_content_placeholder()
    points = []
    for i, (text_chunk, metadata) in enumerate(content_chunks):
        print(f"Generating embedding for chunk {i+1}...")
        embedding = get_embedding(text_chunk)
        points.append(
            models.PointStruct(
                id=i, # Simple ID for placeholder, use UUIDs in production
                vector=embedding,
                payload={"text": text_chunk, **metadata}
            )
        )

        if len(points) >= BATCH_SIZE:
            print(f"Uploading a batch of {len(points)} points...")
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=points
            )
            points = []

    if points:
        print(f"Uploading remaining {len(points)} points...")
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=points
        )
    print(f"Ingestion complete. Total chunks ingested: {len(content_chunks)}")

if __name__ == "__main__":
    if not all([OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY]):
        print("Error: Missing one or more environment variables. Please check your .env file.")
        exit(1)
    ingest_book_content()
