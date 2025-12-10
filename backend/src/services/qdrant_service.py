from qdrant_client import QdrantClient, models
from backend.src.core.config import settings
from typing import List, Dict, Any

class QdrantService:
    def __init__(self):
        self.client = QdrantClient(host=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
        self.collection_name = "book_embeddings" # From ingest_book.py

    def search_embeddings(self, query_vector: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Searches the Qdrant collection for similar embeddings.
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            with_payload=True
        )
        return [{"text": hit.payload['text'], "score": hit.score} for hit in search_result]

    def upsert_embeddings(self, points: List[models.PointStruct]):
        """
        Upserts points (embeddings + payload) into the Qdrant collection.
        """
        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )

    def recreate_collection(self, vector_size: int, distance: models.Distance = models.Distance.COSINE):
        """
        Recreates the Qdrant collection with specified vector parameters.
        """
        self.client.recreate_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=distance),
        )
