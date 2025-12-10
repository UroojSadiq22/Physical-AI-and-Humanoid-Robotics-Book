import os
import unittest
from unittest.mock import MagicMock, patch

# Adjust path to import the script correctly
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../scripts')))

from ingest_book import get_embedding, load_book_content_placeholder, ingest_book_content, COLLECTION_NAME

class TestIngestBook(unittest.TestCase):

    @patch('ingest_book.openai_client')
    def test_get_embedding(self, mock_openai_client):
        """Test that get_embedding calls OpenAI API and returns embedding."""
        mock_openai_client.embeddings.create.return_value = MagicMock(data=[MagicMock(embedding=[0.1, 0.2, 0.3])])
        text = "test text"
        embedding = get_embedding(text)
        mock_openai_client.embeddings.create.assert_called_once_with(
            input=text,
            model="text-embedding-ada-002"
        )
        self.assertEqual(embedding, [0.1, 0.2, 0.3])

    def test_load_book_content_placeholder(self):
        """Test that load_book_content_placeholder returns expected structure."""
        content = load_book_content_placeholder()
        self.assertIsInstance(content, list)
        self.assertGreater(len(content), 0)
        for item in content:
            self.assertIsInstance(item, tuple)
            self.assertEqual(len(item), 2)
            self.assertIsInstance(item[0], str)
            self.assertIsInstance(item[1], dict)

    @patch('ingest_book.qdrant_client')
    @patch('ingest_book.openai_client')
    @patch('ingest_book.load_book_content_placeholder')
    def test_ingest_book_content(self, mock_load_content, mock_openai_client, mock_qdrant_client):
        """Test that ingest_book_content orchestrates embedding generation and Qdrant upsert."""
        # Mock environment variables
        os.environ['OPENAI_API_KEY'] = 'test_openai_key'
        os.environ['QDRANT_URL'] = 'test_qdrant_url'
        os.environ['QDRANT_API_KEY'] = 'test_qdrant_api_key'

        # Mock content and embeddings
        mock_load_content.return_value = [
            ("chunk 1", {"page": 1}),
            ("chunk 2", {"page": 2})
        ]
        mock_openai_client.embeddings.create.side_effect = [
            MagicMock(data=[MagicMock(embedding=[0.1, 0.2, 0.3])]),
            MagicMock(data=[MagicMock(embedding=[0.4, 0.5, 0.6])])
        ]

        # Call the function
        ingest_book_content()

        # Assertions
        mock_qdrant_client.recreate_collection.assert_called_once_with(
            collection_name=COLLECTION_NAME,
            vectors_config=ingest_book.models.VectorParams(size=1536, distance=ingest_book.models.Distance.COSINE),
        )
        self.assertEqual(mock_openai_client.embeddings.create.call_count, 2)
        mock_qdrant_client.upsert.assert_called_once()
        args, kwargs = mock_qdrant_client.upsert.call_args
        self.assertEqual(kwargs['collection_name'], COLLECTION_NAME)
        self.assertEqual(len(kwargs['points']), 2)
        self.assertEqual(kwargs['points'][0].id, 0)
        self.assertEqual(kwargs['points'][0].vector, [0.1, 0.2, 0.3])
        self.assertEqual(kwargs['points'][0].payload, {"text": "chunk 1", "page": 1})

        # Clean up mock environment variables
        del os.environ['OPENAI_API_KEY']
        del os.environ['QDRANT_URL']
        del os.environ['QDRANT_API_KEY']


if __name__ == '__main__':
    unittest.main()
