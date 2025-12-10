from openai import OpenAI
from backend.src.core.config import settings
from typing import List

class OpenAIService:
    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.embedding_model = "text-embedding-ada-002"
        self.chat_model = "gpt-3.5-turbo" # Or gpt-4 if preferred

    def get_embedding(self, text: str) -> List[float]:
        """Generates an embedding for the given text using OpenAI."""
        response = self.client.embeddings.create(
            input=text,
            model=self.embedding_model
        )
        return response.data[0].embedding

    def generate_answer(self, question: str, context: str) -> str:
        """
        Generates an answer to the question given the context using OpenAI.
        """
        messages = [
            {"role": "system", "content": "You are a helpful assistant that answers questions based on the provided text context."},
            {"role": "user", "content": f"Context: {context}\nQuestion: {question}\nAnswer:"}
        ]
        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            temperature=0.7,
            max_tokens=500
        )
        return response.choices[0].message.content
