import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.src.api.main import app, get_db, oauth2_scheme # Import oauth2_scheme
from backend.src.core.database import Base
from backend.src.models import models
from backend.src.services.security import create_access_token, get_password_hash
from unittest.mock import patch, MagicMock
import os
from uuid import UUID

# Setup for testing database
SQLALCHEMY_DATABASE_URL = "sqlite:///./test.db" # Use SQLite for testing

engine = create_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create tables before tests run
@pytest.fixture(name="db")
def db_fixture():
    Base.metadata.create_all(bind=engine)
    db = TestingSessionLocal()
    try:
        yield db
    finally:
        db.close()
        Base.metadata.drop_all(bind=engine) # Clean up after tests

# Override get_db dependency
app.dependency_overrides[get_db] = db_fixture

# Mock authentication for testing
@pytest.fixture(name="mock_oauth2_scheme")
def mock_oauth2_scheme_fixture():
    with patch('backend.src.api.main.oauth2_scheme', MagicMock()) as mock:
        yield mock

# Test client
client = TestClient(app)

@pytest.fixture(name="test_user")
def test_user_fixture(db: Session):
    # Create a test user
    hashed_password = get_password_hash("testpassword")
    user = models.User(email="test@example.com", hashed_password=hashed_password)
    db.add(user)
    db.commit()
    db.refresh(user)
    return user

@pytest.fixture(name="authorized_client")
def authorized_client_fixture(test_user: models.User):
    access_token = create_access_token(data={"sub": str(test_user.id)})
    headers = {"Authorization": f"Bearer {access_token}"}
    return TestClient(app, headers=headers)


@patch('backend.src.services.qdrant_service.QdrantService.search_embeddings')
@patch('backend.src.services.openai_service.OpenAIService.get_embedding')
@patch('backend.src.services.openai_service.OpenAIService.generate_answer')
def test_query_chatbot_general_question(
    mock_generate_answer, mock_get_embedding, mock_search_embeddings, 
    authorized_client: TestClient, test_user: models.User, db: Session
):
    # Mock external service calls
    mock_get_embedding.return_value = [0.1, 0.2, 0.3]
    mock_search_embeddings.return_value = [{"text": "relevant context 1", "score": 0.9}]
    mock_generate_answer.return_value = "This is a generated answer."

    # Make the request
    response = authorized_client.post(
        "/api/query",
        json={"question": "What is AI?"}
    )

    # Assertions
    assert response.status_code == 200
    assert response.json() == {"answer": "This is a generated answer."}

    # Verify service calls
    mock_get_embedding.assert_called_once_with("What is AI?")
    mock_search_embeddings.assert_called_once() # Args checked in more detail later if needed
    mock_generate_answer.assert_called_once_with("What is AI?", "relevant context 1")

    # Verify chat messages are logged
    messages = db.query(models.ChatMessage).filter(models.ChatMessage.session_id.isnot(None)).all()
    assert len(messages) == 2
    assert messages[0].author == "user"
    assert messages[0].content == "What is AI?"
    assert messages[1].author == "bot"
    assert messages[1].content == "This is a generated answer."

    # Verify chat session is created
    sessions = db.query(models.ChatSession).filter(models.ChatSession.user_id == test_user.id).all()
    assert len(sessions) == 1


@patch('backend.src.services.qdrant_service.QdrantService.search_embeddings')
@patch('backend.src.services.openai_service.OpenAIService.get_embedding')
@patch('backend.src.services.openai_service.OpenAIService.generate_answer')
def test_query_chatbot_selected_text_question(
    mock_generate_answer, mock_get_embedding, mock_search_embeddings,
    authorized_client: TestClient, test_user: models.User, db: Session
):
    # Mock external service calls
    mock_get_embedding.return_value = [0.4, 0.5, 0.6]
    mock_search_embeddings.return_value = [{"text": "selected text context", "score": 0.95}]
    mock_generate_answer.return_value = "Answer based on selected text."

    # Make the request with selected_text
    response = authorized_client.post(
        "/api/query",
        json={"question": "Explain this", "selected_text": "A specific paragraph about AI."}
    )

    # Assertions
    assert response.status_code == 200
    assert response.json() == {"answer": "Answer based on selected text."}

    # Verify service calls
    mock_get_embedding.assert_called_once_with("Explain this (Context: A specific paragraph about AI.)")
    mock_search_embeddings.assert_called_once() # Args checked in more detail later if needed
    mock_generate_answer.assert_called_once_with("Explain this", "selected text context")

    # Verify chat messages are logged
    messages = db.query(models.ChatMessage).filter(models.ChatMessage.session_id.isnot(None)).all()
    assert len(messages) == 2
    assert messages[0].author == "user"
    assert messages[0].content == "Explain this"
    assert messages[0].context_text == "A specific paragraph about AI."
    assert messages[1].author == "bot"
    assert messages[1].content == "Answer based on selected text."
