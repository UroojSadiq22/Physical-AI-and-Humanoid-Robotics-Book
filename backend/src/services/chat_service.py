from sqlalchemy.orm import Session
from backend.src.models import models
from backend.src.models import chat as chat_schemas
from backend.src.services.security import get_password_hash
from uuid import UUID

class ChatService:
    def __init__(self, db: Session):
        self.db = db

    def get_user_by_email(self, email: str):
        return self.db.query(models.User).filter(models.User.email == email).first()

    def create_user(self, user: chat_schemas.User):
        hashed_password = get_password_hash(user.hashed_password)
        db_user = models.User(email=user.email, hashed_password=hashed_password)
        self.db.add(db_user)
        self.db.commit()
        self.db.refresh(db_user)
        return db_user

    def create_chat_session(self, user_id: UUID):
        db_session = models.ChatSession(user_id=user_id)
        self.db.add(db_session)
        self.db.commit()
        self.db.refresh(db_session)
        return db_session

    def create_chat_message(self, session_id: UUID, author: str, content: str, context_text: str = None):
        db_message = models.ChatMessage(
            session_id=session_id,
            author=author,
            content=content,
            context_text=context_text
        )
        self.db.add(db_message)
        self.db.commit()
        self.db.refresh(db_message)
        return db_message

    def get_chat_session(self, session_id: UUID):
        return self.db.query(models.ChatSession).filter(models.ChatSession.id == session_id).first()

    def get_chat_messages(self, session_id: UUID):
        return self.db.query(models.ChatMessage).filter(models.ChatMessage.session_id == session_id).order_by(models.ChatMessage.created_at).all()
