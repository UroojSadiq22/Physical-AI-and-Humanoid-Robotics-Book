from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field

class User(BaseModel):
    id: Optional[UUID] = Field(default_factory=uuid4)
    email: str
    hashed_password: str
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = Field(default_factory=datetime.utcnow)

    class Config:
        from_attributes = True # Allow ORM models to be converted to Pydantic models

class UserInDB(User):
    hashed_password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None

class ChatSession(BaseModel):
    id: UUID = Field(default_factory=uuid4)
    user_id: UUID
    created_at: datetime = Field(default_factory=datetime.utcnow)

class ChatMessage(BaseModel):
    id: UUID = Field(default_factory=uuid4)
    session_id: UUID
    author: str # 'user' or 'bot'
    content: str
    context_text: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str