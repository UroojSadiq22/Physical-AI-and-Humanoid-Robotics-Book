from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from sqlalchemy.orm import Session
from datetime import timedelta
from typing import Annotated
from uuid import UUID

from backend.src.core.config import settings
from backend.src.core.database import SessionLocal, engine, get_db, Base
from backend.src.models import models, chat
from backend.src.services import security, qdrant_service, openai_service, chat_service

# Create database tables
Base.metadata.create_all(bind=engine)

app = FastAPI()

# OAuth2PasswordBearer for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Initialize services
qdrant = qdrant_service.QdrantService()
openai = openai_service.OpenAIService()

# Dependency to get current user
async def get_current_user(token: Annotated[str, Depends(oauth2_scheme)], db: Annotated[Session, Depends(get_db)]):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    payload = security.decode_access_token(token)
    if payload is None:
        raise credentials_exception
    user_id = payload.get("sub")
    if user_id is None:
        raise credentials_exception
    
    # Ensure user_id is a valid UUID string before converting
    try:
        user_uuid = UUID(user_id)
    except ValueError:
        raise credentials_exception

    user = db.query(models.User).filter(models.User.id == user_uuid).first()
    if user is None:
        raise credentials_exception
    return user

@app.post("/token", response_model=chat.Token)
async def login_for_access_token(form_data: Annotated[OAuth2PasswordRequestForm, Depends()], db: Annotated[Session, Depends(get_db)]):
    user = db.query(models.User).filter(models.User.email == form_data.username).first()
    if not user or not security.verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = security.create_access_token(
        data={"sub": str(user.id)}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}


@app.post("/register", response_model=chat.User)
async def register_user(user_data: chat.User, db: Annotated[Session, Depends(get_db)]):
    crud = chat_service.ChatService(db)
    db_user = crud.get_user_by_email(user_data.email)
    if db_user:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Email already registered")
    
    new_user = crud.create_user(user_data)
    return new_user

@app.get("/users/me", response_model=chat.User)
async def read_users_me(current_user: Annotated[models.User, Depends(get_current_user)]):
    return current_user

@app.post("/api/query", response_model=chat.QueryResponse)
async def query_chatbot(
    query_request: chat.QueryRequest, 
    current_user: Annotated[models.User, Depends(get_current_user)], 
    db: Annotated[Session, Depends(get_db)]
):
    crud = chat_service.ChatService(db)

    # Create a new chat session or use an existing one (for simplicity, always create new for now)
    # In a real app, you might have session_id passed from frontend
    chat_session = crud.create_chat_session(user_id=current_user.id)
    session_id = chat_session.id

    # Get embedding for the query (and selected text if provided)
    query_text = query_request.question
    if query_request.selected_text:
        query_text = f"{query_text} (Context: {query_request.selected_text})"
    
    query_vector = openai.get_embedding(query_text)

    # Search Qdrant for relevant context
    context_chunks = qdrant.search_embeddings(query_vector)
    
    # Combine context for OpenAI
    combined_context = "\n".join([chunk["text"] for chunk in context_chunks])
    
    # Generate answer using OpenAI
    answer = openai.generate_answer(query_request.question, combined_context)
    
    # Log user message
    crud.create_chat_message(
        session_id=session_id, 
        author="user", 
        content=query_request.question, 
        context_text=query_request.selected_text
    )
    # Log bot response
    crud.create_chat_message(
        session_id=session_id, 
        author="bot", 
        content=answer
    )
    
    return chat.QueryResponse(answer=answer)