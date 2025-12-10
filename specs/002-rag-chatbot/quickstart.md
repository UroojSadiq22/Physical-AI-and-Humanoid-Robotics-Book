# Quickstart: RAG Chatbot

**Date**: 2025-12-10

This guide provides the essential steps to set up and run the RAG Chatbot feature locally.

## Prerequisites

- Python 3.11+ and `pip`
- Node.js 18+ and `npm`
- Docker and Docker Compose (for running local instances of databases if not using cloud services)
- Access credentials for:
    - Neon Postgres
    - Qdrant Cloud
    - OpenAI API

## Backend Setup

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Create a virtual environment and install dependencies**:
    ```bash
    python -m venv .venv
    source .venv/bin/activate  # On Windows use `.venv\Scripts\activate`
    pip install -r requirements.txt
    ```
    *(Note: `requirements.txt` will be created in a later task).*

3.  **Configure environment variables**:
    Create a `.env` file in the `backend` directory and add the following:
    ```env
    DATABASE_URL="your_neon_postgres_connection_string"
    QDRANT_URL="your_qdrant_cloud_url"
    QDRANT_API_KEY="your_qdrant_api_key"
    OPENAI_API_KEY="your_openai_api_key"
    JWT_SECRET_KEY="a_very_secret_key"
    ```

4.  **Run database migrations**:
    *(This will require a migration tool like Alembic, to be added).*
    ```bash
    alembic upgrade head
    ```

5.  **Run the ingestion script**:
    A script will be created to embed the book content into the Qdrant vector store.
    ```bash
    python -m scripts.ingest_book
    ```

6.  **Start the FastAPI server**:
    ```bash
    uvicorn src.api.main:app --reload
    ```
    The backend will be running at `http://127.0.0.1:8000`.

## Frontend Setup

The frontend is part of the Docusaurus application.

1.  **Navigate to the docs directory**:
    ```bash
    cd docs
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```

3.  **Start the Docusaurus development server**:
    ```bash
    npm run start
    ```
    The Docusaurus site will be running at `http://localhost:3000`. The chatbot component will be visible on the pages where it is embedded.

## Running the Full Application

- Ensure both the backend FastAPI server and the frontend Docusaurus server are running.
- Open a web browser and navigate to `http://localhost:3000`.
- Log in to the application (the login mechanism will be part of the implementation).
- The chatbot floating action button should be visible. Click it to open the chat interface and start asking questions.
