# Research & Decisions: RAG Chatbot

**Status**: Completed | **Date**: 2025-12-10

This document records the key architectural and technology decisions for the RAG Chatbot feature.

## Decision 1: Frontend Framework

- **Decision**: Use **React** to build the chatbot component.
- **Rationale**: The existing Docusaurus site is built on React, making it the natural choice for seamless integration. This allows us to create a "swizzled" or custom Docusaurus component that can be easily embedded into MDX pages and can leverage the existing theme and styling capabilities.
- **Alternatives Considered**:
    - **Vanilla JavaScript**: Would be more difficult to manage state and create a component-based architecture.
    - **Other Frameworks (Vue, Svelte)**: Would require integrating a different frontend framework into the existing React application, adding unnecessary complexity.

## Decision 2: Backend Framework

- **Decision**: Use **FastAPI** (Python) for the backend service.
- **Rationale**: FastAPI is a modern, high-performance Python web framework that is perfect for building APIs. Its automatic generation of OpenAPI documentation and use of Pydantic for data validation will accelerate development. Python has a rich ecosystem of libraries for machine learning and interacting with services like Qdrant and OpenAI.
- **Alternatives Considered**:
    - **Node.js (Express/NestJS)**: A valid choice, but the ecosystem for AI/ML tasks is more mature in Python.
    - **Flask**: A lighter-weight Python framework, but FastAPI's performance and built-in async support and data validation make it a better choice for this API.

## Decision 3: Vector Store

- **Decision**: Use **Qdrant** for the vector store.
- **Rationale**: Qdrant is a high-performance vector similarity search engine. It is well-suited for RAG applications and has a straightforward API for uploading and querying vector embeddings.
- **Alternatives Considered**:
    - **FAISS**: A powerful library, but requires more manual setup and management.
    - **Other managed vector DBs (e.g., Pinecone)**: Qdrant offers a great open-source and self-hostable option that is suitable for this project's scale.

## Decision 4: Session & Log Storage

- **Decision**: Use **Neon Postgres** for storing chat sessions and message logs.
- **Rationale**: Neon is a serverless Postgres provider that is easy to set up and manage. A standard relational database like Postgres is ideal for structured data like chat logs, allowing for easy querying and analysis.
- **Alternatives Considered**:
    - **SQLite**: Too simplistic for a web application that may need to scale.
    - **NoSQL Databases (e.g., MongoDB)**: While viable, the relational structure of chat sessions and messages fits well with a SQL model.

## Decision 5: Communication Protocol

- **Decision**: Use **stateless REST (HTTP POST)** for client-server communication.
- **Rationale**: For an MVP developed under hackathon constraints, a simple, stateless REST API is the quickest and simplest approach. It requires no persistent connection management on the backend.
- **Alternatives Considered**:
    - **WebSockets**: Would enable a more real-time, stateful connection, which could be beneficial for streaming responses back to the user. However, it adds complexity to both the frontend and backend, which is not ideal for an MVP. This can be considered a future enhancement.

## Decision 6: Language Model Provider

- **Decision**: Use the **OpenAI API** for the generation step.
- **Rationale**: OpenAI models are powerful and provide high-quality text generation, which is essential for the chatbot's success. Their API is well-documented and easy to integrate.
- **Alternatives Considered**:
    - **Open Source Models (e.g., Llama, Mistral)**: Would require more infrastructure to host and manage. Using a managed API is more efficient for this project.
