# Actionable Tasks: RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-10 | **Plan**: [plan.md](plan.md)

This document breaks down the implementation of the RAG Chatbot feature into a series of actionable, dependency-ordered tasks.

## Phase 1: Project Setup

This phase focuses on initializing the project structure and environment.

- [X] T001 Create backend directory structure: `backend/src/{api,core,services,models}` and `backend/tests/{integration,unit}`
- [X] T002 Initialize Python project in `backend/`: create `requirements.txt` with FastAPI, Uvicorn, Pydantic, python-dotenv, psycopg2-binary, qdrant-client, and openai.
- [X] T003 Create frontend component directory structure: `docs/src/theme/Chatbot/components` and `docs/src/api`.
- [X] T004 [P] Configure `.env` file in `backend/` with placeholder credentials for Neon, Qdrant, and OpenAI.
- [ ] T005 [P] Setup a free-tier cloud instance on Qdrant Cloud.

## Phase 2: Foundational Backend & Data

This phase establishes the database schema and the data ingestion pipeline, which are prerequisites for the API.

- [X] T006 Implement Pydantic models for User, ChatSession, and ChatMessage in `backend/src/models/chat.py`.
- [ ] T007 [P] Set up a new project on Neon Postgres and configure the database schema based on `data-model.md`.
- [X] T008 Implement a data ingestion script in `backend/scripts/ingest_book.py` to read the book content, generate embeddings using OpenAI, and store them in Qdrant.
- [X] T009 [P] Write a unit test for the ingestion script in `backend/tests/unit/test_ingestion.py`.

## Phase 3: User Story 1 - General Question (MVP)

This phase delivers the core MVP: the ability to ask a general question and receive an answer.

- **Goal**: A user can open the chat UI, ask a question, and get an answer based on the whole book.
- **Independent Test**: The full end-to-end flow can be tested. Open the UI, send a POST request to the backend, and see an answer appear.

- [X] T010 [US1] Implement the `/api/query` endpoint in `backend/src/api/query.py`. It should accept a question, query Qdrant, generate an answer with OpenAI, and log the interaction to Postgres.
- [X] T011 [P] [US1] Create the main React component for the chat interface in `docs/src/theme/Chatbot/index.tsx`.
- [X] T012 [P] [US1] Create the `MessageBubble`, `MessageInput`, and `LoadingIndicator` components in `docs/src/theme/Chatbot/components/`.
- [X] T013 [US1] Implement the frontend API service in `docs/src/api/chatbot.ts` to send a POST request to the `/api/query` backend endpoint.
- [X] T014 [US1] Integrate the API service with the frontend: send the user's question and display the returned answer.
- [X] T015 [US1] Implement basic auto-scrolling on new messages in the chat window.
- [X] T016 [P] [US1] Write an integration test for the `/api/query` endpoint in `backend/tests/integration/test_api.py`.

## Phase 4: User Story 2 - Contextual Question

This phase adds the ability to ask questions about a selected piece of text.

- **Goal**: A user can highlight text, click a button, and ask a question specifically about that text.
- **Independent Test**: Can be tested by selecting text, seeing the pop-up button, and verifying the contextual query works end-to-end.

- [X] T017 [US2] Implement a text selection handler in the frontend that displays a pop-up button when text is selected in an MDX page.
- [X] T018 [US2] Modify the frontend API service in `docs/src/api/chatbot.ts` to pass the `selected_text` to the backend.
- [X] T019 [US2] Update the `/api/query` endpoint in `backend/src/api/query.py` to use `selected_text` as the primary context for the RAG query when it is provided.
- [X] T020 [P] [US2] Update the integration test in `backend/tests/integration/test_api.py` to cover the `selected_text` case.

## Phase 5: Polish & Integration

This final phase focuses on styling, final integration, and improving the user experience.

- [X] T021 [P] Apply the "black-gold" theme and responsive styles to all chatbot components in `docs/src/theme/Chatbot/styles.css`.
- [X] T022 [P] Implement robust error handling in the frontend to display user-friendly messages if the API call fails.
- [X] T023 Embed the Chatbot component into the main Docusaurus layout or relevant MDX pages so the floating button appears.
- [X] T024 [P] Implement the user authentication flow (login/logout) and JWT handling on the frontend.
- [X] T025 [P] Secure the `/api/query` endpoint to require a valid JWT token.

## Dependencies & Execution Strategy

- **Implementation Strategy**: The tasks are structured to deliver a testable MVP first (User Story 1), followed by the contextual query feature (User Story 2). This allows for incremental development and testing.
- **Dependency Graph**:
  - `Phase 1` -> `Phase 2` -> `Phase 3 (US1)` -> `Phase 4 (US2)` -> `Phase 5`
- **Parallel Opportunities**: Tasks marked with `[P]` can often be worked on in parallel within their phase, especially frontend and backend tasks that depend only on the agreed-upon API contract. For example, in Phase 3, `T010` (backend API) and `T011-T013` (frontend components) can be developed simultaneously.
- **Suggested MVP**: Completing all tasks up to and including **Phase 3** will deliver the core functionality and constitute a strong Minimum Viable Product.
