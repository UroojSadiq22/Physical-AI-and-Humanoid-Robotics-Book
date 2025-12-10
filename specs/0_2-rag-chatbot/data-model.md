# Data Model: RAG Chatbot

**Date**: 2025-12-10

This document defines the database schema for the RAG Chatbot feature, which will be implemented using Neon Postgres. Based on the feature specification, a user authentication system is required to track chat sessions.

## Table: `users`

Stores user account information.

| Column | Type | Constraints | Description |
|---|---|---|---|
| `id` | `UUID` | Primary Key, Default: `gen_random_uuid()` | Unique identifier for the user. |
| `email` | `VARCHAR(255)` | Not Null, Unique | User's email address, used for login. |
| `hashed_password` | `VARCHAR(255)` | Not Null | Hashed password for the user. |
| `created_at` | `TIMESTAMP WITH TIME ZONE` | Not Null, Default: `NOW()` | Timestamp of when the user account was created. |
| `updated_at` | `TIMESTAMP WITH TIME ZONE` | Not Null, Default: `NOW()` | Timestamp of the last update to the user account. |

**Indexes**:
- `users_email_key` (unique index on `email`)

---

## Table: `chat_sessions`

Stores information about a single chat session, linked to a user.

| Column | Type | Constraints | Description |
|---|---|---|---|
| `id` | `UUID` | Primary Key, Default: `gen_random_uuid()` | Unique identifier for the chat session. |
| `user_id` | `UUID` | Not Null, Foreign Key -> `users.id` | The user who initiated the session. |
| `created_at` | `TIMESTAMP WITH TIME ZONE` | Not Null, Default: `NOW()` | Timestamp of when the session started. |

**Indexes**:
- `chat_sessions_user_id_idx` (index on `user_id`)

---

## Table: `chat_messages`

Stores individual messages within a chat session.

| Column | Type | Constraints | Description |
|---|---|---|---|
| `id` | `UUID` | Primary Key, Default: `gen_random_uuid()` | Unique identifier for the message. |
| `session_id` | `UUID` | Not Null, Foreign Key -> `chat_sessions.id` | The session this message belongs to. |
| `author` | `VARCHAR(50)` | Not Null, Check (`author` IN ('user', 'bot')) | The author of the message (either 'user' or 'bot'). |
| `content` | `TEXT` | Not Null | The text content of the message. |
| `context_text` | `TEXT` | Nullable | The selected text used as context for the user's query, if any. |
| `created_at` | `TIMESTAMP WITH TIME ZONE` | Not Null, Default: `NOW()` | Timestamp of when the message was created. |

**Indexes**:
- `chat_messages_session_id_idx` (index on `session_id`)

## Relationships

- A `user` can have multiple `chat_sessions`.
- A `chat_session` belongs to one `user`.
- A `chat_session` can have multiple `chat_messages`.
- A `chat_message` belongs to one `chat_session`.
