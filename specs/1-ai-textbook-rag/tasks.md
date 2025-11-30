# Tasks: AI-Native Textbook Platform

This document breaks down the implementation of the "Physical AI & Humanoid Robotics" textbook project into atomic tasks, derived from the specification and implementation plan.

## Phase 1: Foundation & Core Content (Days 1-2)
**Goal:** Establish the project structure and create the initial, static version of the textbook.
**Traceability:** `spec.md` (Success Criteria: Comprehensive Content, User-Friendly Interface), `plan.md` (Phase 1)

| Task ID | Duration | Depends On | Acceptance Criterion | Output |
| --- | --- | --- | --- | --- |
| **T1.1** | 30 min | - | Docusaurus project is initialized and runs locally. | `physical-ai-robotics/` directory with a running Docusaurus instance. |
| **T1.2** | 30 min | - | FastAPI project is initialized with basic "hello world" endpoint. | `physical-ai-robotics/backend/` directory with a running FastAPI instance. |
| **T1.3** | 60 min | - | Neon Postgres and Qdrant instances are provisioned and credentials are saved securely. | Credentials file/environment variables for DB access. |
| **T1.3a** | 30 min | - | `.env` file or similar is set up and integrated into the project for environment variables. | `physical-ai-robotics/.env.example` and configuration loading logic in FastAPI. |
| **T1.4** | 30 min | T1.3 | SQL script defines `users` and `progress` tables. | `backend/db/schema.sql` file. |
| **T1.4a** | 60 min | - | A validated Claude prompt is ready to generate content for a textbook section. | `generation/prompts/chapter-section-template.txt` and a small generated sample. |
| **T1.5** | 120 min| T1.4a | Prompt for Claude generates content for Chapter 1. | `generation/prompts/chapter1.txt` and `generation/output/chapter1.md`. |
| **T1.6** | 60 min | T1.5 | Chapter 1 Markdown is correctly formatted and placed in Docusaurus `docs/`. | `physical-ai-robotics/docs/chapter-1/intro.md` |
| **T1.7** | 60 min | T1.1 | Docusaurus `sidebars.ts` is updated to include Chapter 1. | Chapter 1 appears in the site navigation. |
| **T1.8** | 90 min | T1.1 | Basic CSS styling (theme, fonts, colors) is applied. | `physical-ai-robotics/src/css/custom.css` is updated and styles are visible. |

**Checkpoint 1:** A static version of the textbook with one chapter is deployed and viewable. All project foundations are in place.

---

## Phase 2: RAG Chatbot Implementation (Days 3-4)
**Goal:** Implement and integrate the RAG chatbot for interactive learning.
**Traceability:** `spec.md` (Success Criteria: Functional RAG Chatbot), `plan.md` (Phase 2)

| Task ID | Duration | Depends On | Acceptance Criterion | Output |
| --- | --- | --- | --- | --- |
| **T2.1** | 90 min | T1.3 | Ingestion script reads all `.md` files from the `docs/` directory. | Script correctly prints file paths. `scripts/ingest.py` file created. |
| **T2.2** | 60 min | T2.1 | Text from Markdown files is chunked into 256-token segments. | Function in `scripts/ingest.py` that returns a list of text chunks. |
| **T2.3** | 60 min | T2.2 | Each text chunk is converted into a vector embedding using an embedding model. | Function in `scripts/ingest.py` that returns embeddings. |
| **T2.4** | 60 min | T2.3 | Embeddings and corresponding text chunks are stored in the Qdrant database. | Data is verified via Qdrant's UI or API count check. |
| **T2.5** | 120 min| T1.2, T2.4 | FastAPI `/chat` endpoint receives a query, performs a vector search, and returns top results. | Endpoint tested with a tool like `curl` or Postman returns relevant text chunks. `backend/routers/chat.py` |
| **T2.6** | 90 min | T2.5 | RAG model takes the user query and retrieved context to generate an answer. | `/chat` endpoint returns a generated answer instead of just chunks. `backend/services/rag.py` |
| **T2.6a** | 60 min | T2.6 | Unit tests for `backend/services/rag.py` pass. | `backend/tests/unit/test_rag.py`. |
| **T2.7** | 90 min | T1.1 | A basic, non-functional chat widget UI is created as a React component. | `src/components/ChatWidget/index.tsx` file created and component visible on site. |
| **T2.8** | 60 min | T2.7, T2.6 | Chat widget's input is sent to the `/chat` endpoint on submit. | Browser network tab shows a successful request to the backend. |
| **T2.9** | 60 min | T2.8 | The response from the backend is displayed in the chat widget's message area. | Chatbot provides a complete answer to a user's question. |

**Checkpoint 2:** The RAG chatbot is fully functional. Users can ask questions about the textbook content and receive accurate answers.

---

## Phase 3: User Authentication & Personalization (Days 5-6)
**Goal:** Enable user accounts to track progress and personalize the experience.
**Traceability:** `spec.md` (Success Criteria: Personalized Learning Experience), `plan.md` (Phase 3)

| Task ID | Duration | Depends On | Acceptance Criterion | Output |
| --- | --- | --- | --- | --- |
| **T3.1** | 90 min | T1.2, T1.4 | `fastapi-users` is integrated; `/login` and `/register` endpoints are functional. | Users can be created and authenticated via API calls. `backend/routers/auth.py` |
| **T3.2** | 60 min | T1.1 | A simple Login/Register page is created in Docusaurus. | `src/pages/login.tsx` created and accessible. |
| **T3.3** | 60 min | T3.1, T3.2 | Frontend login page successfully authenticates with the backend and stores a token. | User is redirected to the homepage after a successful login. |
| **T3.4** | 90 min | T3.1 | A `/progress` endpoint is created to record which chapters a user has viewed. | Endpoint accepts a `chapter_id` and `user_id`, and stores it in the `progress` table. `backend/routers/progress.py` |
| **T3.4a** | 60 min | T3.4 | Unit tests for `backend/routers/progress.py` pass. | `backend/tests/unit/test_progress.py`. |
| **T3.5** | 60 min | T3.4 | The frontend calls the `/progress` endpoint when a user visits a chapter page. | Database records are verified for the logged-in user. |
| **T3.6** | 60 min | T3.1 | A "My Profile" page is created that displays the user's email. | `src/pages/profile.tsx` is created and displays user info. |
| **T3.7** | 90 min | T3.5, T3.6 | The profile page fetches and displays the user's viewing history from the backend. | Profile page shows a list of completed chapters. |

**Checkpoint 3:** User authentication is complete. Users can sign up, log in, and track their reading progress.

---

## Phase 4: Urdu Translation (Day 7)
**Goal:** Add on-the-fly Urdu translation for the textbook content.
**Traceability:** `spec.md` (Success Criteria: Urdu Translation), `plan.md` (Phase 4)

| Task ID | Duration | Depends On | Acceptance Criterion | Output |
| --- | --- | --- | --- | --- |
| **T4.1** | 90 min | T1.2 | A `/translate` endpoint in FastAPI takes text and a target language, returning the translation. | Endpoint successfully translates a sample text block to Urdu. `backend/services/translation.py` |
| **T4.1a** | 60 min | T4.1 | Unit tests for `backend/services/translation.py` pass. | `backend/tests/unit/test_translation.py`. |
| **T4.2** | 30 min | T1.1 | A language switcher (English/Urdu) is added to the Docusaurus navigation bar. | UI element is visible on the site. |
| **T4.3** | 120 min| T4.1, T4.2 | When "Urdu" is selected, the frontend fetches the translated content for the current page and displays it. | The text on a chapter page is replaced with Urdu text. |
| **T4.4** | 60 min | T4.3 | Translated content is cached in the frontend (e.g., in `localStorage` or state) to avoid re-fetching. | Network tab shows the `/translate` endpoint is only called once per page visit. |

**Checkpoint 4:** The entire textbook can be read in either English or Urdu, with seamless switching between the two.

---

## Phase 5: Testing, Deployment, & Polish (Day 8)
**Goal:** Ensure the application is stable, deployed, and user-friendly.
**Traceability:** `spec.md` (All Success Criteria), `plan.md` (Phase 5)

| Task ID | Duration | Depends On | Acceptance Criterion | Output |
| --- | --- | --- | --- | --- |
| **T5.1a** | 60 min | T2.9 | End-to-end tests for RAG chatbot pass. | Test report. |
| **T5.1b** | 60 min | T3.7 | End-to-end tests for user authentication and progress tracking pass. | Test report. |
| **T5.1c** | 60 min | T4.4 | End-to-end tests for language translation pass. | Test report. |
| **T5.2** | 90 min | T1.1 | CI/CD pipeline is configured to auto-deploy the Docusaurus frontend to Vercel. | A push to the `main` branch triggers a successful deployment. |
| **T5.3** | 90 min | T1.2 | CI/CD pipeline is configured to auto-deploy the FastAPI backend to Render. | A push to the `main` branch triggers a successful deployment. |
| **T5.4** | 120 min| T5.2, T5.3 | A final review of the deployed application's UI/UX is conducted, and minor style adjustments are made. | `src/css/custom.css` updated with final polish. |
| **T5.5** | 60 min | T5.4 | The project's `README.md` is updated with final setup and deployment instructions. | `README.md` is complete and accurate. |

**Final Checkpoint:** The project is fully deployed, tested, and meets all success criteria outlined in the specification.