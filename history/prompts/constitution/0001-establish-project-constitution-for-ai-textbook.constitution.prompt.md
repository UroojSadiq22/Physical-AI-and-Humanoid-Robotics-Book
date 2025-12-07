---
id: 0001
title: Establish Project Constitution for AI Textbook
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-pro
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - none
---

## Prompt

/sp.constitution Project: Physical AI & Humanoid Robotics — AI-Native Technical Textbook for Panaversity Hackathon I

Core principles:
- Technical accuracy aligned with ROS 2, Gazebo, Unity, NVIDIA Isaac, and modern humanoid robotics concepts
- Clarity for learners with basic programming knowledge (Python/AI beginners to intermediate robotics students)
- Consistency with Panaversity AI-native textbook standards
- Modular and reusable content generation for Docusaurus-based structure
- Structured writing suitable for both human readers and RAG chatbot ingestion
- Progressive learning design: simple → advanced → applied robotics
- High-quality explanations, diagrams (text-form), and real-world robotics examples

Key standards:
- All explanations must be 100% original (no copying from any copyrighted robotics documentation)
- Use beginner-friendly but technically correct explanations
- Every chapter must include:
  - Learning objectives
  - Technical breakdowns
  - ROS 2 / Gazebo / Unity / Isaac examples where relevant
  - Small code snippets (Python or ROS 2)
  - Diagrams described in text
  - Assignments or practice tasks
  - Key takeaways
- Writing clarity should match “technical textbook” tone (Flesch-Kincaid grade 10–12)
- All content must be compatible with Docusaurus markdown
- Style must be consistent across the entire book

Constraints:
- No plagiarism (0% tolerance)
- Avoid any copyrighted robot model names unless open-source (e.g., use generic humanoid robot)
- No claims requiring external citations — the book is self-contained
- Each chapter should be 800–1500 words
- Total book size may vary but must remain fully structured and cohesive
- Must follow Spec-Kit Plus book specification structure (`id`, `title`, `slug`, etc.)
- Output must remain deterministic and reproducible when re-generated

Success criteria:
- Book compiles correctly in Docusaurus without formatting errors
- Every chapter maintains consistent structure and tone
- Output is fully usable with RAG chatbot training (clean sections, no mixed formatting)
- All examples are correct and relevant to the course
- Content is original, clear, and free of factual errors
- Completes full table of contents exactly matching the course’s modules and weeks

## Response snapshot

I have successfully updated the project constitution and related templates. The new constitution for the "Physical AI & Humanoid Robotics" project is now in effect.

## Outcome

- ✅ Impact: Project constitution established, setting the standards for the AI-native textbook.
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md and .specify/templates/plan-template.md
- 🔁 Next prompts: none
- 🧠 Reflection: The constitution is a critical document that will guide the project.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
