# Tasks: Physical AI & Humanoid Robotics — Textbook

**Input**: Design documents from `/specs/001-textbook-spec/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story (module) to enable independent implementation and testing of each module.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Create Docusaurus project structure in the `docs` directory.
- [X] T002 [P] Configure Docusaurus sidebar for the 4 modules in `docs/sidebars.js`.

---

## Phase 2: User Story 1 - Module 1: ROS 2 — The Robotic Nervous System (Priority: P1) 🎯 MVP

**Goal**: Write the chapters for the first module, covering the fundamentals of ROS 2.

**Independent Test**: The chapters for Module 1 are complete and can be read and understood independently. The code examples can be run successfully.

### Implementation for User Story 1

- [X] T003 [US1] Write Chapter 1: Introduction to ROS 2, in `docs/module1/chapter1.md`.
- [X] T004 [US1] Write Chapter 2: ROS 2 Nodes, in `docs/module1/chapter2.md`.
- [X] T005 [US1] Write Chapter 3: ROS 2 Topics, in `docs/module1/chapter3.md`.
- [X] T006 [US1] Write Chapter 4: ROS 2 Services and Actions, in `docs/module1/chapter4.md`.

---

## Phase 3: User Story 2 - Module 2: Digital Twin — Gazebo & Unity (Priority: P2)

**Goal**: Write the chapters for the second module, covering robot simulation.

**Independent Test**: The chapters for Module 2 are complete. The URDF models can be loaded in Gazebo and Unity.

### Implementation for User Story 2

- [X] T007 [US2] Write Chapter 5: Introduction to Simulation, in `docs/module2/chapter5.md`.
- [X] T008 [US2] Write Chapter 6: Building a Robot with URDF, in `docs/module2/chapter6.md`.
- [X] T009 [US2] Write Chapter 7: Simulating a Robot with Gazebo, in `docs/module2/chapter7.md`.
- [X] T010 [US2] Write Chapter 8: Simulating a Robot with Unity, in `docs/module2/chapter8.md`.

---

## Phase 4: User Story 3 - Module 3: AI-Robot Brain — NVIDIA Isaac (Priority: P3)

**Goal**: Write the chapters for the third module, covering AI integration with NVIDIA Isaac.

**Independent Test**: The chapters for Module 3 are complete. The examples using Isaac ROS can be run.

### Implementation for User Story 3

- [X] T011 [US3] Write Chapter 9: Introduction to NVIDIA Isaac, in `docs/module3/chapter9.md`.
- [X] T012 [US3] Write Chapter 10: Isaac Sim for Perception, in `docs/module3/chapter10.md`.
- [X] T013 [US3] Write Chapter 11: Integrating Isaac ROS Gems, in `docs/module3/chapter11.md`.

---

## Phase 5: User Story 4 - Module 4: Vision-Language-Action (VLA) (Priority: P4)

**Goal**: Write the chapters for the fourth module, covering VLA models and the capstone project.

**Independent Test**: The chapters for Module 4 are complete. The capstone project can be built and run.

### Implementation for User Story 4

- [X] T014 [US4] Write Chapter 12: Introduction to VLAs, in `docs/module4/chapter12.md`.
- [X] T015 [US4] Write Chapter 13: Building the Capstone Project, in `docs/module4/chapter13.md`.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T016 [P] Review and edit all chapters for clarity, consistency, and correctness.
- [X] T017 [P] Validate all code examples.
- [X] T018 Run Docusaurus build to ensure no errors.
- [X] T019 Validate RAG compatibility.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **User Stories (Phase 2+)**: All depend on Setup phase completion.
  - User stories can then proceed in parallel (if staffed) or sequentially in priority order (P1 → P2 → P3 → P4).
- **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup (Phase 1).
- **User Story 2 (P2)**: Can start after Setup (Phase 1).
- **User Story 3 (P3)**: Can start after Setup (Phase 1).
- **User Story 4 (P4)**: Depends on User Story 1, 2, and 3.

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel.
- Once Setup phase completes, User Stories 1, 2, and 3 can start in parallel.
- All Polish tasks marked [P] can run in parallel.

---

## Implementation Strategy

### Incremental Delivery

1. Complete Setup → Foundation ready.
2. Add User Story 1 → Test independently → MVP!
3. Add User Story 2 → Test independently.
4. Add User Story 3 → Test independently.
5. Add User Story 4 → Test independently.
6. Each story adds value without breaking previous stories.
