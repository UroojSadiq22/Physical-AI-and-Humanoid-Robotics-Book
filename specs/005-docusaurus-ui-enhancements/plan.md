# Implementation Plan: Docusaurus UI Enhancements (Fix & Chatbot)

**Branch**: `005-docusaurus-ui-enhancements` | **Date**: 2025-12-11 | **Spec**: [Consolidated from 003-docusaurus-ui-fix/spec.md, 004-floating-chatbot-widget/spec.md]
**Input**: Consolidated Feature specifications for Docusaurus UI Fix and Floating Chatbot Widget.

## Summary

This plan outlines the implementation steps to enhance the Docusaurus user interface by:
1.  **Simplifying the Navbar**: Restricting the top navigation bar to a single "Book" item that links to the documentation section.
2.  **Restoring Classic Homepage**: Ensuring the homepage prominently displays a hero layout consistent with the default Docusaurus classic template.
3.  **Refining Sidebar Navigation**: Organizing all book modules (Module 1, 2, 3, 4) exclusively within the documentation sidebar.
4.  **Integrating a Floating Chatbot Widget**: Adding a modern, globally available floating chatbot widget with smooth open/close transitions, a minimal UI, and no theme conflicts.

The goal is to create a clean, intuitive, and standard Docusaurus documentation site experience while integrating a user-friendly chatbot.

## Technical Context

**Language/Version**: TypeScript, JavaScript (for Docusaurus configuration and React components)
**Primary Dependencies**: React, Docusaurus (core libraries, `@docusaurus/preset-classic`), clsx (for conditional CSS classes)
**Storage**: N/A (UI-focused feature)
**Testing**: Manual visual and functional testing across browsers and devices; accessibility checks.
**Target Platform**: Web (Docusaurus static site)
**Project Type**: Frontend web application (Docusaurus)
**Performance Goals**: Smooth UI transitions, responsive layout, minimal impact on page load times.
**Constraints**: No breaking changes to core Docusaurus functionality; chatbot component must be isolated; no collision with Docusaurus default styles; modules organized only in sidebar.
**Scale/Scope**: Enhancements to the existing Docusaurus frontend UI for a single textbook website.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

*   [X] **Technical Accuracy**: The plan adheres to Docusaurus and React best practices.
*   [X] **Clarity for Learners**: The UI changes aim to improve navigation clarity for learners. The chatbot enhances accessibility to information.
*   [X] **Consistency**: The plan ensures consistency with Docusaurus classic template standards and internal textbook standards where applicable.
*   [X] **Modular and Reusable Content**: UI components are designed to be modular (e.g., chatbot widget).
*   [ ] **Structured Writing for RAG**: N/A (UI feature).
*   [ ] **Progressive Learning Design**: N/A (UI feature).
*   [X] **High-Quality Explanations**: The UI enhancements support access to high-quality explanations.

**Result**: The plan aligns with all relevant constitutional principles.

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-ui-enhancements/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # N/A for this feature
├── quickstart.md        # N/A for this feature
├── contracts/           # N/A for this feature
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (relative to `docs/` directory)

```text
docs/
├── docusaurus.config.ts    # Navbar configuration, homepage plugin
├── sidebars.ts             # Sidebar module structure
├── src/
│   ├── css/custom.css      # Custom CSS overrides to review/clean
│   ├── pages/index.tsx     # Homepage component (if custom hero section is needed)
│   ├── theme/
│   │   ├── Layout/index.tsx # Override to embed global components (e.g., Chatbot)
│   │   └── Chatbot/         # (already exists from 002-rag-chatbot)
│   │       ├── index.tsx
│   │       ├── styles.module.css
│   │       └── components/  # MessageBubble, MessageInput, LoadingIndicator
│   └── api/                 # (already exists from 002-rag-chatbot)
│       └── chatbot.ts
├── blog/
├── docs/
├── static/
└── vercel.json
```

**Structure Decision**: The modifications will primarily involve existing Docusaurus configuration files (`docusaurus.config.ts`, `sidebars.ts`) and existing Docusaurus theme overrides or custom files (`src/css/custom.css`, `src/pages/index.tsx`, `src/theme/Layout/index.tsx`). The Chatbot widget, previously implemented as a global component, will be integrated into the `Layout` override.

## Architecture Decisions & File-level Modifications

### 1. Navbar Simplification & Sidebar Integration
-   **Decision**: Modify `docusaurus.config.ts` to include only one navbar item ("Book") pointing to the documentation route. Ensure `sidebars.ts` correctly defines all book modules (`module1`, `module2`, `module3`, `module4`).
-   **Rationale**: Directly addresses FR-001, FR-002, FR-003, FR-006, FR-007. Configuration is the most direct way to control Docusaurus navigation.
-   **Tradeoffs**: None, as this aligns with standard Docusaurus practices.
-   **Files to modify**:
    -   `docs/docusaurus.config.ts`
    -   `docs/sidebars.ts`

### 2. Classic Homepage Restoration
-   **Decision**: Review `docs/src/pages/index.tsx` to ensure it implements the classic Docusaurus hero layout. If it currently uses custom components that deviate, replace them with the standard structure or reintroduce the classic template's elements.
-   **Rationale**: Addresses FR-004. Ensures a consistent and expected landing page experience.
-   **Tradeoffs**: May require refactoring existing homepage components if they are not compatible.
-   **Files to modify**:
    -   `docs/src/pages/index.tsx`
    -   Potentially `docs/src/components/HomepageFeatures/index.tsx` and `docs/src/components/HomepageFeatures/styles.module.css` if they are part of the custom hero layout that needs removal.

### 3. Floating Chatbot Widget Integration
-   **Decision**: The existing `Chatbot` component (from `002-rag-chatbot` feature) will be mounted as a global element by modifying `docs/src/theme/Layout/index.tsx` to wrap the original Docusaurus Layout and render the `Chatbot` component.
-   **Rationale**: Addresses FR-006 for global availability. Using `Layout` override ensures the component is present on all pages.
-   **Tradeoffs**: Requires careful CSS scoping within the chatbot component to avoid theme conflicts.
-   **Files to modify**:
    -   `docs/src/theme/Layout/index.tsx` (already modified during `002-rag-chatbot` implementation, will verify)
    -   `docs/src/theme/Chatbot/index.tsx` (existing, will verify integration)
    -   `docs/src/theme/Chatbot/styles.module.css` (existing, will verify scoping)

### 4. CSS Cleanup & Scoping Strategy
-   **Decision**: Review `docs/src/css/custom.css` for any overrides affecting navbar, sidebar, or homepage that conflict with the desired clean/minimal theme. Remove or refactor as necessary. For the chatbot, continue using CSS Modules (`styles.module.css`) to ensure strict scoping.
-   **Rationale**: Addresses FR-005, FR-008, FR-009. Essential for maintaining Docusaurus theme integrity and preventing UI bugs.
-   **Tradeoffs**: Requires careful inspection of existing CSS, potentially leading to visual changes in other parts of the site if custom styles were broadly applied.
-   **Files to modify**:
    -   `docs/src/css/custom.css`
    -   `docs/src/theme/Chatbot/styles.module.css` (verify existing scoping)

## Component Breakdown (Chatbot Widget)

The chatbot widget is already broken down into the following existing components, developed as part of the `002-rag-chatbot` feature:
-   `docs/src/theme/Chatbot/index.tsx`: Main chatbot container, state management, FAB logic, authentication.
-   `docs/src/theme/Chatbot/components/MessageBubble.tsx`: Displays individual chat messages.
-   `docs/src/theme/Chatbot/components/MessageInput.tsx`: Input field for user messages.
-   `docs/src/theme/Chatbot/components/LoadingIndicator.tsx`: Visual feedback during message processing.
-   `docs/src/api/chatbot.ts`: Frontend API service for backend communication.

## CSS/Animation Strategy

-   **CSS Scoping**: For the chatbot widget, CSS Modules (`*.module.css`) will be used (`docs/src/theme/Chatbot/styles.module.css`) to ensure styles are localized to the component and do not leak globally.
-   **Animations**: CSS transitions and keyframe animations will be used for smooth open/close of the chat window and icon transformation, adhering to modern UX principles.
-   **Theme Adherence**: General UI fixes will prioritize using Docusaurus CSS variables (`var(--ifm-color-primary)`) to ensure consistency with the overall Docusaurus theme.
-   **Responsive Design**: Media queries will be utilized within `styles.module.css` and `custom.css` (if needed for global overrides) to ensure responsive behavior across different screen sizes.

## Testing Checklist

-   [ ] **Navbar Functionality**: Verify only "Book" link is present and correctly navigates to docs.
-   [ ] **Sidebar Functionality**: Verify sidebar appears on docs pages and contains all modules.
-   [ ] **Homepage Layout**: Verify hero section matches classic Docusaurus template.
-   [ ] **Chatbot Visibility**: Verify chatbot icon is visible on all pages (bottom-right).
-   [ ] **Chatbot Open/Close**: Verify smooth animations and icon transformations.
-   [ ] **Chatbot Responsiveness**: Verify widget behavior on mobile/tablet devices.
-   [ ] **Accessibility**: Verify keyboard navigation and screen reader compatibility for chatbot and UI elements.
-   [ ] **Theme Integrity**: Verify no regressions in other parts of the Docusaurus theme due to UI changes or chatbot integration.

## Validation Steps

-   **Build Docusaurus**: Run `npm run build` in `docs/` to ensure no build errors.
-   **Serve Docusaurus Locally**: Run `npm run start` in `docs/` to inspect changes in a development environment.
-   **Browser Compatibility**: Test across Chrome, Firefox, Safari (and Edge if applicable).
-   **Device Responsiveness**: Test on various screen sizes and device emulators.
-   **Functional Check**: Follow all user scenarios outlined in the respective `spec.md` files.

## Complexity Tracking

No constitutional violations were identified that require justification.
