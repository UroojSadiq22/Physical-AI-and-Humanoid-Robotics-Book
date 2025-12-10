# Actionable Tasks: Docusaurus UI Enhancements (Fix & Chatbot)

**Branch**: `005-docusaurus-ui-enhancements` | **Date**: 2025-12-11 | **Plan**: [plan.md](plan.md)

This document breaks down the implementation of the Docusaurus UI enhancements into a series of actionable, dependency-ordered tasks.

## Phase 1: Setup & Initial Cleanup

This phase focuses on initial environment setup and cleaning up conflicting styles.

- [X] T001 Review `docs/src/css/custom.css` and remove any overrides affecting navbar, sidebar, or homepage that conflict with a clean/minimal Docusaurus theme.
- [X] T002 Ensure `docs/src/theme/Chatbot/styles.module.css` (from `002-rag-chatbot` feature) uses CSS Modules for strict scoping.

## Phase 2: UI Fix - Navbar & Sidebar

This phase implements the simplification of the navbar and the correct structuring of the sidebar.
**(Relates to User Story 1 - Navigate to the Book from `003-docusaurus-ui-fix`)**

- [X] T003 Modify `docs/docusaurus.config.ts` to show only one navbar item: "Book", pointing to the documentation root (`/docs`).
- [X] T004 Update `docs/sidebars.ts` to ensure `module1`, `module2`, `module3`, `module4` are organized in the sidebar, matching the Docusaurus docs layout.

## Phase 3: UI Fix - Homepage Restoration

This phase focuses on restoring the classic Docusaurus homepage hero layout.
**(Relates to User Story 2 - View Home Page from `003-docusaurus-ui-fix`)**

- [X] T005 Review `docs/src/pages/index.tsx` and ensure it implements the classic Docusaurus hero layout. If custom components (`HomepageFeatures`) deviate, modify `docs/src/pages/index.tsx` to use the standard structure or reintroduce classic template elements.
- [X] T006 If `HomepageFeatures` were removed/modified, clean up any associated files in `docs/src/components/HomepageFeatures/`.

## Phase 4: Floating Chatbot Widget Integration

This phase verifies and integrates the existing floating chatbot widget into the Docusaurus theme.
**(Relates to User Story 1 - Open Chatbot, User Story 2 - Close Chatbot, User Story 3 - Global Availability and Theme Compatibility from `004-floating-chatbot-widget`)**

- [X] T007 Verify `docs/src/theme/Chatbot/index.tsx` and its sub-components are correctly implemented and functional as a standalone widget.
- [X] T008 Verify `docs/src/theme/Layout/index.tsx` (from `002-rag-chatbot` feature) correctly mounts the Chatbot component globally.
- [X] T009 Ensure existing styles in `docs/src/theme/Chatbot/styles.module.css` adhere to the "minimal UI, rounded corners, 320px width" requirements and use smooth animations.
- [X] T010 Check mobile responsiveness of the chatbot widget.
- [X] T011 Review chatbot widget for accessibility (keyboard navigation, ARIA attributes).

## Phase 5: Final Validation & Polish

This final phase focuses on comprehensive testing and ensuring overall quality.

- [X] T012 Perform a full visual inspection of the site on various browsers and screen sizes (desktop, tablet, mobile) to confirm all UI fixes and chatbot integration are correct and free of regressions.
- [X] T013 Verify that the Docusaurus build process (`npm run build` in `docs/`) completes without warnings or errors.
- [X] T014 Conduct functional testing to ensure navbar, sidebar, homepage, and chatbot open/close mechanisms all work as expected.

## Dependencies & Execution Strategy

-   **Implementation Strategy**: The tasks are ordered to first address the foundational UI fixes (navbar, sidebar, homepage) before integrating the chatbot widget.
-   **Dependency Graph**:
    -   `Phase 1` -> `Phase 2` -> `Phase 3` -> `Phase 4` -> `Phase 5`
-   **Parallel Opportunities**: Within `Phase 4`, verification tasks (T007-T011) can be done in parallel where appropriate.
-   **Suggested MVP**: Completing tasks up to and including **Phase 3** will deliver the core UI fixes for the Docusaurus site.

## Testing Checklist

(Refer to "Testing Checklist" section in [plan.md](plan.md))
