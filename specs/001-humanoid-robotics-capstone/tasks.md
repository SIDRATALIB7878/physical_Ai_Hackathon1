---

description: "Task list for Add GitHub Profile Link feature"
---

# Tasks: Add GitHub Profile Link

**Input**: User request for updating GitHub profile link.
**Prerequisites**: Existing Docusaurus project configuration.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase: Refactor & Configuration Updates

**Purpose**: To integrate the user's GitHub profile link into the Docusaurus project's configuration wherever a GitHub icon or reference is used.

- [ ] T001 Update `href` for GitHub link in `themeConfig.navbar.items` in `docusaurus.config.js` to `https://github.com/ersa-rani`.
- [ ] T002 Update `href` for GitHub link in `themeConfig.footer.links` in `docusaurus.config.js` to `https://github.com/ersa-rani`.
- [ ] T003 Update `docs.editUrl` in `docusaurus.config.js` to `https://github.com/ersa-rani/humanoid-robotics-book/tree/main/`.
- [ ] T004 Update `organizationName` in `docusaurus.config.js` to `ersa-rani`.
- [ ] T005 Update `url` in `docusaurus.config.js` to `https://ersa-rani.github.io`.
- [ ] T006 Verify updated GitHub links by running local Docusaurus server (`npm run start` or `npm run serve`) and checking the navigation bar, footer, and edit links on documentation pages.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Refactor & Configuration Updates**: No dependencies - can start immediately.

### Parallel Opportunities

- Tasks T001-T005 can be executed in any order, as they modify different properties within the same file.

---

## Implementation Strategy

### Incremental Delivery

1.  Complete tasks T001-T005 to update configuration.
2.  Complete T006 for verification.

---

## Notes

- This `tasks.md` is specifically for integrating the user's GitHub profile link.
- Manual verification (T006) is critical to ensure all links are correctly updated and functional.
