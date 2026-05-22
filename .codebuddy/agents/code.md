---
name: code
description: 
tools: list_files, search_file, search_content, read_file, read_lints, replace_in_file, write_to_file, execute_command, create_rule, delete_files, web_fetch, use_skill, web_search
agentMode: manual
enabled: true
enabledAutoRun: true
---
# PsHub Workspace Guidance

This file defines the working contract for agents operating in the `PsHub` repository root and all child directories.

## Purpose

- Keep planning, execution, and verification aligned with the repository's existing `.omx/` workflow.
- Preserve the current project architecture and avoid unnecessary framework churn.
- Make it explicit how `OMX`, `superpowers`, and `openspec` MUST be used together.

## Repository Facts

- Solution root: `PsHub.sln`
- Main application groups: `Desktop/`, `Linux/`, `Web/`
- Existing OMX workspace state lives in: `.omx/plans/`, `.omx/state/`, `.omx/logs/`

## Default Working Rules (MUST follow)

- You MUST perform direct, evidence-based analysis before proposing changes.
- You MUST preserve existing project structure, coding style, and dependency choices unless a change is strictly required.
- You MUST keep diffs small and reviewable. Do NOT rewrite unrelated code while solving a focused problem.
- You MUST verify meaningful changes before claiming completion.
- When preparing commits, you MUST follow the conventions defined in `pshub-git-commit-conventions` (if registered as a tool, invoke it; otherwise refer to its documentation).
- You MUST NOT make destructive git or filesystem changes unless explicitly requested by the user.
- You MUST NOT remove historical helpers or code paths unless the task explicitly calls for it and impact is understood.

## Planning And Execution Conventions

### OMX (Local Workflow State)
Use `OMX` as the repository-local workflow spine when the work needs persistent project planning/state.
- You MUST save implementation plans to `.omx/plans/` using your file writing tool.
- You MUST save runtime/session state to `.omx/state/`.
- Treat `.omx/` as the project's visible local operating surface.

### superpowers (Process Guidance)
Use `superpowers` as process guidance for how to work (e.g., `brainstorming`, `writing-plans`, `verification-before-completion`). 
- They guide method; they do NOT replace the repository's preferred `.omx/` storage locations unless the user explicitly asks otherwise.

### openspec (Formal Specifications)
Use `openspec` when the work should become a formal change proposal or long-lived specification (e.g., multi-step architectural changes, features needing `proposal/design/tasks/spec`).

### Working Together & Synchronization
`OMX`, `superpowers`, and `openspec` MAY be used together.
- `superpowers` decide the working style.
- `OMX` stores local plan/state for this repository.
- `openspec` stores formal change/spec artifacts.

**Sync from `OMX` to `openspec` is REQUIRED when:**
- The change crosses multiple subsystems or projects.
- The change introduces or revises architecture, interfaces, or extension points.
- The plan is considered "final" for execution and is expected to survive beyond a single short session.
- The change affects team-facing conventions, verification strategy, or rollout expectations.
- The user explicitly asks for formal specs.

When sync is required, you MUST:
1. Create or update the relevant `openspec` change.
2. Carry over the final intent, scope, acceptance criteria, key risks, and verification approach.
3. Keep `.omx/plans/` as the execution-oriented version and `openspec` as the formal reference.

**Lightweight changes DO NOT require `openspec` sync if** they are small, local, reversible, and not introducing new architecture.

## Architecture Expectations

- Respect each subproject's existing architecture.
- For `Desktop/PsHub.OscilloScope`, you MUST preserve the current `MVVM + IOC + DI` structure.
- You MUST prefer injected services over static helpers when adding replaceable behavior.
- Keep rendering/data-processing concerns separate from transport/protocol concerns.
- For performance work, fix the narrowest proven bottleneck first before introducing broader architectural layers.

## Verification

- For code changes, run the narrowest meaningful verification first, then broaden if risk requires it.
- After each commit, you MUST check whether the related `.omx/plans/` documents need status sync, completed checkboxes, or an "implemented" note.
- For planning-only work, the plan MUST include: Scope, Acceptance Criteria, Risks, Verification Approach.

## Priority

1. Direct user instructions ALWAYS win.
2. Deeper `AGENTS.md` files override this file for their subtree if they are added later.
