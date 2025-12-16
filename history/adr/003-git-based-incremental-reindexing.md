# ADR-003: Git-Based Incremental Re-indexing Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** rag-chatbot-integration
- **Context:** Textbook content evolves continuously (typo fixes, new chapters, updated explanations). Need to keep Qdrant vector database synchronized with latest content without manual intervention. Challenges include: (1) detecting which files changed, (2) avoiding full re-indexing overhead (~250k tokens = ~$0.05 per full re-index), (3) ensuring vector DB doesn't serve stale content, (4) tracking content version for debugging. Must support 21 chapters + 5 module indexes + 6 docs (34 files total) with ~700-900 chunks.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects data consistency, DevOps workflow, maintenance burden
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Full re-index, timestamp-based, manual trigger, webhook-based
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts CI/CD pipeline, vector DB schema, debugging workflows
-->

## Decision

**Adopt git-based incremental re-indexing with CI/CD automation:**

**Versioning Mechanism**:
- Store `commit_hash` in Qdrant chunk metadata for every indexed chunk
- Track `file_path`, `line_start`, `line_end` to map chunks to source locations
- Add `last_indexed` timestamp for debugging staleness issues

**Re-indexing Trigger**: GitHub Actions workflow on `push` to `main` branch:
1. Detect changed `.md` files using `git diff HEAD~1 HEAD --name-only`
2. Invoke `rag-indexer` skill for only changed files
3. Generate Qwen embeddings for new/modified chunks
4. Update Qdrant with new `commit_hash` metadata
5. Delete chunks from removed files using `file_path` filter

**Incremental Logic**:
- **Changed file**: Delete old chunks (match `file_path`), index new chunks with new `commit_hash`
- **Deleted file**: Delete all chunks matching `file_path`
- **New file**: Index all chunks with current `commit_hash`

**Staleness Detection**: Frontend warns users if chunk's `last_indexed` timestamp is >7 days old (indicates indexing pipeline failure).

**Manual Fallback**: Admin can trigger full re-index via API endpoint `/admin/reindex?full=true` (requires auth token).

## Consequences

### Positive

- **Cost-Effective**: Only re-index changed files (~5-10 files/commit) instead of all 34 files (reduces embedding costs by 90%)
- **Fast Updates**: Incremental re-indexing completes in <2 minutes vs ~3-5 minutes for full re-index
- **Version Traceability**: `commit_hash` enables debugging "why did chatbot answer change?" by comparing vector DB state across commits
- **Automated Workflow**: No manual intervention required—CI/CD handles re-indexing on every merge to main
- **Consistency Guarantee**: Vector DB always reflects latest `main` branch content (eventual consistency within 2 minutes of merge)
- **Scalability**: Incremental approach scales linearly—adding 100 new chapters doesn't slow down re-indexing for existing chapters

### Negative

- **CI/CD Complexity**: Requires configuring GitHub Actions, managing API keys in secrets, handling workflow failures
- **Git Coupling**: Tightly couples vector DB lifecycle to git workflow—alternative content sources (CMS, external docs) require separate indexing logic
- **Race Conditions**: Rapid commits to `main` (<2 minutes apart) could trigger concurrent re-indexing jobs, risking partial updates
- **Debugging Overhead**: Chunk staleness issues require correlating git commits, CI/CD logs, and Qdrant metadata
- **Cold Start Problem**: Initial setup requires full indexing of all 34 files (~3-5 minutes) before incremental updates work
- **Metadata Bloat**: Storing `commit_hash` (40 chars) for every chunk adds ~50KB storage overhead (negligible but measurable)

## Alternatives Considered

### Alternative 1: Full Re-index on Every Change

**Description**: Trigger full re-indexing of all 34 files whenever any `.md` file changes in `main` branch.

**Tradeoffs**:
- ✅ Simplest implementation—no change detection logic required
- ✅ Guaranteed consistency—vector DB always reflects full content snapshot
- ✅ No risk of partial updates or stale chunks
- ❌ High cost: ~$0.35 per re-index × 50 commits/month = $17.50/month (vs ~$0.50/month incremental)
- ❌ Slow: 3-5 minute re-indexing blocks CI/CD pipeline for every commit
- ❌ Wastes embeddings quota—re-generates vectors for unchanged content

**Rejection Rationale**: Cost and speed unacceptable for MVP. Incremental approach achieves same consistency guarantees with 35x cost savings and 2-3x faster updates.

### Alternative 2: Timestamp-Based Versioning

**Description**: Track `last_modified` timestamp from file system instead of git commit hash. Re-index files where `file.mtime > chunk.last_indexed`.

**Tradeoffs**:
- ✅ Git-agnostic—works with any content source (CMS, Dropbox, etc.)
- ✅ Slightly simpler—no git diff parsing required
- ❌ Timestamps unreliable across environments (local dev, CI/CD, production)
- ❌ No version history—cannot debug "what changed between yesterday and today?"
- ❌ Clock skew issues—file copied from old commit has stale timestamp
- ❌ Difficult to rollback—deleting a file doesn't trigger cleanup (timestamp still exists in chunks)

**Rejection Rationale**: Textbook content is git-managed, so git commit hash provides stronger guarantees. Timestamps fragile for distributed systems (CI/CD workers, multiple developers). Git-based versioning enables auditing and rollback.

### Alternative 3: Manual Trigger Only

**Description**: No automatic re-indexing. Require admin to manually trigger re-indexing via API endpoint or CLI command.

**Tradeoffs**:
- ✅ Full control—admin decides when to re-index (e.g., after major chapter updates)
- ✅ No CI/CD complexity—no GitHub Actions workflow needed
- ✅ Avoids accidental re-indexing for trivial typo fixes
- ❌ Human error—admin forgets to trigger, vector DB becomes stale
- ❌ Poor developer experience—contributors must remember to notify admin after merges
- ❌ Inconsistency risk—vector DB state diverges from `main` branch unpredictably

**Rejection Rationale**: Manual processes don't scale. Automated workflow ensures consistency without human intervention. Admin can still trigger manual full re-index for edge cases.

### Alternative 4: Webhook-Based Real-Time Indexing

**Description**: Set up GitHub webhook to notify backend on every push. Backend immediately triggers re-indexing in background job.

**Tradeoffs**:
- ✅ Real-time updates—vector DB synced within seconds of push
- ✅ No CI/CD workflow dependency—backend handles re-indexing directly
- ❌ Requires backend to be always running (vs CI/CD runners are ephemeral)
- ❌ Webhook security complexity—must validate GitHub signatures, handle replay attacks
- ❌ Backend becomes stateful—needs job queue (Celery, Redis) for background tasks
- ❌ Difficult to debug—webhook failures silent unless backend logging robust

**Rejection Rationale**: Adds significant backend complexity (job queue, webhook auth, monitoring). CI/CD approach simpler—GitHub Actions already handles auth, retries, logging. Real-time updates (seconds vs minutes) not critical for textbook content (updates are infrequent).

## References

- Feature Spec: [specs/002-rag-chatbot-integration/spec.md](../../specs/002-rag-chatbot-integration/spec.md) (FR-029: Git-based versioning)
- Implementation Plan: [specs/002-rag-chatbot-integration/plan.md](../../specs/002-rag-chatbot-integration/plan.md) (Phase 0 Task 0.6, Component Responsibilities)
- Related ADRs: ADR-001 (RAG Modular Skills - rag-indexer skill), ADR-002 (Technology Stack - Qdrant schema)
- Clarification Decision: Clarification Q5 (User chose git-based versioning with CI/CD hooks over alternatives)
- Data Model: Qdrant chunk metadata schema with `commit_hash`, `last_indexed`, `file_path`, `line_start`, `line_end`
- CI/CD Workflow: `.github/workflows/reindex-on-commit.yml` (Phase 2 implementation task)
