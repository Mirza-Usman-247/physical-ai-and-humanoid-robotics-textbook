# ADR-005: Authentication Architecture

> **Scope**: Document decision cluster for authentication including library choice, token strategy, and security architecture.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** auth-personalization-translation
- **Context:** Need to implement user authentication for the Physical AI & Humanoid Robotics textbook platform to enable personalized content and translation features. Requirements include: (1) email/password authentication, (2) user profile storage (skill levels and hardware access), (3) JWT token validation for protected APIs, (4) integration with existing FastAPI backend and Docusaurus frontend, (5) secure password storage and session management, (6) future-proof for potential OAuth/SSO integration. Constraints: minimize maintenance burden, avoid vendor lock-in, maintain GitHub Pages compatibility.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects security posture, user data management, API access control, future scalability
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Better Auth vs Auth0/Clerk vs Custom vs Firebase Auth vs Passport.js
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts frontend, backend, database, all protected API endpoints
-->

## Decision

**Adopt the following integrated authentication architecture:**

- **Frontend Auth Library**: Better Auth (https://www.better-auth.com/)
- **Token Strategy**: JWT tokens with short-lived access tokens (15-minute expiry) and long-lived refresh tokens (7-day expiry with rotation)
- **Backend Validation**: FastAPI dependency injection with `python-jose[cryptography]` for JWT verification
- **Password Security**: bcrypt hashing with cost factor 12 (2^12 iterations)
- **User Storage**: Neon Serverless Postgres with `users` and `user_profiles` tables
- **Session Management**: Stateless authentication (no server-side sessions)
- **Security Hardening**: Rate limiting (100 requests/15 min per user), CORS restrictions, HTTPS-only in production

**Architecture Pattern**: Frontend (Better Auth) issues JWT → Backend (FastAPI) validates JWT on each protected request → No session synchronization required (stateless)

## Consequences

### Positive

- **Better Auth**: Modern, lightweight, React-first library with TypeScript support; designed for SPA/SSR apps like Docusaurus; built-in email/password with secure defaults; extensible for future OAuth providers
- **JWT Stateless**: No backend session storage required → scales horizontally; tokens verifiable cryptographically without database lookup; reduces backend memory footprint
- **Short-lived Access Tokens**: 15-minute expiry limits damage from token theft; refresh token rotation provides security with UX convenience
- **bcrypt Cost Factor 12**: Industry-standard password hashing; 2^12 iterations balances security (computationally expensive for attackers) with performance (~200ms login time acceptable)
- **FastAPI Native Support**: Excellent OAuth2/JWT integration via dependency injection; automatic OpenAPI docs for auth endpoints; type-safe Pydantic models for validation
- **No Vendor Lock-in**: Better Auth is self-hosted (not SaaS); full control over auth flow; can migrate to different library if needed
- **GitHub Pages Compatible**: Authentication happens client-side (Better Auth) + API calls to backend; no serverless function requirements; static site remains deployable to GitHub Pages

### Negative

- **Better Auth Learning Curve**: Newer library (less mature than Passport.js); smaller community compared to Auth0/Clerk; requires understanding JWT flow
- **Token Management Complexity**: Frontend must handle token refresh logic, expiry detection, and secure storage (memory vs localStorage debate)
- **No Built-in 2FA**: Better Auth requires custom implementation for two-factor authentication (out of scope for MVP, but future consideration)
- **JWT Size**: Tokens can be 500-1000 bytes (larger than session IDs); included in every API request header → slight bandwidth overhead
- **Token Revocation Challenge**: Stateless JWTs cannot be revoked server-side before expiry; requires Redis blacklist or short expiry + refresh rotation (chosen: short expiry)
- **Better Auth + FastAPI Coordination**: Two separate auth systems (frontend Better Auth, backend JWT validation) must stay synchronized on secret key and algorithm

## Alternatives Considered

### Alternative 1: Auth0 (SaaS Authentication Service)

**Components**:
- Auth0 Universal Login (hosted UI)
- Auth0 Management API for user profiles
- Auth0 tokens (JWT) validated by FastAPI

**Tradeoffs**:
- ✅ Production-ready with 2FA, social login, enterprise SSO out-of-box
- ✅ Managed security (Auth0 handles password hashing, token management, compliance)
- ✅ Generous free tier (7,000 active users/month)
- ❌ Vendor lock-in (migrating off Auth0 requires rewriting entire auth flow)
- ❌ External dependency (Auth0 outage blocks all user access)
- ❌ Cost scaling ($25/month for 10K+ users, $228/month for 50K users)
- ❌ Overkill for simple email/password auth (90% of features unused)

**Rejection Rationale**: Over-engineered for MVP. Better Auth provides equivalent email/password functionality at zero cost with full control. Can migrate to Auth0 later if enterprise SSO becomes a requirement.

### Alternative 2: Custom Authentication Implementation

**Components**:
- Manual JWT generation with `python-jose`
- Custom password hashing with `passlib[bcrypt]`
- Hand-coded login/signup endpoints in FastAPI
- Manual token refresh logic

**Tradeoffs**:
- ✅ Full control over every aspect of auth flow
- ✅ No external library dependencies (frontend or backend)
- ✅ Minimal code footprint (only what we need)
- ❌ Security risk (easy to make mistakes with crypto, token validation, CSRF protection)
- ❌ Reinventing the wheel (Better Auth already provides battle-tested implementation)
- ❌ Maintenance burden (must stay up-to-date with security best practices ourselves)
- ❌ No TypeScript types or tooling (manual API client code)

**Rejection Rationale**: High risk, low reward. Authentication is critical security infrastructure—using a well-tested library (Better Auth) reduces attack surface and development time. Custom auth only justified for highly specialized requirements (not applicable here).

### Alternative 3: Firebase Authentication

**Components**:
- Firebase Auth SDK (frontend)
- Firebase Admin SDK (backend token verification)
- Firebase Cloud Firestore (user profiles)

**Tradeoffs**:
- ✅ Mature ecosystem with extensive documentation
- ✅ Built-in social login (Google, Facebook, Twitter, etc.)
- ✅ Generous free tier (50K monthly active users)
- ✅ Real-time database sync for user profiles (Firestore)
- ❌ Vendor lock-in to Google Cloud Platform
- ❌ Firestore not suitable for relational user profiles (prefer PostgreSQL schema)
- ❌ Requires Firebase SDK (~100KB bundle size) → slower page loads
- ❌ Firestore cold start latency (~500ms) worse than Neon Postgres
- ❌ Email/password flow requires extra setup (not as streamlined as Better Auth)

**Rejection Rationale**: Firestore mismatch with Neon Postgres requirement (spec specifies Neon for user profiles). Firebase SDK adds unnecessary bundle size for features we don't need. Better Auth is lighter weight and PostgreSQL-first.

### Alternative 4: Passport.js (Node.js Auth Middleware)

**Components**:
- Passport.js with passport-local strategy (email/password)
- Express.js middleware for session management
- PostgreSQL session store (connect-pg-simple)

**Tradeoffs**:
- ✅ Extremely mature (industry standard for Node.js auth)
- ✅ 500+ authentication strategies (OAuth, SAML, OpenID, etc.)
- ✅ Large community and extensive tutorials
- ❌ Node.js-centric (not designed for React SPAs like Docusaurus)
- ❌ Session-based by default (requires backend session store → not stateless)
- ❌ Heavier weight than Better Auth (designed for server-rendered apps, not SPAs)
- ❌ Less TypeScript-friendly than Better Auth

**Rejection Rationale**: Better Auth is purpose-built for React SPAs and modern frontend frameworks. Passport.js excels in server-rendered apps but is overkill for Docusaurus. JWT stateless pattern (Better Auth) is cleaner than session management (Passport.js).

## References

- Feature Spec: [specs/003-auth-personalization-translation/spec.md](../../specs/003-auth-personalization-translation/spec.md) (FR-002, FR-004, FR-022, FR-023, FR-024)
- Implementation Plan: [specs/003-auth-personalization-translation/plan.md](../../specs/003-auth-personalization-translation/plan.md) (Technical Context, Security)
- Research: [specs/003-auth-personalization-translation/research.md](../../specs/003-auth-personalization-translation/research.md) (Section 1: Authentication Architecture, Section 6: Security Best Practices)
- Data Model: [specs/003-auth-personalization-translation/data-model.md](../../specs/003-auth-personalization-translation/data-model.md) (Users table, UserProfiles table)
- Related ADRs: ADR-002 (Technology Stack Selection - FastAPI and Neon Postgres)
- Success Criteria: SC-001 (registration <3 minutes), SC-010 (99% auth success rate), SC-011 (100% access control enforcement)
- Better Auth Documentation: https://www.better-auth.com/docs
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
- JWT Best Practices: https://tools.ietf.org/html/rfc8725
