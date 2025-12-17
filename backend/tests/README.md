# Backend Test Suite

## Overview

Comprehensive test suite for the authentication and authorization system following TDD (Test-Driven Development) methodology.

## Test Structure

```
backend/tests/
├── conftest.py           # Pytest fixtures and configuration
├── api/                  # API endpoint contract tests
│   └── test_auth.py      # T019: Auth endpoint tests
├── integration/          # End-to-end integration tests
│   └── test_auth_flow.py # T020: Full signup flow tests
└── utils/                # Unit tests for utilities
    ├── test_password.py  # T021: Password hashing tests
    └── test_jwt.py       # T022: JWT token tests
```

## Test Results (Phase 2 Red - 2025-12-17)

### ✅ T021: Password Hashing Tests (7/7 PASS)
- `test_hash_password` - Bcrypt hash generation
- `test_verify_password` - Password verification
- `test_verify_password_with_different_hashes` - Cross-verification
- `test_needs_rehash` - Hash migration detection
- `test_hash_password_special_characters` - Unicode/special char support
- `test_hash_password_edge_cases` - Min/max length passwords
- `test_password_hash_timing_resistance` - Constant-time comparison

**Status**: ✅ All tests pass. Password utilities are production-ready.

### ✅ T022: JWT Token Tests (11/12 PASS, 1 Expected Failure)
- `test_create_access_token` - 15-minute access token generation
- `test_create_access_token_custom_expiration` - Custom expiry
- `test_create_refresh_token` - 7-day refresh token generation
- `test_create_refresh_token_custom_expiration` - Custom expiry
- `test_verify_token_access` - Access token verification
- `test_verify_token_refresh` - Refresh token verification
- `test_verify_token_invalid` - Invalid token rejection
- `test_verify_token_expired` - Expired token rejection
- `test_decode_token_unsafe` - Debug decoding
- `test_different_tokens_for_same_user` - Token uniqueness
- `test_token_signature_algorithm` - HS256 algorithm verification
- `test_token_contains_no_sensitive_data` ⚠️ **EXPECTED FAILURE** - Security vulnerability detected (passwords included in token payload)

**Status**: ✅ 11/12 pass. One expected failure identifies security issue to fix in Green phase (filter sensitive fields).

### 📝 T019: API Contract Tests (Written, PostgreSQL Required)
- `test_signup_success` - Valid signup returns 201 with user + tokens
- `test_signup_duplicate_email` - Duplicate email returns 409
- `test_signup_invalid_password` - Password validation (uppercase, lowercase, number, min 8 chars)
- `test_signup_invalid_skill_levels` - Skill level range validation (1-5)
- `test_signup_missing_required_fields` - Required field validation

**Status**: 📝 Tests written. Require PostgreSQL or constraint simplification (SQLite doesn't support `~*` regex operator and `gen_random_uuid()`).

### 📝 T020: Integration Tests (Written, PostgreSQL Required)
- `test_signup_creates_user_and_profile` - Full database persistence verification
- `test_signup_cloud_only_computation` - Computed column verification
- `test_signup_password_verification` - Bcrypt hash verification

**Status**: 📝 Tests written. Require PostgreSQL for full testing.

## Running Tests

### Run All Tests
```bash
cd backend
python -m pytest tests/ -v
```

### Run Specific Test Suites
```bash
# Password tests (T021)
python -m pytest tests/utils/test_password.py -v

# JWT tests (T022)
python -m pytest tests/utils/test_jwt.py -v

# API contract tests (T019) - Requires PostgreSQL
python -m pytest tests/api/test_auth.py -v

# Integration tests (T020) - Requires PostgreSQL
python -m pytest tests/integration/test_auth_flow.py -v
```

### Run with Coverage
```bash
python -m pytest tests/ --cov=src --cov-report=html
```

## Database Configuration

### SQLite (In-Memory) - Default for Fast Unit Tests
- Used for password and JWT utility tests
- Fast, no setup required
- Limited: No PostgreSQL-specific features (UUID, regex constraints, computed columns)

### PostgreSQL - Required for API/Integration Tests
To run full test suite with PostgreSQL:
1. Set `TEST_DATABASE_URL` environment variable
2. Update `conftest.py` to use PostgreSQL instead of SQLite
3. Run migrations: `alembic upgrade head`

## Known Issues

1. **SQLite UUID Compatibility**: Custom UUID TypeDecorator added to conftest.py for SQLite compatibility
2. **Regex Constraints**: PostgreSQL `~*` operator not supported in SQLite
3. **Computed Columns**: Requires PostgreSQL for `cloud_only` computed column
4. **JWT Security**: One test deliberately fails to highlight security vulnerability (sensitive data in tokens) - to be fixed in Green phase

## Next Steps (Phase 2 Green)

1. **T023-T031**: Implement backend auth endpoints to make tests pass
2. Fix JWT security issue (filter sensitive fields from token payload)
3. Consider PostgreSQL test database for full integration testing
4. Add performance benchmarks for password hashing (target: <200ms)
5. Add load testing for JWT token generation (target: >1000 tokens/sec)

## Test Coverage Goals

- Unit Tests: 90%+ coverage
- Integration Tests: 80%+ coverage
- API Contract Tests: 100% endpoint coverage
- Security Tests: 100% vulnerability coverage

---

**Last Updated**: 2025-12-17
**Phase**: Phase 2 Red (Tests Written)
**Status**: ✅ Red Phase Complete - Ready for Green Phase Implementation
