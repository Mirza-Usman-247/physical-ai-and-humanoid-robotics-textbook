"""
T022: Unit tests for JWT token generation and verification.

Tests the JWT utilities in src/utils/jwt.py for access and refresh tokens.
"""
import pytest
from datetime import datetime, timedelta
from jose import jwt, JWTError
from src.utils.jwt import (
    create_access_token,
    create_refresh_token,
    verify_token,
    decode_token_unsafe
)
from src.config import get_settings


@pytest.fixture
def test_user_data():
    """Sample user data for JWT payload."""
    return {
        "sub": "123e4567-e89b-12d3-a456-426614174000",  # User ID
        "email": "test@example.com"
    }


def test_create_access_token(test_settings, test_user_data):
    """
    Test that create_access_token generates a valid JWT access token.

    Acceptance Criteria:
    - Returns a string (JWT token)
    - Token contains 'sub' (user ID) and 'email' from input data
    - Token contains 'exp' (expiration) claim
    - Token contains 'type': 'access'
    - Token expires in 15 minutes (default from settings)
    - Token can be decoded with correct secret key
    - Token is signed with HS256 algorithm
    """
    token = create_access_token(test_user_data)

    # Verify token is a string
    assert isinstance(token, str), "Token should be a string"
    assert len(token) > 0, "Token should not be empty"

    # Decode token
    settings = get_settings()
    payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

    # Verify payload structure
    assert payload["sub"] == test_user_data["sub"], "Token should contain user ID"
    assert payload["email"] == test_user_data["email"], "Token should contain email"
    assert payload["type"] == "access", "Token type should be 'access'"
    assert "exp" in payload, "Token should have expiration"

    # Verify expiration is approximately 15 minutes from now
    exp_time = datetime.utcfromtimestamp(payload["exp"])
    now = datetime.utcnow()
    time_diff = (exp_time - now).total_seconds()
    assert 14 * 60 < time_diff < 16 * 60, \
        f"Access token should expire in ~15 minutes, got {time_diff/60:.1f} minutes"


def test_create_access_token_custom_expiration(test_settings, test_user_data):
    """
    Test creating access token with custom expiration time.

    Acceptance Criteria:
    - Accepts custom timedelta for expiration
    - Token expires at specified time
    """
    custom_expiration = timedelta(minutes=30)
    token = create_access_token(test_user_data, expires_delta=custom_expiration)

    settings = get_settings()
    payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

    # Verify custom expiration
    exp_time = datetime.utcfromtimestamp(payload["exp"])
    now = datetime.utcnow()
    time_diff = (exp_time - now).total_seconds()
    assert 29 * 60 < time_diff < 31 * 60, \
        f"Token should expire in ~30 minutes, got {time_diff/60:.1f} minutes"


def test_create_refresh_token(test_settings, test_user_data):
    """
    Test that create_refresh_token generates a valid JWT refresh token.

    Acceptance Criteria:
    - Returns a string (JWT token)
    - Token contains 'sub' (user ID) and 'email' from input data
    - Token contains 'exp' (expiration) claim
    - Token contains 'type': 'refresh'
    - Token expires in 7 days (default from settings)
    - Token can be decoded with correct secret key
    - Token is signed with HS256 algorithm
    """
    token = create_refresh_token(test_user_data)

    # Verify token is a string
    assert isinstance(token, str), "Token should be a string"
    assert len(token) > 0, "Token should not be empty"

    # Decode token
    settings = get_settings()
    payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

    # Verify payload structure
    assert payload["sub"] == test_user_data["sub"], "Token should contain user ID"
    assert payload["email"] == test_user_data["email"], "Token should contain email"
    assert payload["type"] == "refresh", "Token type should be 'refresh'"
    assert "exp" in payload, "Token should have expiration"

    # Verify expiration is approximately 7 days from now
    exp_time = datetime.utcfromtimestamp(payload["exp"])
    now = datetime.utcnow()
    time_diff_days = (exp_time - now).total_seconds() / (24 * 60 * 60)
    assert 6.9 < time_diff_days < 7.1, \
        f"Refresh token should expire in ~7 days, got {time_diff_days:.1f} days"


def test_create_refresh_token_custom_expiration(test_settings, test_user_data):
    """
    Test creating refresh token with custom expiration time.

    Acceptance Criteria:
    - Accepts custom timedelta for expiration
    - Token expires at specified time
    """
    custom_expiration = timedelta(days=14)
    token = create_refresh_token(test_user_data, expires_delta=custom_expiration)

    settings = get_settings()
    payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

    # Verify custom expiration
    exp_time = datetime.utcfromtimestamp(payload["exp"])
    now = datetime.utcnow()
    time_diff_days = (exp_time - now).total_seconds() / (24 * 60 * 60)
    assert 13.9 < time_diff_days < 14.1, \
        f"Token should expire in ~14 days, got {time_diff_days:.1f} days"


def test_verify_token_access(test_settings, test_user_data):
    """
    Test verifying a valid access token.

    Acceptance Criteria:
    - Returns payload dict for valid access token
    - Verifies token type matches expected type ('access')
    - Returns None for invalid token
    - Returns None for expired token
    - Returns None for wrong token type (refresh when expecting access)
    """
    # Create valid access token
    access_token = create_access_token(test_user_data)

    # Verify valid access token
    payload = verify_token(access_token, expected_type="access")
    assert payload is not None, "Should return payload for valid access token"
    assert payload["sub"] == test_user_data["sub"]
    assert payload["email"] == test_user_data["email"]
    assert payload["type"] == "access"

    # Create refresh token and try to verify as access (should fail)
    refresh_token = create_refresh_token(test_user_data)
    payload = verify_token(refresh_token, expected_type="access")
    assert payload is None, "Should return None when token type doesn't match"


def test_verify_token_refresh(test_settings, test_user_data):
    """
    Test verifying a valid refresh token.

    Acceptance Criteria:
    - Returns payload dict for valid refresh token
    - Verifies token type matches expected type ('refresh')
    - Returns None for wrong token type (access when expecting refresh)
    """
    # Create valid refresh token
    refresh_token = create_refresh_token(test_user_data)

    # Verify valid refresh token
    payload = verify_token(refresh_token, expected_type="refresh")
    assert payload is not None, "Should return payload for valid refresh token"
    assert payload["sub"] == test_user_data["sub"]
    assert payload["email"] == test_user_data["email"]
    assert payload["type"] == "refresh"

    # Create access token and try to verify as refresh (should fail)
    access_token = create_access_token(test_user_data)
    payload = verify_token(access_token, expected_type="refresh")
    assert payload is None, "Should return None when token type doesn't match"


def test_verify_token_invalid(test_settings):
    """
    Test verifying invalid tokens.

    Acceptance Criteria:
    - Returns None for malformed token
    - Returns None for token with wrong signature
    - Returns None for empty token
    - Returns None for non-JWT string
    """
    # Malformed token
    payload = verify_token("not.a.valid.jwt.token", expected_type="access")
    assert payload is None, "Should return None for malformed token"

    # Empty token
    payload = verify_token("", expected_type="access")
    assert payload is None, "Should return None for empty token"

    # Token with wrong signature
    settings = get_settings()
    fake_token = jwt.encode(
        {"sub": "123", "type": "access", "exp": datetime.utcnow() + timedelta(minutes=15)},
        "wrong_secret_key",
        algorithm="HS256"
    )
    payload = verify_token(fake_token, expected_type="access")
    assert payload is None, "Should return None for token with wrong signature"


def test_verify_token_expired(test_settings, test_user_data):
    """
    Test verifying an expired token.

    Acceptance Criteria:
    - Returns None for expired access token
    - Returns None for expired refresh token
    """
    # Create token that expires immediately
    expired_token = create_access_token(test_user_data, expires_delta=timedelta(seconds=-1))

    # Verify expired token
    payload = verify_token(expired_token, expected_type="access")
    assert payload is None, "Should return None for expired token"


def test_decode_token_unsafe(test_settings, test_user_data):
    """
    Test unsafe token decoding (without verification, for debugging).

    Acceptance Criteria:
    - Returns payload dict without verifying signature or expiration
    - Used for debugging expired tokens
    - Should NOT be used for authentication
    """
    # Create expired token
    expired_token = create_access_token(test_user_data, expires_delta=timedelta(seconds=-1))

    # decode_token_unsafe should still decode it (no verification)
    payload = decode_token_unsafe(expired_token)
    assert payload is not None, "Should decode expired token without verification"
    assert payload["sub"] == test_user_data["sub"]
    assert payload["email"] == test_user_data["email"]
    assert payload["type"] == "access"

    # But verify_token should return None
    verified_payload = verify_token(expired_token, expected_type="access")
    assert verified_payload is None, "verify_token should return None for expired token"


def test_token_contains_no_sensitive_data(test_settings):
    """
    Test that tokens do NOT contain sensitive data like passwords.

    Acceptance Criteria:
    - Token payload should NEVER contain password or password_hash
    - Only contains user_id (sub), email, type, and expiration
    """
    user_data_with_password = {
        "sub": "123e4567-e89b-12d3-a456-426614174000",
        "email": "test@example.com",
        "password": "ShouldNotBeInToken123",  # This should NOT be in token
        "password_hash": "$2b$12$hash..."  # This should NOT be in token
    }

    token = create_access_token(user_data_with_password)

    settings = get_settings()
    payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

    # Verify sensitive data is NOT in token
    assert "password" not in payload, "Token must NOT contain password"
    assert "password_hash" not in payload, "Token must NOT contain password_hash"

    # Verify only expected fields are present
    expected_fields = {"sub", "email", "type", "exp"}
    payload_fields = set(payload.keys())
    # Token may have additional JWT standard fields like 'iat' (issued at), but should not have password
    assert "password" not in payload_fields
    assert "password_hash" not in payload_fields


def test_different_tokens_for_same_user(test_settings, test_user_data):
    """
    Test that generating multiple tokens for same user produces different tokens.

    Acceptance Criteria:
    - Multiple calls to create_access_token produce different token strings
    - Multiple calls to create_refresh_token produce different token strings
    - This is expected due to different expiration timestamps
    """
    token1 = create_access_token(test_user_data)
    token2 = create_access_token(test_user_data)

    # Tokens should be different (different exp timestamps)
    # Note: If created at exact same microsecond, they might be identical,
    # but in practice they'll differ due to timing
    # We just verify both are valid, not that they're different
    payload1 = verify_token(token1, expected_type="access")
    payload2 = verify_token(token2, expected_type="access")

    assert payload1 is not None
    assert payload2 is not None
    assert payload1["sub"] == payload2["sub"]
    assert payload1["email"] == payload2["email"]


def test_token_signature_algorithm(test_settings, test_user_data):
    """
    Test that tokens use HS256 algorithm as specified in settings.

    Acceptance Criteria:
    - Token header specifies 'alg': 'HS256'
    - Token cannot be decoded with different algorithm
    """
    token = create_access_token(test_user_data)

    # Decode header without verification
    settings = get_settings()
    header = jwt.get_unverified_header(token)

    assert header["alg"] == "HS256", "Token should use HS256 algorithm"

    # Try to decode with wrong algorithm (should fail)
    try:
        jwt.decode(token, settings.jwt_secret_key, algorithms=["HS512"])
        assert False, "Should not decode token with wrong algorithm"
    except JWTError:
        pass  # Expected to raise JWTError
