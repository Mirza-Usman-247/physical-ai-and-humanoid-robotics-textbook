"""
JWT token utilities for Better Auth integration.
Handles JWT access and refresh token creation/verification.
"""

from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from jose import JWTError, jwt
from src.config import settings


def create_access_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Payload data to encode (typically {sub: user_id, email: user_email})
        expires_delta: Optional custom expiration time (default: 15 minutes from settings)

    Returns:
        Encoded JWT access token string

    Example:
        >>> token = create_access_token({"sub": "user-uuid-here", "email": "user@example.com"})

    Security Note:
        Only 'sub' and 'email' fields are included in the token.
        Sensitive fields (password, password_hash, etc.) are filtered out for security.
    """
    # Only include safe fields in token (security measure)
    safe_fields = {'sub', 'email'}
    to_encode = {k: v for k, v in data.items() if k in safe_fields}

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.jwt_access_token_expire_minutes)

    to_encode.update({"exp": expire, "type": "access"})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)
    return encoded_jwt


def create_refresh_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT refresh token with longer expiration.

    Args:
        data: Payload data to encode (typically {sub: user_id})
        expires_delta: Optional custom expiration time (default: 7 days from settings)

    Returns:
        Encoded JWT refresh token string

    Example:
        >>> token = create_refresh_token({"sub": "user-uuid-here"})

    Security Note:
        Only 'sub' and 'email' fields are included in the token.
        Sensitive fields (password, password_hash, etc.) are filtered out for security.
    """
    # Only include safe fields in token (security measure)
    safe_fields = {'sub', 'email'}
    to_encode = {k: v for k, v in data.items() if k in safe_fields}

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=settings.jwt_refresh_token_expire_days)

    to_encode.update({"exp": expire, "type": "refresh"})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)
    return encoded_jwt


def verify_token(token: str, expected_type: str = "access") -> Optional[Dict[str, Any]]:
    """
    Verify and decode a JWT token.

    Args:
        token: JWT token string to verify
        expected_type: Expected token type ("access" or "refresh")

    Returns:
        Decoded token payload if valid, None if invalid/expired

    Raises:
        JWTError: If token is malformed or signature verification fails

    Example:
        >>> payload = verify_token("eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...")
        >>> if payload:
        >>>     user_id = payload.get("sub")
    """
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        token_type = payload.get("type")

        if token_type != expected_type:
            return None  # Wrong token type (e.g., refresh token used where access token expected)

        return payload
    except JWTError:
        return None  # Token expired, malformed, or invalid signature


def decode_token_unsafe(token: str) -> Optional[Dict[str, Any]]:
    """
    Decode a JWT token WITHOUT verification (for debugging or extracting expired token data).

    WARNING: Do NOT use for authentication! Only use for debugging/logging purposes.

    Args:
        token: JWT token string to decode

    Returns:
        Decoded token payload (unverified), None if malformed

    Example:
        >>> payload = decode_token_unsafe("expired-token-here")
        >>> print(f"Token was for user: {payload.get('sub')}")
    """
    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret_key,
            algorithms=[settings.jwt_algorithm],
            options={"verify_signature": False, "verify_exp": False}
        )
        return payload
    except JWTError:
        return None
