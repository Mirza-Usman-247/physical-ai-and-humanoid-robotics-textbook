"""
JWT authentication middleware for FastAPI.
Provides dependency injection for protected routes.
"""

from typing import Optional
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from src.utils.jwt import verify_token
from src.db.models import AuthUser
from src.db.connection import get_db


# HTTP Bearer security scheme (extracts "Authorization: Bearer <token>" header)
security = HTTPBearer()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> AuthUser:
    """
    FastAPI dependency to get the current authenticated user from JWT token.

    Usage:
        @app.get("/api/profile")
        async def get_profile(current_user: AuthUser = Depends(get_current_user)):
            return {"user_id": str(current_user.id), "email": current_user.email}

    Args:
        credentials: HTTP Bearer credentials (automatically extracted from Authorization header)
        db: Database session (dependency injected)

    Returns:
        AuthUser: Authenticated user object from database

    Raises:
        HTTPException 401: If token is missing, invalid, or user not found
        HTTPException 403: If user account is inactive

    Example request headers:
        Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    # Extract token from credentials
    token = credentials.credentials

    # Verify and decode token
    payload = verify_token(token, expected_type="access")
    if payload is None:
        raise credentials_exception

    # Extract user_id from token payload
    user_id: str = payload.get("sub")
    if user_id is None:
        raise credentials_exception

    # Fetch user from database
    user = db.query(AuthUser).filter(AuthUser.id == user_id).first()
    if user is None:
        raise credentials_exception

    # Check if user account is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="User account is inactive"
        )

    return user


async def get_current_user_optional(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(HTTPBearer(auto_error=False)),
    db: Session = Depends(get_db)
) -> Optional[AuthUser]:
    """
    FastAPI dependency to optionally get the current authenticated user.

    Returns None if no token is provided or token is invalid (no exception raised).
    Useful for endpoints that work differently for authenticated vs anonymous users.

    Usage:
        @app.get("/api/content")
        async def get_content(user: Optional[AuthUser] = Depends(get_current_user_optional)):
            if user:
                return {"personalized": True, "user_id": str(user.id)}
            else:
                return {"personalized": False, "content": "generic content"}

    Args:
        credentials: Optional HTTP Bearer credentials
        db: Database session (dependency injected)

    Returns:
        Optional[AuthUser]: Authenticated user object or None if not authenticated
    """
    if credentials is None:
        return None

    try:
        token = credentials.credentials
        payload = verify_token(token, expected_type="access")
        if payload is None:
            return None

        user_id: str = payload.get("sub")
        if user_id is None:
            return None

        user = db.query(AuthUser).filter(AuthUser.id == user_id).first()
        if user is None or not user.is_active:
            return None

        return user
    except Exception:
        return None  # Silently fail for optional authentication
