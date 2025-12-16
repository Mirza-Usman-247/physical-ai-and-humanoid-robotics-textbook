"""
Rate limiting middleware to prevent abuse.
Implements 20 requests per minute per session as per FR-030.
"""

from typing import Dict, Tuple
from datetime import datetime, timedelta
from fastapi import Request, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
import logging

from src.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class RateLimiter:
    """
    In-memory rate limiter using sliding window algorithm.

    Attributes:
        requests: Dictionary tracking requests per session
        limit: Maximum requests allowed per minute
        window: Time window in seconds
    """

    def __init__(self, limit: int = 20, window: int = 60):
        """
        Initialize rate limiter.

        Args:
            limit: Maximum requests per window (default: 20)
            window: Time window in seconds (default: 60)
        """
        self.requests: Dict[str, list[datetime]] = {}
        self.limit = limit
        self.window = timedelta(seconds=window)

    def is_allowed(self, session_id: str) -> Tuple[bool, int]:
        """
        Check if request from session is allowed.

        Args:
            session_id: Session identifier

        Returns:
            Tuple[bool, int]: (is_allowed, remaining_requests)
        """
        now = datetime.utcnow()

        # Initialize session if not exists
        if session_id not in self.requests:
            self.requests[session_id] = []

        # Remove requests outside the time window
        self.requests[session_id] = [
            req_time for req_time in self.requests[session_id]
            if now - req_time < self.window
        ]

        # Check if limit exceeded
        request_count = len(self.requests[session_id])
        if request_count >= self.limit:
            return False, 0

        # Add current request
        self.requests[session_id].append(now)

        remaining = self.limit - (request_count + 1)
        return True, remaining

    def cleanup_old_sessions(self, max_age_hours: int = 24):
        """
        Cleanup old session data to prevent memory bloat.

        Args:
            max_age_hours: Maximum age of session data to keep (default: 24)
        """
        now = datetime.utcnow()
        cutoff = now - timedelta(hours=max_age_hours)

        sessions_to_remove = [
            session_id for session_id, requests in self.requests.items()
            if not requests or max(requests) < cutoff
        ]

        for session_id in sessions_to_remove:
            del self.requests[session_id]

        if sessions_to_remove:
            logger.info(f"Cleaned up {len(sessions_to_remove)} old rate limit sessions")


# Global rate limiter instance
rate_limiter = RateLimiter(
    limit=settings.rate_limit_per_minute,
    window=60
)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    FastAPI middleware for rate limiting.

    Applies rate limiting to all API endpoints based on session ID.
    """

    async def dispatch(self, request: Request, call_next):
        """
        Process request and apply rate limiting.

        Args:
            request: Incoming HTTP request
            call_next: Next middleware/route handler

        Returns:
            Response or raises HTTPException if rate limit exceeded
        """
        # Skip rate limiting for health check endpoint
        if request.url.path == "/health":
            return await call_next(request)

        # Extract session ID from headers or cookies
        session_id = (
            request.headers.get("X-Session-ID") or
            request.cookies.get("session_id") or
            request.client.host  # Fallback to IP address
        )

        # Check rate limit
        is_allowed, remaining = rate_limiter.is_allowed(session_id)

        if not is_allowed:
            logger.warning(f"Rate limit exceeded for session: {session_id}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "rate_limit_exceeded",
                    "message": "Too many requests. Please try again in a moment.",
                    "retry_after_seconds": 60
                }
            )

        # Add rate limit headers to response
        response = await call_next(request)
        response.headers["X-RateLimit-Limit"] = str(settings.rate_limit_per_minute)
        response.headers["X-RateLimit-Remaining"] = str(remaining)
        response.headers["X-RateLimit-Reset"] = str(60)  # Seconds until window resets

        return response


def get_rate_limiter() -> RateLimiter:
    """
    Get the global rate limiter instance.

    Returns:
        RateLimiter: Configured rate limiter

    Example:
        >>> from src.api.middleware.rate_limit import get_rate_limiter
        >>> limiter = get_rate_limiter()
        >>> is_allowed, remaining = limiter.is_allowed("session-123")
        >>> print(is_allowed, remaining)
    """
    return rate_limiter
