"""
Global error handling middleware with user-friendly messages.
Implements FR-031: Return friendly error messages for all failures.
"""

from fastapi import Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from datetime import datetime
import logging
import traceback

from src.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """
    Handle Pydantic validation errors with user-friendly messages.

    Args:
        request: Incoming HTTP request
        exc: Validation exception

    Returns:
        JSONResponse with error details
    """
    errors = exc.errors()
    error_messages = []

    for error in errors:
        field = " -> ".join(str(loc) for loc in error["loc"])
        message = error["msg"]
        error_messages.append(f"{field}: {message}")

    logger.warning(f"Validation error on {request.url.path}: {error_messages}")

    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={
            "error": "validation_error",
            "message": "Invalid input data. Please check your request and try again.",
            "details": {
                "fields": error_messages
            },
            "timestamp": datetime.utcnow().isoformat()
        }
    )


async def http_exception_handler(request: Request, exc: StarletteHTTPException):
    """
    Handle HTTP exceptions with user-friendly messages.

    Args:
        request: Incoming HTTP request
        exc: HTTP exception

    Returns:
        JSONResponse with error details
    """
    # Map status codes to user-friendly messages
    friendly_messages = {
        400: "Invalid request. Please check your input and try again.",
        401: "Authentication required. Please log in to continue.",
        403: "You don't have permission to access this resource.",
        404: "The requested resource was not found.",
        405: "This request method is not allowed for this endpoint.",
        429: "Too many requests. Please slow down and try again in a moment.",
        500: "An internal server error occurred. Our team has been notified.",
        502: "The server is temporarily unavailable. Please try again later.",
        503: "The service is temporarily unavailable. Please try again later.",
        504: "The request timed out. Please try again."
    }

    status_code = exc.status_code
    friendly_message = friendly_messages.get(
        status_code,
        "An error occurred while processing your request."
    )

    # Use custom detail if provided (e.g., from rate limiter)
    if isinstance(exc.detail, dict):
        error_detail = exc.detail
    else:
        error_detail = {
            "error": f"http_{status_code}",
            "message": friendly_message,
            "details": {"original_message": str(exc.detail)} if settings.debug else None,
            "timestamp": datetime.utcnow().isoformat()
        }

    logger.warning(f"HTTP {status_code} on {request.url.path}: {exc.detail}")

    return JSONResponse(
        status_code=status_code,
        content=error_detail
    )


async def general_exception_handler(request: Request, exc: Exception):
    """
    Handle unexpected exceptions with user-friendly messages.

    Args:
        request: Incoming HTTP request
        exc: Exception

    Returns:
        JSONResponse with error details
    """
    # Log full stack trace for debugging
    logger.error(
        f"Unexpected error on {request.url.path}: {str(exc)}",
        exc_info=True,
        extra={
            "path": request.url.path,
            "method": request.method,
            "client": str(request.client.host) if request.client else None
        }
    )

    # Determine if error is related to external services
    error_type = "internal_error"
    friendly_message = "An unexpected error occurred. Our team has been notified and is working on a fix."

    # Detect Qdrant connectivity errors
    if "qdrant" in str(exc).lower() or "vector" in str(exc).lower():
        error_type = "vector_db_error"
        friendly_message = "Unable to search the textbook right now. The vector database is temporarily unavailable. Try rephrasing your question or check back in a moment."

    # Detect OpenRouter API errors
    elif "openrouter" in str(exc).lower() or "openai" in str(exc).lower():
        error_type = "llm_api_error"
        friendly_message = "Unable to generate an answer right now. The AI service is temporarily unavailable. Please try again in a moment."

    # Detect database connectivity errors
    elif "database" in str(exc).lower() or "postgres" in str(exc).lower():
        error_type = "database_error"
        friendly_message = "Unable to save your conversation right now. The database is temporarily unavailable. Your question was received, but we couldn't store the history."

    # Detect token limit errors
    elif "token" in str(exc).lower() or "limit" in str(exc).lower():
        error_type = "token_limit_error"
        friendly_message = "Your query or selected text is too long. Please try a shorter question or select less text."

    error_response = {
        "error": error_type,
        "message": friendly_message,
        "details": {
            "exception_type": type(exc).__name__,
            "stack_trace": traceback.format_exc() if settings.debug else None
        },
        "timestamp": datetime.utcnow().isoformat()
    }

    # Remove debug info in production
    if not settings.debug:
        error_response["details"] = None

    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content=error_response
    )


# Alternative action suggestions for common errors
ALTERNATIVE_ACTIONS = {
    "vector_db_error": [
        "Try rephrasing your question in simpler terms",
        "Ask about a different topic while we fix this issue",
        "Check back in a few minutes - the service should recover soon"
    ],
    "llm_api_error": [
        "Try asking a simpler question",
        "Wait a moment and try again",
        "Check if there are related chapters you can read directly"
    ],
    "token_limit_error": [
        "Try a shorter question (under 500 words)",
        "Select less text from the textbook (under 2000 characters)",
        "Break your question into multiple smaller questions"
    ],
    "rate_limit_exceeded": [
        "Wait 60 seconds before sending another request",
        "Review your previous answers while waiting",
        "Consider reading the textbook sections directly"
    ]
}


def get_alternative_actions(error_type: str) -> list[str]:
    """
    Get alternative action suggestions for an error type.

    Args:
        error_type: Type of error

    Returns:
        List of suggested alternative actions
    """
    return ALTERNATIVE_ACTIONS.get(error_type, [
        "Try again in a moment",
        "Refresh the page and retry",
        "Contact support if the issue persists"
    ])
