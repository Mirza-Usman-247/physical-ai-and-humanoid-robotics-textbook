"""
FastAPI application initialization and configuration.
Main entry point for the RAG Chatbot backend API.
"""

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from contextlib import asynccontextmanager
import logging

from src.config import get_settings
from src.db.connection import init_db
from src.api.middleware.rate_limit import RateLimitMiddleware
from src.api.middleware.error_handler import (
    validation_exception_handler,
    http_exception_handler,
    general_exception_handler
)
from src.api.routes import health, chat, auth, profile, personalize, translation

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for FastAPI application.
    Handles startup and shutdown events.
    """
    # Startup
    logger.info("Starting RAG Chatbot backend API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Debug mode: {settings.debug}")

    try:
        # Initialize database
        init_db()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database: {e}")
        if settings.environment == "production":
            raise

    logger.info("API startup complete")

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot backend API...")
    logger.info("API shutdown complete")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    docs_url="/docs" if settings.debug else None,  # Disable docs in production
    redoc_url="/redoc" if settings.debug else None,
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus dev server
        "http://localhost:8000",  # Backend dev server
        "https://*.github.io",  # GitHub Pages
        "https://*.pages.dev",  # Alternative deployment
    ] if settings.is_development else [
        "https://*.github.io",  # GitHub Pages production
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"]
)

# Add rate limiting middleware
app.add_middleware(RateLimitMiddleware)

# Register exception handlers
app.add_exception_handler(RequestValidationError, validation_exception_handler)
app.add_exception_handler(StarletteHTTPException, http_exception_handler)
app.add_exception_handler(Exception, general_exception_handler)

# Include routers
app.include_router(health.router, tags=["Health"])
app.include_router(chat.router, tags=["Chat"])
app.include_router(auth.router, prefix="/api/auth", tags=["Authentication"])
app.include_router(profile.router, prefix="/api", tags=["Profile"])
app.include_router(personalize.router, prefix="/api", tags=["Personalization"])
app.include_router(translation.router, prefix="/api", tags=["Translation"])

# Root endpoint
@app.get("/")
async def root():
    """
    Root endpoint providing API information.

    Returns:
        dict: API name, version, and status
    """
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "operational",
        "environment": settings.environment,
        "docs_url": "/docs" if settings.debug else "Disabled in production"
    }


# Middleware for request logging
@app.middleware("http")
async def log_requests(request: Request, call_next):
    """
    Log all incoming requests.

    Args:
        request: Incoming HTTP request
        call_next: Next middleware/route handler

    Returns:
        Response from route handler
    """
    logger.info(f"{request.method} {request.url.path} - Client: {request.client.host if request.client else 'Unknown'}")
    response = await call_next(request)
    logger.info(f"{request.method} {request.url.path} - Status: {response.status_code}")
    return response


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level=settings.log_level.lower()
    )
