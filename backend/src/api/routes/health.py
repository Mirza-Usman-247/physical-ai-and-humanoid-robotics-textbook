"""
Health check endpoint for monitoring and load balancer probes.
Implements FR-026: GET /health endpoint with connectivity checks.
"""

from fastapi import APIRouter, status
from datetime import datetime
import logging

from src.api.models.response import HealthResponse
from src.db.connection import check_db_health
from src.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()

router = APIRouter()


def check_qdrant_health() -> bool:
    """
    Check Qdrant vector database connectivity.

    Returns:
        bool: True if Qdrant is reachable, False otherwise
    """
    try:
        # Import here to avoid circular dependencies
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=5  # 5 second timeout for health check
        )

        # Check if collection exists
        collections = client.get_collections()
        logger.debug(f"Qdrant health check: {len(collections.collections)} collections found")
        return True

    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        return False


@router.get(
    "/health",
    response_model=HealthResponse,
    status_code=status.HTTP_200_OK,
    summary="Health check endpoint",
    description="Check backend service health including database and vector store connectivity"
)
async def health_check():
    """
    Health check endpoint for monitoring.

    Checks:
    - Database (Neon Postgres) connectivity
    - Qdrant vector database connectivity
    - Overall service status

    Returns:
        HealthResponse: Health status of all components

    Example:
        GET /health
        {
            "status": "healthy",
            "database": "connected",
            "qdrant": "connected",
            "timestamp": "2025-12-16T14:30:00Z"
        }
    """
    # Check database health
    db_healthy = check_db_health()
    db_status = "connected" if db_healthy else "disconnected"

    # Check Qdrant health
    qdrant_healthy = check_qdrant_health()
    qdrant_status = "connected" if qdrant_healthy else "disconnected"

    # Determine overall status
    overall_healthy = db_healthy and qdrant_healthy
    overall_status = "healthy" if overall_healthy else "unhealthy"

    # Log unhealthy status
    if not overall_healthy:
        logger.warning(f"Health check failed - Database: {db_status}, Qdrant: {qdrant_status}")

    return HealthResponse(
        status=overall_status,
        database=db_status,
        qdrant=qdrant_status,
        timestamp=datetime.utcnow()
    )


@router.get(
    "/health/database",
    status_code=status.HTTP_200_OK,
    summary="Database health check",
    description="Check only database connectivity"
)
async def database_health_check():
    """
    Database-specific health check.

    Returns:
        dict: Database connection status
    """
    db_healthy = check_db_health()
    return {
        "database": "connected" if db_healthy else "disconnected",
        "timestamp": datetime.utcnow().isoformat()
    }


@router.get(
    "/health/qdrant",
    status_code=status.HTTP_200_OK,
    summary="Qdrant health check",
    description="Check only Qdrant vector database connectivity"
)
async def qdrant_health_check():
    """
    Qdrant-specific health check.

    Returns:
        dict: Qdrant connection status
    """
    qdrant_healthy = check_qdrant_health()
    return {
        "qdrant": "connected" if qdrant_healthy else "disconnected",
        "timestamp": datetime.utcnow().isoformat()
    }
