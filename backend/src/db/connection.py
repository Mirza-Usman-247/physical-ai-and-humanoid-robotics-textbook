"""
Database connection management for Neon Serverless Postgres.
Implements connection pooling and session management.
"""

from typing import Generator
from sqlalchemy import create_engine, event, text
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import QueuePool
from contextlib import contextmanager
import logging

from src.config import get_settings
from src.db.models import Base

logger = logging.getLogger(__name__)
settings = get_settings()


# Create engine with connection pooling
engine = create_engine(
    settings.neon_database_url,
    poolclass=QueuePool,
    pool_size=10,  # Minimum 10 connections as per plan.md
    max_overflow=20,  # Allow up to 30 total connections
    pool_pre_ping=True,  # Verify connections before using
    pool_recycle=3600,  # Recycle connections after 1 hour
    echo=settings.debug,  # Log SQL queries in debug mode
)


# Create session factory
SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
)


def init_db() -> None:
    """
    Initialize the database by creating all tables.

    This should be called on application startup.
    Note: In production, use Alembic migrations instead.
    """
    try:
        Base.metadata.create_all(bind=engine)
        logger.info("Database tables created successfully")
    except Exception as e:
        logger.error(f"Failed to create database tables: {e}")
        raise


def get_db() -> Generator[Session, None, None]:
    """
    Dependency that provides a database session.

    Yields:
        Session: SQLAlchemy session

    Example:
        >>> from fastapi import Depends
        >>> @app.get("/users")
        >>> def get_users(db: Session = Depends(get_db)):
        >>>     return db.query(User).all()
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@contextmanager
def get_db_context() -> Generator[Session, None, None]:
    """
    Context manager for database sessions outside of FastAPI dependency injection.

    Yields:
        Session: SQLAlchemy session

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     user = db.query(User).first()
        >>>     print(user.session_id)
    """
    db = SessionLocal()
    try:
        yield db
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def check_db_health() -> bool:
    """
    Check database connectivity for health checks.

    Returns:
        bool: True if database is reachable, False otherwise
    """
    try:
        with engine.connect() as connection:
            connection.execute(text("SELECT 1"))
        return True
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        return False


# Event listeners for connection lifecycle logging
@event.listens_for(engine, "connect")
def receive_connect(dbapi_conn, connection_record):
    """Log database connection events."""
    logger.debug("Database connection established")


@event.listens_for(engine, "close")
def receive_close(dbapi_conn, connection_record):
    """Log database connection close events."""
    logger.debug("Database connection closed")
