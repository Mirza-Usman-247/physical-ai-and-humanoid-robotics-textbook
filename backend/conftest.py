"""
Pytest configuration and fixtures for testing.
"""
import os
import pytest
import uuid
from typing import Generator
from fastapi.testclient import TestClient
from sqlalchemy import create_engine, TypeDecorator, CHAR
from sqlalchemy.dialects.postgresql import UUID as PGUUID
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import StaticPool


# Custom UUID type that works with both PostgreSQL and SQLite
class UUID(TypeDecorator):
    """Platform-independent UUID type.
    Uses PostgreSQL's UUID type, otherwise uses CHAR(36), storing as stringified hex values."""
    impl = CHAR
    cache_ok = True

    def __init__(self, as_uuid=True):
        """Accept as_uuid parameter for compatibility with PostgreSQL UUID type."""
        self.as_uuid = as_uuid
        super().__init__()

    def load_dialect_impl(self, dialect):
        if dialect.name == 'postgresql':
            return dialect.type_descriptor(PGUUID())
        else:
            return dialect.type_descriptor(CHAR(36))

    def process_bind_param(self, value, dialect):
        if value is None:
            return value
        elif dialect.name == 'postgresql':
            return str(value)
        else:
            if isinstance(value, uuid.UUID):
                return str(value)
            else:
                return str(uuid.UUID(value))

    def process_result_value(self, value, dialect):
        if value is None:
            return value
        else:
            if isinstance(value, uuid.UUID):
                return value
            else:
                return uuid.UUID(value)


# Monkey-patch UUID in sqlalchemy BEFORE importing models
import sqlalchemy
original_sqlalchemy_uuid = sqlalchemy.UUID
sqlalchemy.UUID = UUID

# Now import modules that use UUID (after monkey-patch)
from src.db.models import Base
from src.config import get_settings
from src.api.main import app
from src.db.connection import get_db


# Use in-memory SQLite for tests
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"

@pytest.fixture(scope="function")
def test_db_engine():
    """Create a fresh in-memory SQLite database for each test."""
    engine = create_engine(
        SQLALCHEMY_DATABASE_URL,
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )
    Base.metadata.create_all(bind=engine)
    yield engine
    Base.metadata.drop_all(bind=engine)
    engine.dispose()


@pytest.fixture(scope="function")
def test_db_session(test_db_engine) -> Generator[Session, None, None]:
    """Create a new database session for each test."""
    TestingSessionLocal = sessionmaker(
        autocommit=False, autoflush=False, bind=test_db_engine
    )
    session = TestingSessionLocal()
    try:
        yield session
    finally:
        session.rollback()
        session.close()


@pytest.fixture(scope="function")
def test_client(test_db_session: Session) -> TestClient:
    """Create a TestClient with overridden database dependency."""
    def override_get_db():
        try:
            yield test_db_session
        finally:
            pass

    app.dependency_overrides[get_db] = override_get_db
    client = TestClient(app)
    yield client
    app.dependency_overrides.clear()


@pytest.fixture(scope="session")
def test_settings():
    """Override settings for testing."""
    # Set test environment variables
    os.environ["JWT_SECRET_KEY"] = "test_secret_key_at_least_32_characters_long_for_jwt"
    os.environ["JWT_ALGORITHM"] = "HS256"
    os.environ["JWT_ACCESS_TOKEN_EXPIRE_MINUTES"] = "15"
    os.environ["JWT_REFRESH_TOKEN_EXPIRE_DAYS"] = "7"
    return get_settings()
