"""
Configuration management for RAG Chatbot backend.
Loads and validates environment variables with proper typing.
"""

from typing import Optional
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, validator


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    # Qdrant Configuration
    qdrant_url: str = Field(..., description="Qdrant Cloud cluster URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")
    qdrant_collection: str = Field(default="physical-ai-textbook", description="Qdrant collection name")

    # OpenRouter API Configuration
    openrouter_api_key: str = Field(..., description="OpenRouter API key")
    openrouter_base_url: str = Field(default="https://openrouter.ai/api/v1", description="OpenRouter base URL")
    qwen_embedding_model: str = Field(default="qwen/qwen-2-embedding", description="Qwen embedding model")
    openrouter_llm_model: str = Field(default="deepseek/deepseek-chat", description="LLM model for answer generation")
    openrouter_llm_model_personalize: str = Field(default="google/gemini-2.0-flash-exp:free", description="Free LLM model for content personalization")
    openrouter_llm_model_translate: str = Field(default="google/gemini-2.0-flash-exp:free", description="Free LLM model for translation")

    # Neon Serverless Postgres
    neon_database_url: str = Field(..., description="Neon Postgres connection string")

    # Application Settings
    environment: str = Field(default="development", description="Environment: development, staging, production")
    debug: bool = Field(default=False, description="Enable debug mode")
    log_level: str = Field(default="INFO", description="Logging level")

    # Security
    secret_key: str = Field(..., description="Secret key for session management")
    rate_limit_per_minute: int = Field(default=20, description="Rate limit per user per minute")

    # JWT Authentication (Better Auth Integration)
    jwt_secret_key: str = Field(..., description="Secret key for JWT token signing (minimum 32 characters)")
    jwt_algorithm: str = Field(default="HS256", description="JWT signing algorithm")
    jwt_access_token_expire_minutes: int = Field(default=15, description="Access token expiration in minutes")
    jwt_refresh_token_expire_days: int = Field(default=7, description="Refresh token expiration in days")

    # Token Limits
    max_input_tokens: int = Field(default=8000, description="Maximum input tokens per OpenRouter request")
    max_output_tokens: int = Field(default=2000, description="Maximum output tokens per OpenRouter request")

    # Retrieval Settings
    similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score for retrieval")
    top_k_chunks: int = Field(default=5, ge=1, le=20, description="Number of chunks to retrieve")
    conversation_context_window: int = Field(default=5, ge=0, le=20, description="Number of previous Q&A exchanges to retain")

    @validator("neon_database_url")
    def validate_database_url(cls, v: str) -> str:
        """Validate that database URL uses SSL."""
        if "sslmode" not in v:
            raise ValueError("Database URL must include sslmode parameter for security")
        return v

    @validator("qdrant_url")
    def validate_qdrant_url(cls, v: str) -> str:
        """Validate Qdrant URL format."""
        if not v.startswith(("http://", "https://")):
            raise ValueError("Qdrant URL must start with http:// or https://")
        return v

    @validator("environment")
    def validate_environment(cls, v: str) -> str:
        """Validate environment is one of allowed values."""
        allowed = ["development", "staging", "production"]
        if v.lower() not in allowed:
            raise ValueError(f"Environment must be one of: {', '.join(allowed)}")
        return v.lower()

    @validator("jwt_secret_key")
    def validate_jwt_secret_key(cls, v: str) -> str:
        """Validate JWT secret key is sufficiently long."""
        if len(v) < 32:
            raise ValueError("JWT_SECRET_KEY must be at least 32 characters for security")
        return v

    @property
    def is_production(self) -> bool:
        """Check if running in production mode."""
        return self.environment == "production"

    @property
    def is_development(self) -> bool:
        """Check if running in development mode."""
        return self.environment == "development"


# Global settings instance
settings = Settings()


def get_settings() -> Settings:
    """
    Get the application settings instance.

    Returns:
        Settings: Configured settings object

    Example:
        >>> from src.config import get_settings
        >>> config = get_settings()
        >>> print(config.qdrant_url)
    """
    return settings
