"""
Pydantic request models for API endpoints.
Defines input validation and serialization for all API requests.
"""

from typing import Optional, Literal
from pydantic import BaseModel, Field, validator
from uuid import UUID


class ChatQueryRequest(BaseModel):
    """
    Request model for POST /chat/query endpoint.

    Attributes:
        query: Natural language question (required, max 500 words ~2000 chars)
        selected_text: User-selected text context (optional, max 2000 chars)
        source_location: Source location of selected text (optional)
        conversation_id: Existing conversation ID for multi-turn (optional)
        answer_length: Desired answer length (optional)
        answer_style: Desired answer style (optional)
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="Natural language question from user"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=2000,
        description="User-selected text from textbook as context"
    )
    source_location: Optional[str] = Field(
        None,
        max_length=500,
        description="Source location of selected text (file path, line number)"
    )
    conversation_id: Optional[UUID] = Field(
        None,
        description="Existing conversation ID for multi-turn dialogue"
    )
    answer_length: Optional[Literal["brief", "medium", "detailed"]] = Field(
        "medium",
        description="Desired answer length"
    )
    answer_style: Optional[Literal["technical", "beginner-friendly"]] = Field(
        "technical",
        description="Desired answer style"
    )

    @validator("query")
    def validate_query_not_empty(cls, v):
        """Ensure query is not just whitespace."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty or whitespace")
        return v.strip()

    @validator("selected_text")
    def validate_selected_text(cls, v):
        """Sanitize selected text."""
        if v:
            return v.strip()
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is forward kinematics?",
                "selected_text": "Forward kinematics uses DH parameters",
                "source_location": "docs/module-1/chapter-3.md#L67",
                "conversation_id": None,
                "answer_length": "medium",
                "answer_style": "technical"
            }
        }


class FeedbackRequest(BaseModel):
    """
    Request model for POST /chat/feedback endpoint.

    Attributes:
        message_id: ID of the message being rated (required)
        rating: Thumbs up/down rating (required)
        comment: Optional free-text feedback (max 500 chars)
    """
    message_id: UUID = Field(
        ...,
        description="ID of the assistant message being rated"
    )
    rating: Literal["positive", "negative"] = Field(
        ...,
        description="User rating: positive (thumbs up) or negative (thumbs down)"
    )
    comment: Optional[str] = Field(
        None,
        max_length=500,
        description="Optional free-text feedback explaining the rating"
    )

    @validator("comment")
    def validate_comment(cls, v):
        """Sanitize comment."""
        if v:
            return v.strip()
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "550e8400-e29b-41d4-a716-446655440000",
                "rating": "positive",
                "comment": "Very helpful explanation with clear examples"
            }
        }


class ConversationHistoryRequest(BaseModel):
    """
    Request model for GET /chat/history endpoint (query parameters).

    Attributes:
        conversation_id: ID of conversation to retrieve
        limit: Maximum number of messages to return (optional)
    """
    conversation_id: UUID = Field(
        ...,
        description="Conversation ID to retrieve history for"
    )
    limit: Optional[int] = Field(
        20,
        ge=1,
        le=100,
        description="Maximum number of messages to return"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                "limit": 20
            }
        }


# Authentication and Personalization Request Models (Better Auth Integration)
from pydantic import EmailStr
import re


class SignupRequest(BaseModel):
    """
    Request model for user signup.

    Validates:
    - Email format (EmailStr)
    - Password complexity (min 8 chars, 1 uppercase, 1 lowercase, 1 number)
    - Skill levels (1-5 range)
    - Hardware access flags (optional, default False)
    """
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, max_length=128, description="User password (min 8 chars)")

    # Skill levels (mandatory, 1-5 range)
    ai_level: int = Field(..., ge=1, le=5, description="AI knowledge level (1=beginner, 5=expert)")
    ml_level: int = Field(..., ge=1, le=5, description="Machine Learning knowledge level")
    ros_level: int = Field(..., ge=1, le=5, description="ROS expertise level")
    python_level: int = Field(..., ge=1, le=5, description="Python programming proficiency")
    linux_level: int = Field(..., ge=1, le=5, description="Linux system administration skill")

    # Hardware access (optional, default False)
    has_gpu: bool = Field(default=False, description="Whether user has GPU access")
    has_jetson: bool = Field(default=False, description="Whether user has Nvidia Jetson device")
    has_robot: bool = Field(default=False, description="Whether user has physical robot hardware")

    @validator('password')
    def validate_password_complexity(cls, v: str) -> str:
        """
        Validate password meets complexity requirements:
        - At least 1 uppercase letter
        - At least 1 lowercase letter
        - At least 1 number
        - Minimum 8 characters (enforced by Field min_length)
        """
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not re.search(r'[0-9]', v):
            raise ValueError('Password must contain at least one number')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecurePass123",
                "ai_level": 3,
                "ml_level": 2,
                "ros_level": 1,
                "python_level": 4,
                "linux_level": 3,
                "has_gpu": True,
                "has_jetson": False,
                "has_robot": False
            }
        }


class SigninRequest(BaseModel):
    """
    Request model for user signin/login.

    Validates:
    - Email format
    - Password provided (no complexity check on signin, only on signup)
    """
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=1, description="User password")

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecurePass123"
            }
        }


class RefreshTokenRequest(BaseModel):
    """
    Request model for refreshing access token using refresh token.
    """
    refresh_token: str = Field(..., description="Valid refresh token")

    class Config:
        json_schema_extra = {
            "example": {
                "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
            }
        }


class PersonalizeRequest(BaseModel):
    """
    Request model for chapter personalization.

    User profile will be extracted from JWT token (authenticated endpoint).
    """
    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'module-0-foundations/chapter-2')")
    chapter_content: str = Field(..., min_length=10, max_length=50000, description="Original chapter markdown content")
    focus_areas: Optional[list[str]] = Field(default=None, description="Optional list of topics to emphasize (e.g., ['neural networks', 'GPU optimization'])")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "module-0-foundations/chapter-2",
                "chapter_content": "# Introduction to ROS 2\n\nROS 2 is the next generation...",
                "focus_areas": ["ROS navigation", "sensor fusion"]
            }
        }


class TranslateRequest(BaseModel):
    """
    Request model for chapter translation to Urdu.

    Includes Focus Mode toggle for technical faithfulness.
    """
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_content: str = Field(..., min_length=10, max_length=50000, description="Original chapter markdown content")
    focus_mode: bool = Field(default=True, description="Enable Focus Mode (technical faithfulness, no extra commentary)")
    focus_areas: Optional[list[str]] = Field(default=None, description="Optional list of topics to emphasize in translation (e.g., ['navigation', 'sensors'])")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "module-0-foundations/chapter-2",
                "chapter_content": "# Introduction to ROS 2\n\nROS 2 is the next generation...",
                "focus_mode": True,
                "focus_areas": ["ROS navigation", "sensor integration"]
            }
        }
