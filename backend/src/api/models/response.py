"""
Pydantic response models for API endpoints.
Defines output serialization for all API responses.
"""

from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime


class CitationSource(BaseModel):
    """
    Model for a single citation source.

    Attributes:
        id: Citation number (e.g., 1 for [1])
        file_path: Path to source file
        line_range: Line range in source file
        section: Section hierarchy (Module > Chapter > Section)
        score: Similarity score from retrieval
    """
    id: int = Field(..., description="Citation number")
    file_path: str = Field(..., description="Path to source file")
    line_range: str = Field(..., description="Line range (e.g., '67-95')")
    section: str = Field(..., description="Section hierarchy")
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score")

    class Config:
        json_schema_extra = {
            "example": {
                "id": 1,
                "file_path": "docs/module-1/chapter-3.md",
                "line_range": "67-95",
                "section": "Module 1 > Chapter 3 > Forward Kinematics",
                "score": 0.92
            }
        }


class ChatQueryResponse(BaseModel):
    """
    Response model for POST /chat/query endpoint.

    Attributes:
        message_id: UUID of the assistant message
        conversation_id: UUID of the conversation
        answer: Generated answer with inline [N] citations
        sources: List of citation sources
        follow_up_suggestions: Suggested follow-up questions (optional)
        confidence: Confidence score for the answer
        response_time_ms: Response time in milliseconds
        timestamp: ISO timestamp when response was generated
    """
    message_id: UUID = Field(..., description="Message ID")
    conversation_id: UUID = Field(..., description="Conversation ID")
    answer: str = Field(..., description="Generated answer with inline citations")
    sources: List[CitationSource] = Field(..., description="Citation sources")
    follow_up_suggestions: Optional[List[str]] = Field(
        None,
        max_length=3,
        description="2-3 suggested follow-up questions"
    )
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score")
    response_time_ms: int = Field(..., ge=0, description="Response time in milliseconds")
    timestamp: datetime = Field(..., description="Response timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "550e8400-e29b-41d4-a716-446655440000",
                "conversation_id": "660e8400-e29b-41d4-a716-446655440001",
                "answer": "Forward kinematics is the process of determining the position and orientation of a robot's end-effector given its joint parameters [1]. It uses the Denavit-Hartenberg convention to establish coordinate frames [2].",
                "sources": [
                    {
                        "id": 1,
                        "file_path": "docs/module-1/chapter-3.md",
                        "line_range": "67-95",
                        "section": "Module 1 > Chapter 3 > Forward Kinematics",
                        "score": 0.92
                    },
                    {
                        "id": 2,
                        "file_path": "docs/module-1/chapter-3.md",
                        "line_range": "120-145",
                        "section": "Module 1 > Chapter 3 > DH Convention",
                        "score": 0.88
                    }
                ],
                "follow_up_suggestions": [
                    "How does inverse kinematics differ from forward kinematics?",
                    "Can you show me the DH parameter table?",
                    "What are the limitations of forward kinematics?"
                ],
                "confidence": 0.89,
                "response_time_ms": 2450,
                "timestamp": "2025-12-16T14:30:00Z"
            }
        }


class MessageResponse(BaseModel):
    """
    Model for a single message in conversation history.

    Attributes:
        message_id: UUID of the message
        role: Message role (user or assistant)
        content: Message content
        timestamp: Message timestamp
        response_time_ms: Response time (for assistant messages)
    """
    message_id: UUID = Field(..., description="Message ID")
    role: str = Field(..., description="Message role: user or assistant")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(..., description="Message timestamp")
    response_time_ms: Optional[int] = Field(None, description="Response time in milliseconds")

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "550e8400-e29b-41d4-a716-446655440000",
                "role": "user",
                "content": "What is forward kinematics?",
                "timestamp": "2025-12-16T14:29:58Z",
                "response_time_ms": None
            }
        }


class ConversationHistoryResponse(BaseModel):
    """
    Response model for GET /chat/history endpoint.

    Attributes:
        conversation_id: UUID of the conversation
        messages: List of messages in chronological order
        total_messages: Total number of messages in conversation
        start_timestamp: When conversation started
    """
    conversation_id: UUID = Field(..., description="Conversation ID")
    messages: List[MessageResponse] = Field(..., description="Messages in chronological order")
    total_messages: int = Field(..., description="Total message count")
    start_timestamp: datetime = Field(..., description="Conversation start time")

    class Config:
        json_schema_extra = {
            "example": {
                "conversation_id": "660e8400-e29b-41d4-a716-446655440001",
                "messages": [
                    {
                        "message_id": "550e8400-e29b-41d4-a716-446655440000",
                        "role": "user",
                        "content": "What is forward kinematics?",
                        "timestamp": "2025-12-16T14:29:58Z",
                        "response_time_ms": None
                    }
                ],
                "total_messages": 1,
                "start_timestamp": "2025-12-16T14:29:58Z"
            }
        }


class FeedbackResponse(BaseModel):
    """
    Response model for POST /chat/feedback endpoint.

    Attributes:
        feedback_id: UUID of the created feedback
        message_id: UUID of the rated message
        rating: Rating value
        timestamp: When feedback was submitted
    """
    feedback_id: UUID = Field(..., description="Feedback ID")
    message_id: UUID = Field(..., description="Rated message ID")
    rating: str = Field(..., description="Rating: positive or negative")
    timestamp: datetime = Field(..., description="Feedback timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "feedback_id": "770e8400-e29b-41d4-a716-446655440002",
                "message_id": "550e8400-e29b-41d4-a716-446655440000",
                "rating": "positive",
                "timestamp": "2025-12-16T14:30:15Z"
            }
        }


class HealthResponse(BaseModel):
    """
    Response model for GET /health endpoint.

    Attributes:
        status: Overall health status
        database: Database connectivity status
        qdrant: Qdrant connectivity status
        timestamp: Health check timestamp
    """
    status: str = Field(..., description="Overall status: healthy or unhealthy")
    database: str = Field(..., description="Database status: connected or disconnected")
    qdrant: str = Field(..., description="Qdrant status: connected or disconnected")
    timestamp: datetime = Field(..., description="Health check timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "database": "connected",
                "qdrant": "connected",
                "timestamp": "2025-12-16T14:30:00Z"
            }
        }


class ErrorResponse(BaseModel):
    """
    Standard error response model for all API errors.

    Attributes:
        error: Error type/code
        message: Human-readable error message
        details: Optional additional error details
        timestamp: Error timestamp
    """
    error: str = Field(..., description="Error type or code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details")
    timestamp: datetime = Field(..., description="Error timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "validation_error",
                "message": "Query cannot be empty",
                "details": {"field": "query"},
                "timestamp": "2025-12-16T14:30:00Z"
            }
        }


# Authentication and Personalization Response Models (Better Auth Integration)


class UserProfileResponse(BaseModel):
    """
    Response model for user profile data.

    Includes skill levels and hardware access flags.
    """
    user_id: UUID = Field(..., description="User UUID")
    email: str = Field(..., description="User email address")
    ai_level: int = Field(..., description="AI knowledge level (1-5)")
    ml_level: int = Field(..., description="Machine Learning knowledge level (1-5)")
    ros_level: int = Field(..., description="ROS expertise level (1-5)")
    python_level: int = Field(..., description="Python programming proficiency (1-5)")
    linux_level: int = Field(..., description="Linux system administration skill (1-5)")
    has_gpu: bool = Field(..., description="Whether user has GPU access")
    has_jetson: bool = Field(..., description="Whether user has Nvidia Jetson device")
    has_robot: bool = Field(..., description="Whether user has physical robot hardware")
    cloud_only: bool = Field(..., description="Computed: true if no physical hardware")
    created_at: datetime = Field(..., description="Account creation timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": "550e8400-e29b-41d4-a716-446655440000",
                "email": "user@example.com",
                "ai_level": 3,
                "ml_level": 2,
                "ros_level": 1,
                "python_level": 4,
                "linux_level": 3,
                "has_gpu": True,
                "has_jetson": False,
                "has_robot": False,
                "cloud_only": False,
                "created_at": "2025-12-17T10:00:00Z"
            }
        }


class AuthTokensResponse(BaseModel):
    """
    Response model for JWT tokens.

    Includes access token (15-min expiry) and refresh token (7-day expiry).
    """
    access_token: str = Field(..., description="JWT access token (15-minute expiry)")
    refresh_token: str = Field(..., description="JWT refresh token (7-day expiry)")
    token_type: str = Field(default="bearer", description="Token type (always 'bearer')")

    class Config:
        json_schema_extra = {
            "example": {
                "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI1NTBlODQwMC1lMjliLTQxZDQtYTcxNi00NDY2NTU0NDAwMDAiLCJleHAiOjE3MDI4MjQ2MDAsInR5cGUiOiJhY2Nlc3MifQ.signature",
                "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI1NTBlODQwMC1lMjliLTQxZDQtYTcxNi00NDY2NTU0NDAwMDAiLCJleHAiOjE3MDM0MjkwMDAsInR5cGUiOiJyZWZyZXNoIn0.signature",
                "token_type": "bearer"
            }
        }


class SignupResponse(BaseModel):
    """
    Response model for successful user signup.

    Includes user profile and JWT tokens.
    """
    user: UserProfileResponse = Field(..., description="User profile data")
    tokens: AuthTokensResponse = Field(..., description="JWT access and refresh tokens")

    class Config:
        json_schema_extra = {
            "example": {
                "user": {
                    "user_id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "user@example.com",
                    "ai_level": 3,
                    "ml_level": 2,
                    "ros_level": 1,
                    "python_level": 4,
                    "linux_level": 3,
                    "has_gpu": True,
                    "has_jetson": False,
                    "has_robot": False,
                    "cloud_only": False,
                    "created_at": "2025-12-17T10:00:00Z"
                },
                "tokens": {
                    "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                    "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                    "token_type": "bearer"
                }
            }
        }


class SigninResponse(BaseModel):
    """
    Response model for successful user signin.

    Same structure as SignupResponse.
    """
    user: UserProfileResponse = Field(..., description="User profile data")
    tokens: AuthTokensResponse = Field(..., description="JWT access and refresh tokens")

    class Config:
        json_schema_extra = {
            "example": {
                "user": {
                    "user_id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "user@example.com",
                    "ai_level": 3,
                    "ml_level": 2,
                    "ros_level": 1,
                    "python_level": 4,
                    "linux_level": 3,
                    "has_gpu": True,
                    "has_jetson": False,
                    "has_robot": False,
                    "cloud_only": False,
                    "created_at": "2025-12-17T10:00:00Z"
                },
                "tokens": {
                    "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                    "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                    "token_type": "bearer"
                }
            }
        }


class PersonalizeResponse(BaseModel):
    """
    Response model for chapter personalization.

    Includes personalized markdown content and transformation metadata.
    """
    chapter_id: str = Field(..., description="Chapter identifier")
    personalized_content: str = Field(..., description="Personalized markdown content")
    transformation_metadata: Dict[str, Any] = Field(..., description="LLM transformation metadata")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "module-0-foundations/chapter-2",
                "personalized_content": "# Introduction to ROS 2 (Tailored for Your Level)\n\n...",
                "transformation_metadata": {
                    "llm_model": "meta-llama/llama-3.3-70b-instruct",
                    "profile_snapshot": {"ai_level": 3, "ml_level": 2, "has_gpu": True},
                    "request_timestamp": "2025-12-17T10:05:00Z",
                    "response_timestamp": "2025-12-17T10:05:08Z",
                    "token_usage": {"input_tokens": 3500, "output_tokens": 4200}
                }
            }
        }


class TranslateResponse(BaseModel):
    """
    Response model for chapter translation to Urdu.

    Includes translated markdown content and transformation metadata.
    """
    chapter_id: str = Field(..., description="Chapter identifier")
    translated_content: str = Field(..., description="Urdu translated markdown content")
    transformation_metadata: Dict[str, Any] = Field(..., description="LLM transformation metadata")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "module-0-foundations/chapter-2",
                "translated_content": "# ROS 2 کا تعارف\n\n...",
                "transformation_metadata": {
                    "llm_model": "google/gemini-2.0-flash-exp:free",
                    "focus_mode": True,
                    "request_timestamp": "2025-12-17T10:10:00Z",
                    "response_timestamp": "2025-12-17T10:10:12Z",
                    "token_usage": {"input_tokens": 3500, "output_tokens": 4500}
                }
            }
        }
