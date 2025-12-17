"""
SQLAlchemy database models for RAG Chatbot.
Defines schema for users, conversations, messages, and feedback.
"""

import uuid
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, String, Integer, Text, TIMESTAMP, UUID, ForeignKey, JSON, Enum as SQLEnum
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import enum


Base = declarative_base()


class ConversationStatus(str, enum.Enum):
    """Enum for conversation status."""
    ACTIVE = "active"
    ARCHIVED = "archived"


class MessageRole(str, enum.Enum):
    """Enum for message role."""
    USER = "user"
    ASSISTANT = "assistant"


class FeedbackRating(str, enum.Enum):
    """Enum for feedback rating."""
    POSITIVE = "positive"
    NEGATIVE = "negative"


class User(Base):
    """
    User model representing a student or reader.

    Attributes:
        user_id: UUID primary key
        session_id: Unique session identifier for anonymous tracking
        created_at: Timestamp when user was created
    """
    __tablename__ = "rag_users"

    user_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    created_at = Column(TIMESTAMP, server_default=func.now(), nullable=False)

    # Relationships
    conversations = relationship("Conversation", back_populates="user", cascade="all, delete-orphan")


class Conversation(Base):
    """
    Conversation model representing a multi-turn dialogue session.

    Attributes:
        conversation_id: UUID primary key
        user_id: Foreign key to users table
        start_timestamp: When conversation started
        end_timestamp: When conversation ended (NULL if active)
        message_count: Number of messages in conversation
        status: Current status (active/archived)
    """
    __tablename__ = "conversations"

    conversation_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("rag_users.user_id", ondelete="CASCADE"), nullable=False, index=True)
    start_timestamp = Column(TIMESTAMP, server_default=func.now(), nullable=False)
    end_timestamp = Column(TIMESTAMP, nullable=True)
    message_count = Column(Integer, default=0, nullable=False)
    status = Column(SQLEnum(ConversationStatus), default=ConversationStatus.ACTIVE, nullable=False)

    # Relationships
    user = relationship("User", back_populates="conversations")
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")


class Message(Base):
    """
    Message model representing a single Q&A exchange.

    Attributes:
        message_id: UUID primary key
        conversation_id: Foreign key to conversations table
        role: Message role (user/assistant)
        content_text: Message content
        selected_context: User-selected text context (JSON)
        retrieved_chunks: Retrieved context chunks (JSON)
        citations: Citation mappings (JSON)
        timestamp: When message was created
        response_time_ms: Response time in milliseconds (for assistant messages)
    """
    __tablename__ = "messages"

    message_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.conversation_id", ondelete="CASCADE"), nullable=False, index=True)
    role = Column(SQLEnum(MessageRole), nullable=False)
    content_text = Column(Text, nullable=False)
    selected_context = Column(JSON, nullable=True)  # {text: str, source_location: str}
    retrieved_chunks = Column(JSON, nullable=True)  # [{chunk_id: str, score: float, content: str}]
    citations = Column(JSON, nullable=True)  # [{number: int, file_path: str, line_range: str}]
    timestamp = Column(TIMESTAMP, server_default=func.now(), nullable=False)
    response_time_ms = Column(Integer, nullable=True)

    # Relationships
    conversation = relationship("Conversation", back_populates="messages")
    feedback = relationship("Feedback", back_populates="message", uselist=False, cascade="all, delete-orphan")


class Feedback(Base):
    """
    Feedback model representing user evaluation of a chatbot answer.

    Attributes:
        feedback_id: UUID primary key
        message_id: Foreign key to messages table
        rating: Feedback rating (positive/negative)
        comment: Optional free-text feedback
        timestamp: When feedback was submitted
    """
    __tablename__ = "feedback"

    feedback_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    message_id = Column(UUID(as_uuid=True), ForeignKey("messages.message_id", ondelete="CASCADE"), nullable=False, unique=True, index=True)
    rating = Column(SQLEnum(FeedbackRating), nullable=False)
    comment = Column(Text, nullable=True)
    timestamp = Column(TIMESTAMP, server_default=func.now(), nullable=False)

    # Relationships
    message = relationship("Message", back_populates="feedback")


# Index definitions for query optimization
from sqlalchemy import Index

Index("idx_conversations_user", Conversation.user_id)
Index("idx_messages_conversation", Message.conversation_id)
Index("idx_feedback_message", Feedback.message_id)
Index("idx_rag_users_session", User.session_id)


# Authentication models (Better Auth integration)
from sqlalchemy import Boolean, CheckConstraint, Computed


class AuthUser(Base):
    """
    Authentication user model for Better Auth integration.

    Attributes:
        id: UUID primary key (auto-generated)
        email: Unique email address (validated format)
        password_hash: bcrypt hashed password (cost factor 12)
        is_active: Whether account is active (default: true)
        created_at: Account creation timestamp
        updated_at: Last update timestamp (auto-updated via trigger)
    """
    __tablename__ = "auth_users"

    id = Column(UUID(as_uuid=True), primary_key=True, server_default=func.gen_random_uuid())
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    is_active = Column(Boolean, nullable=False, server_default="true")
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")

    # Email format validation constraint (enforced at database level)
    __table_args__ = (
        CheckConstraint(
            "email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Z|a-z]{2,}$'",
            name="email_format_check"
        ),
    )


class UserProfile(Base):
    """
    User profile model storing skill levels and hardware access.

    Attributes:
        user_id: Foreign key to auth_users (one-to-one relationship)
        ai_level: AI knowledge level (1-5, where 1=beginner, 5=expert)
        ml_level: Machine Learning knowledge level (1-5)
        ros_level: ROS expertise level (1-5)
        python_level: Python programming proficiency (1-5)
        linux_level: Linux system administration skill (1-5)
        has_gpu: Whether user has GPU access
        has_jetson: Whether user has Nvidia Jetson device
        has_robot: Whether user has physical robot hardware
        cloud_only: Computed field (true if no physical hardware)
        created_at: Profile creation timestamp
        updated_at: Last update timestamp (auto-updated via trigger)
    """
    __tablename__ = "user_profiles"

    user_id = Column(UUID(as_uuid=True), ForeignKey("auth_users.id", ondelete="CASCADE"), primary_key=True)
    ai_level = Column(Integer, nullable=False)
    ml_level = Column(Integer, nullable=False)
    ros_level = Column(Integer, nullable=False)
    python_level = Column(Integer, nullable=False)
    linux_level = Column(Integer, nullable=False)
    has_gpu = Column(Boolean, nullable=False, server_default="false")
    has_jetson = Column(Boolean, nullable=False, server_default="false")
    has_robot = Column(Boolean, nullable=False, server_default="false")
    cloud_only = Column(Boolean, Computed("(NOT has_gpu AND NOT has_jetson AND NOT has_robot)"))
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())

    # Relationships
    user = relationship("AuthUser", back_populates="profile")

    # Skill level range validation constraints (1-5)
    __table_args__ = (
        CheckConstraint("ai_level >= 1 AND ai_level <= 5", name="ai_level_range_check"),
        CheckConstraint("ml_level >= 1 AND ml_level <= 5", name="ml_level_range_check"),
        CheckConstraint("ros_level >= 1 AND ros_level <= 5", name="ros_level_range_check"),
        CheckConstraint("python_level >= 1 AND python_level <= 5", name="python_level_range_check"),
        CheckConstraint("linux_level >= 1 AND linux_level <= 5", name="linux_level_range_check"),
    )


# Additional indexes for authentication models
Index("idx_auth_users_email", AuthUser.email, unique=True)
Index("idx_user_profiles_user_id", UserProfile.user_id, unique=True)
