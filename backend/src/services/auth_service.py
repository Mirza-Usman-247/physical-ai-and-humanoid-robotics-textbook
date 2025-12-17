"""
Authentication service for user signup, signin, and token management.

Handles user creation, authentication, and JWT token generation.
"""
from typing import Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from fastapi import HTTPException, status

from src.db.models import AuthUser, UserProfile
from src.utils.password import hash_password, verify_password
from src.utils.jwt import create_access_token, create_refresh_token


def signup_user(
    db: Session,
    email: str,
    password: str,
    ai_level: int,
    ml_level: int,
    ros_level: int,
    python_level: int,
    linux_level: int,
    has_gpu: bool = False,
    has_jetson: bool = False,
    has_robot: bool = False,
) -> Dict[str, Any]:
    """
    Create a new user account with profile and return JWT tokens.

    Args:
        db: Database session
        email: User email address
        password: Plain text password (will be hashed)
        ai_level: AI knowledge level (1-5)
        ml_level: ML knowledge level (1-5)
        ros_level: ROS expertise level (1-5)
        python_level: Python proficiency (1-5)
        linux_level: Linux skill (1-5)
        has_gpu: Has GPU hardware (default: False)
        has_jetson: Has Jetson hardware (default: False)
        has_robot: Has robot hardware (default: False)

    Returns:
        Dict containing user profile and JWT tokens:
        {
            "user": {
                "user_id": str,
                "email": str,
                "ai_level": int,
                ...
            },
            "tokens": {
                "access_token": str,
                "refresh_token": str,
                "token_type": "bearer"
            }
        }

    Raises:
        HTTPException(409): Email already registered
        HTTPException(500): Database error

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     user_session = signup_user(
        >>>         db=db,
        >>>         email="user@example.com",
        >>>         password="SecurePass123",
        >>>         ai_level=3, ml_level=4, ros_level=2,
        >>>         python_level=5, linux_level=3,
        >>>         has_gpu=True
        >>>     )
        >>>     print(user_session["tokens"]["access_token"])
    """
    # Check if email already exists
    existing_user = db.query(AuthUser).filter(AuthUser.email == email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered"
        )

    try:
        # Hash password
        password_hash = hash_password(password)

        # Create user
        new_user = AuthUser(
            email=email,
            password_hash=password_hash,
            is_active=True
        )
        db.add(new_user)
        db.flush()  # Flush to get user.id without committing

        # Create user profile
        user_profile = UserProfile(
            user_id=new_user.id,
            ai_level=ai_level,
            ml_level=ml_level,
            ros_level=ros_level,
            python_level=python_level,
            linux_level=linux_level,
            has_gpu=has_gpu,
            has_jetson=has_jetson,
            has_robot=has_robot
        )
        db.add(user_profile)
        db.commit()
        db.refresh(new_user)
        db.refresh(user_profile)

        # Generate JWT tokens
        token_data = {
            "sub": str(new_user.id),
            "email": new_user.email
        }
        access_token = create_access_token(token_data)
        refresh_token = create_refresh_token(token_data)

        # Return user session
        return {
            "user": {
                "user_id": str(new_user.id),
                "email": new_user.email,
                "ai_level": user_profile.ai_level,
                "ml_level": user_profile.ml_level,
                "ros_level": user_profile.ros_level,
                "python_level": user_profile.python_level,
                "linux_level": user_profile.linux_level,
                "has_gpu": user_profile.has_gpu,
                "has_jetson": user_profile.has_jetson,
                "has_robot": user_profile.has_robot,
                "cloud_only": user_profile.cloud_only,
                "created_at": user_profile.created_at.isoformat()
            },
            "tokens": {
                "access_token": access_token,
                "refresh_token": refresh_token,
                "token_type": "bearer"
            }
        }

    except IntegrityError as e:
        db.rollback()
        # This can happen if email constraint is violated between check and insert
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered"
        )
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create user: {str(e)}"
        )


def signin_user(db: Session, email: str, password: str) -> Dict[str, Any]:
    """
    Authenticate user and return JWT tokens.

    Args:
        db: Database session
        email: User email address
        password: Plain text password

    Returns:
        Dict containing user profile and JWT tokens (same structure as signup_user)

    Raises:
        HTTPException(401): Invalid credentials or inactive account

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     user_session = signin_user(
        >>>         db=db,
        >>>         email="user@example.com",
        >>>         password="SecurePass123"
        >>>     )
    """
    # Find user by email
    user = db.query(AuthUser).filter(AuthUser.email == email).first()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
            headers={"WWW-Authenticate": "Bearer"}
        )

    # Verify password
    if not verify_password(password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
            headers={"WWW-Authenticate": "Bearer"}
        )

    # Check if account is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Account is inactive"
        )

    # Get user profile
    user_profile = db.query(UserProfile).filter(UserProfile.user_id == user.id).first()

    if not user_profile:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="User profile not found"
        )

    # Generate JWT tokens
    token_data = {
        "sub": str(user.id),
        "email": user.email
    }
    access_token = create_access_token(token_data)
    refresh_token = create_refresh_token(token_data)

    # Return user session
    return {
        "user": {
            "user_id": str(user.id),
            "email": user.email,
            "ai_level": user_profile.ai_level,
            "ml_level": user_profile.ml_level,
            "ros_level": user_profile.ros_level,
            "python_level": user_profile.python_level,
            "linux_level": user_profile.linux_level,
            "has_gpu": user_profile.has_gpu,
            "has_jetson": user_profile.has_jetson,
            "has_robot": user_profile.has_robot,
            "cloud_only": user_profile.cloud_only,
            "created_at": user_profile.created_at.isoformat()
        },
        "tokens": {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer"
        }
    }


def refresh_access_token(db: Session, user_id: str) -> Dict[str, str]:
    """
    Generate new access token and refresh token for a user.

    Args:
        db: Database session
        user_id: User UUID string

    Returns:
        Dict containing new access and refresh tokens:
        {
            "access_token": str,
            "refresh_token": str,
            "token_type": "bearer"
        }

    Raises:
        HTTPException(404): User not found
        HTTPException(403): Account inactive

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     new_tokens = refresh_access_token(
        >>>         db=db,
        >>>         user_id="123e4567-e89b-12d3-a456-426614174000"
        >>>     )
    """
    # Find user
    user = db.query(AuthUser).filter(AuthUser.id == user_id).first()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Check if account is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Account is inactive"
        )

    # Generate new tokens
    token_data = {
        "sub": str(user.id),
        "email": user.email
    }
    access_token = create_access_token(token_data)
    refresh_token = create_refresh_token(token_data)

    return {
        "access_token": access_token,
        "refresh_token": refresh_token,
        "token_type": "bearer"
    }
