"""
Profile API routes for user profile management.

Endpoints:
- GET /api/profile - Get current user's profile (requires authentication)
"""
from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from src.db.connection import get_db
from src.db.models import AuthUser
from src.api.models.response import UserProfileResponse
from src.api.middleware.auth import get_current_user
from src.services.profile_service import get_profile


router = APIRouter()


@router.get("/profile", response_model=UserProfileResponse)
def get_user_profile(
    current_user: AuthUser = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get current user's profile.

    **Authentication Required:**
    - Requires valid JWT access token in Authorization header

    **Headers:**
    - Authorization: Bearer {access_token}

    **Response (200 OK):**
    - user_id: User UUID
    - email: User email address
    - ai_level: AI knowledge level (1-5)
    - ml_level: ML knowledge level (1-5)
    - ros_level: ROS expertise level (1-5)
    - python_level: Python proficiency (1-5)
    - linux_level: Linux skill (1-5)
    - has_gpu: Has GPU hardware
    - has_jetson: Has Jetson hardware
    - has_robot: Has robot hardware
    - cloud_only: Computed (true if no physical hardware)
    - created_at: Profile creation timestamp (ISO 8601)

    **Error Responses:**
    - 401 Unauthorized: Invalid or expired access token
    - 403 Forbidden: Account is inactive
    - 404 Not Found: Profile not found

    **Example:**
    ```bash
    curl -X GET http://localhost:8000/api/profile \\
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
    ```

    **Response Example:**
    ```json
    {
      "user_id": "123e4567-e89b-12d3-a456-426614174000",
      "email": "user@example.com",
      "ai_level": 3,
      "ml_level": 4,
      "ros_level": 2,
      "python_level": 5,
      "linux_level": 3,
      "has_gpu": true,
      "has_jetson": false,
      "has_robot": false,
      "cloud_only": false,
      "created_at": "2025-12-17T10:30:00"
    }
    ```

    **Use Cases:**
    1. Display user profile in settings page
    2. Determine personalization parameters for chapter content
    3. Check hardware availability for exercise recommendations
    4. Validate user access to specific content sections
    """
    profile_data = get_profile(db=db, user_id=str(current_user.id))
    return profile_data
