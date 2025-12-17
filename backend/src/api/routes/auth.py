"""
Authentication API routes for user signup, signin, token refresh, and logout.

Endpoints:
- POST /api/auth/signup - Create new user account
- POST /api/auth/signin - Authenticate user
- POST /api/auth/refresh - Refresh access token
- POST /api/auth/logout - Logout user (client-side token clearing)
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from src.db.connection import get_db
from src.api.models.request import SignupRequest, SigninRequest, RefreshTokenRequest
from src.api.models.response import SignupResponse, SigninResponse, AuthTokensResponse
from src.services.auth_service import signup_user, signin_user, refresh_access_token
from src.utils.jwt import verify_token


router = APIRouter()


@router.post("/signup", response_model=SignupResponse, status_code=status.HTTP_201_CREATED)
def signup(
    request: SignupRequest,
    db: Session = Depends(get_db)
):
    """
    Create a new user account with profile.

    **Request Body:**
    - email: Valid email address
    - password: Strong password (min 8 chars, 1 uppercase, 1 lowercase, 1 number)
    - ai_level: AI knowledge level (1-5)
    - ml_level: ML knowledge level (1-5)
    - ros_level: ROS expertise level (1-5)
    - python_level: Python proficiency (1-5)
    - linux_level: Linux skill (1-5)
    - has_gpu: Has GPU hardware (optional, default: false)
    - has_jetson: Has Jetson hardware (optional, default: false)
    - has_robot: Has robot hardware (optional, default: false)

    **Response (201 Created):**
    - user: User profile data (user_id, email, skill levels, hardware, cloud_only, created_at)
    - tokens: JWT tokens (access_token, refresh_token, token_type)

    **Error Responses:**
    - 409 Conflict: Email already registered
    - 422 Unprocessable Entity: Validation error (invalid password, skill levels out of range)
    - 500 Internal Server Error: Database error

    **Example:**
    ```bash
    curl -X POST http://localhost:8000/api/auth/signup \\
      -H "Content-Type: application/json" \\
      -d '{
        "email": "user@example.com",
        "password": "SecurePass123",
        "ai_level": 3, "ml_level": 4, "ros_level": 2,
        "python_level": 5, "linux_level": 3,
        "has_gpu": true
      }'
    ```
    """
    user_session = signup_user(
        db=db,
        email=request.email,
        password=request.password,
        ai_level=request.ai_level,
        ml_level=request.ml_level,
        ros_level=request.ros_level,
        python_level=request.python_level,
        linux_level=request.linux_level,
        has_gpu=request.has_gpu,
        has_jetson=request.has_jetson,
        has_robot=request.has_robot
    )

    return user_session


@router.post("/signin", response_model=SigninResponse, status_code=status.HTTP_200_OK)
def signin(
    request: SigninRequest,
    db: Session = Depends(get_db)
):
    """
    Authenticate user and return JWT tokens.

    **Request Body:**
    - email: User email address
    - password: User password

    **Response (200 OK):**
    - user: User profile data
    - tokens: JWT tokens (access_token, refresh_token, token_type)

    **Error Responses:**
    - 401 Unauthorized: Invalid email or password
    - 403 Forbidden: Account is inactive
    - 422 Unprocessable Entity: Validation error

    **Example:**
    ```bash
    curl -X POST http://localhost:8000/api/auth/signin \\
      -H "Content-Type: application/json" \\
      -d '{
        "email": "user@example.com",
        "password": "SecurePass123"
      }'
    ```
    """
    user_session = signin_user(
        db=db,
        email=request.email,
        password=request.password
    )

    return user_session


@router.post("/refresh", response_model=AuthTokensResponse, status_code=status.HTTP_200_OK)
def refresh(
    request: RefreshTokenRequest,
    db: Session = Depends(get_db)
):
    """
    Refresh access token using refresh token.

    **Request Body:**
    - refresh_token: Valid JWT refresh token

    **Response (200 OK):**
    - access_token: New JWT access token (15-minute expiry)
    - refresh_token: New JWT refresh token (7-day expiry)
    - token_type: "bearer"

    **Error Responses:**
    - 401 Unauthorized: Invalid or expired refresh token
    - 403 Forbidden: Account is inactive
    - 404 Not Found: User not found

    **Example:**
    ```bash
    curl -X POST http://localhost:8000/api/auth/refresh \\
      -H "Content-Type: application/json" \\
      -d '{
        "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
      }'
    ```

    **Token Rotation:**
    This endpoint implements token rotation - both access and refresh tokens are regenerated.
    The old refresh token becomes invalid after use.
    """
    # Verify refresh token
    payload = verify_token(request.refresh_token, expected_type="refresh")

    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired refresh token",
            headers={"WWW-Authenticate": "Bearer"}
        )

    user_id = payload.get("sub")
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
            headers={"WWW-Authenticate": "Bearer"}
        )

    # Generate new tokens
    new_tokens = refresh_access_token(db=db, user_id=user_id)

    return new_tokens


@router.post("/logout", status_code=status.HTTP_204_NO_CONTENT)
def logout():
    """
    Logout user (client-side token clearing).

    **Note:** This is a stateless backend, so logout is handled client-side by:
    1. Clearing tokens from sessionStorage/localStorage
    2. Clearing HTTP-only cookies (if used)
    3. Calling this endpoint for logging purposes (optional)

    **Response (204 No Content):**
    - Empty response body

    **Token Invalidation:**
    Since JWT tokens are stateless, they remain valid until expiration.
    For immediate invalidation, consider:
    1. Short token expiration times (15 minutes for access tokens)
    2. Token blacklist (not implemented in this stateless design)
    3. Refresh token rotation (implemented in /refresh endpoint)

    **Example:**
    ```bash
    curl -X POST http://localhost:8000/api/auth/logout \\
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
    ```

    **Client-side Logout Example (JavaScript):**
    ```javascript
    async function logout() {
      // Clear tokens from storage
      sessionStorage.removeItem('auth_tokens');

      // Optionally call logout endpoint
      await fetch('http://localhost:8000/api/auth/logout', {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${accessToken}`
        }
      });

      // Redirect to login page
      window.location.href = '/login';
    }
    ```
    """
    # Stateless logout - no server-side action needed
    # Client clears tokens from storage
    return None
