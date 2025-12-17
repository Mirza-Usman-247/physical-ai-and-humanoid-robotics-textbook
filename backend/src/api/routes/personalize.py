"""
Personalization API routes for chapter content adaptation.

Endpoints:
- POST /api/personalize - Personalize chapter content based on user profile
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from src.db.connection import get_db
from src.db.models import AuthUser, UserProfile
from src.api.models.request import PersonalizeRequest
from src.api.models.response import PersonalizeResponse
from src.api.middleware.auth import get_current_user
from src.services.personalization_service import personalize_chapter


router = APIRouter()


@router.post("/personalize", response_model=PersonalizeResponse, status_code=status.HTTP_200_OK)
def personalize_content(
    request: PersonalizeRequest,
    current_user: AuthUser = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Personalize chapter content based on user's skill levels and hardware.

    **Authentication Required:**
    - Requires valid JWT access token in Authorization header

    **Headers:**
    - Authorization: Bearer {access_token}

    **Request Body:**
    - chapter_id: Chapter identifier (e.g., "module-1-chapter-3")
    - chapter_content: Original chapter content in markdown format
    - focus_areas: Optional list of topics to emphasize (e.g., ["neural networks", "GPU optimization"])

    **Response (200 OK):**
    - chapter_id: Chapter identifier
    - personalized_content: Adapted chapter content in markdown
    - transformation_metadata: Details about personalization applied
      - skill_adjustments: User's skill levels used for adaptation
      - hardware_recommendations: Hardware-specific recommendations added
      - model: LLM model used (google/gemini-2.0-flash-exp:free)
      - focus_areas: Topics emphasized
      - usage: Token usage statistics

    **Error Responses:**
    - 401 Unauthorized: Invalid or expired access token
    - 404 Not Found: User profile not found
    - 500 Internal Server Error: Personalization service error

    **Example Request:**
    ```bash
    curl -X POST http://localhost:8000/api/personalize \\
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..." \\
      -H "Content-Type: application/json" \\
      -d '{
        "chapter_id": "module-1-chapter-3",
        "chapter_content": "# Neural Networks\\n\\nNeural networks are...",
        "focus_areas": ["GPU acceleration", "PyTorch"]
      }'
    ```

    **Example Response:**
    ```json
    {
      "chapter_id": "module-1-chapter-3",
      "personalized_content": "# Neural Networks (Intermediate Level)\\n\\n...",
      "transformation_metadata": {
        "skill_adjustments": {
          "ai_level": 3,
          "ml_level": 4,
          "ros_level": 2,
          "python_level": 5,
          "linux_level": 3
        },
        "hardware_recommendations": [
          "GPU-accelerated training exercises included",
          "Cloud/simulation alternatives provided"
        ],
        "model": "google/gemini-2.0-flash-exp:free",
        "focus_areas": ["GPU acceleration", "PyTorch"],
        "usage": {
          "prompt_tokens": 1234,
          "completion_tokens": 567,
          "total_tokens": 1801
        }
      }
    }
    ```

    **Use Cases:**
    1. User clicks "Personalize Chapter" button in textbook
    2. Content is adapted to their AI/ML/ROS skill levels
    3. Hardware-specific exercises are added/removed
    4. Code examples match their Python proficiency
    5. Personalized content is stored in IndexedDB (client-side)

    **Notes:**
    - Personalization uses free Gemini 2.0 Flash model (no cost per request)
    - Results are NOT stored in database (session-only, per ADR-006)
    - Users can re-personalize same chapter with different focus areas
    - Original content is always preserved (no destructive changes)
    """
    # Get user profile
    user_profile = db.query(UserProfile).filter(UserProfile.user_id == current_user.id).first()

    if not user_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found. Please complete your profile setup."
        )

    try:
        # Personalize chapter content
        result = personalize_chapter(
            chapter_id=request.chapter_id,
            chapter_content=request.chapter_content,
            user_profile=user_profile,
            focus_areas=request.focus_areas
        )

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Personalization failed: {str(e)}"
        )
