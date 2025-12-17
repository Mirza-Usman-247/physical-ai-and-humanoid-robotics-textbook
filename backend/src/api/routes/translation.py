"""
Translation API Routes

Handles chapter translation to Urdu with Focus Mode.
Endpoints:
- POST /api/translate - Translate chapter to Urdu (requires authentication)
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from src.api.models.request import TranslateRequest
from src.api.models.response import TranslateResponse, ErrorResponse
from src.api.middleware.auth import get_current_user
from src.db.connection import get_db
from src.db.models import AuthUser
from src.services.translation_service import translate_chapter

router = APIRouter()


@router.post(
    "/translate",
    response_model=TranslateResponse,
    status_code=status.HTTP_200_OK,
    responses={
        200: {
            "description": "Chapter successfully translated to Urdu",
            "model": TranslateResponse
        },
        400: {
            "description": "Invalid request (unsupported language, invalid content)",
            "model": ErrorResponse
        },
        401: {
            "description": "Not authenticated",
            "model": ErrorResponse
        },
        500: {
            "description": "Translation service error",
            "model": ErrorResponse
        }
    },
    summary="Translate Chapter to Urdu",
    description="""
    Translate entire chapter to Urdu with technical term preservation.

    **Features:**
    - Focus Mode: Faithful translation without extra commentary
    - Technical terms preserved in English or transliterated
    - Code blocks unchanged
    - Markdown formatting preserved
    - Uses free Google Gemini 2.0 Flash model

    **Requirements:**
    - User must be authenticated (JWT token required)
    - Chapter content must be valid markdown (10-50,000 characters)

    **Focus Mode Characteristics:**
    - No additional explanations beyond translation
    - Technical terms remain in English when appropriate
    - 100% fidelity to original meaning
    - Maintains educational tone

    **Translation Time:**
    Typically 10-15 seconds depending on chapter length.
    """
)
def translate(
    request: TranslateRequest,
    current_user: AuthUser = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> TranslateResponse:
    """
    Translate chapter content to Urdu.

    Args:
        request: Translation request with chapter content and focus mode flag
        current_user: Authenticated user from JWT token
        db: Database session

    Returns:
        TranslateResponse with translated content and metadata

    Raises:
        HTTPException 400: Invalid request parameters
        HTTPException 500: Translation service failure
    """
    try:
        # Determine translation mode
        mode = "focus" if request.focus_mode else "standard"

        # Call translation service
        result = translate_chapter(
            chapter_id=request.chapter_id,
            chapter_content=request.chapter_content,
            target_language="ur",  # Urdu
            mode=mode,
            focus_areas=request.focus_areas
        )

        # Build response
        return TranslateResponse(
            chapter_id=result["chapter_id"],
            translated_content=result["translated_content"],
            transformation_metadata=result["translation_metadata"]
        )

    except ValueError as e:
        # Validation errors (unsupported language, invalid mode)
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

    except Exception as e:
        # Translation service errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )


@router.get(
    "/supported-languages",
    response_model=dict,
    status_code=status.HTTP_200_OK,
    summary="Get Supported Languages",
    description="Returns list of supported target languages for translation."
)
def get_supported_languages():
    """
    Get list of supported translation target languages.

    Currently supports:
    - Urdu (ur)

    Future support planned for:
    - Arabic (ar)
    - Hindi (hi)
    - Bengali (bn)

    Returns:
        Dictionary of language codes and names
    """
    return {
        "supported_languages": [
            {
                "code": "ur",
                "name": "Urdu",
                "native_name": "اردو",
                "direction": "rtl",
                "status": "active"
            }
        ],
        "default_mode": "focus",
        "modes": [
            {
                "code": "focus",
                "name": "Focus Mode",
                "description": "Faithful translation with technical term preservation, no extra commentary"
            }
        ]
    }
