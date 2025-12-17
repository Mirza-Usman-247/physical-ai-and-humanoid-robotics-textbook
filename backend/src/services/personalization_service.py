"""
Personalization service for adapting chapter content to user skill levels.

Uses OpenRouter API with free Gemini 2.0 Flash model for content personalization.
"""
from typing import Dict, Any, Optional
from openai import OpenAI
import logging

from src.config import get_settings
from src.db.models import UserProfile

logger = logging.getLogger(__name__)
settings = get_settings()


# Initialize OpenRouter client
openrouter_client = OpenAI(
    base_url=settings.openrouter_base_url,
    api_key=settings.openrouter_api_key,
)


def personalize_chapter(
    chapter_id: str,
    chapter_content: str,
    user_profile: UserProfile,
    focus_areas: Optional[list[str]] = None
) -> Dict[str, Any]:
    """
    Personalize chapter content based on user's skill levels and hardware access.

    Args:
        chapter_id: Chapter identifier (e.g., "module-1-chapter-3")
        chapter_content: Original chapter content (markdown)
        user_profile: User profile with skill levels and hardware flags
        focus_areas: Optional list of topics to emphasize

    Returns:
        Dict containing:
        {
            "chapter_id": str,
            "personalized_content": str (markdown),
            "transformation_metadata": {
                "skill_adjustments": {...},
                "hardware_recommendations": [...],
                "model": str,
                "timestamp": str
            }
        }

    Example:
        >>> profile = UserProfile(ai_level=3, ml_level=4, has_gpu=True, ...)
        >>> result = personalize_chapter(
        >>>     chapter_id="module-1-chapter-3",
        >>>     chapter_content="# Chapter 3: Neural Networks\\n...",
        >>>     user_profile=profile
        >>> )
        >>> print(result["personalized_content"])
    """
    logger.info(f"Personalizing chapter {chapter_id} for user with AI level {user_profile.ai_level}")

    # Build personalization prompt based on user profile
    prompt = _build_personalization_prompt(chapter_content, user_profile, focus_areas)

    try:
        # Call OpenRouter API with free Gemini model
        response = openrouter_client.chat.completions.create(
            model=settings.openrouter_llm_model_personalize,
            messages=[
                {
                    "role": "system",
                    "content": "You are an expert AI/robotics educator who adapts technical content to match student skill levels and available hardware."
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            max_tokens=settings.max_output_tokens,
            temperature=0.3,  # Lower temperature for more consistent educational content
        )

        personalized_content = response.choices[0].message.content

        # Build transformation metadata
        metadata = {
            "skill_adjustments": {
                "ai_level": user_profile.ai_level,
                "ml_level": user_profile.ml_level,
                "ros_level": user_profile.ros_level,
                "python_level": user_profile.python_level,
                "linux_level": user_profile.linux_level,
            },
            "hardware_recommendations": _get_hardware_recommendations(user_profile),
            "model": settings.openrouter_llm_model_personalize,
            "focus_areas": focus_areas or [],
            "usage": {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens,
            }
        }

        return {
            "chapter_id": chapter_id,
            "personalized_content": personalized_content,
            "transformation_metadata": metadata
        }

    except Exception as e:
        logger.error(f"Personalization failed for chapter {chapter_id}: {str(e)}")
        # Return original content with error metadata
        return {
            "chapter_id": chapter_id,
            "personalized_content": chapter_content,
            "transformation_metadata": {
                "error": str(e),
                "fallback": "original_content"
            }
        }


def _build_personalization_prompt(
    content: str,
    profile: UserProfile,
    focus_areas: Optional[list[str]] = None
) -> str:
    """
    Build personalization prompt based on user profile.

    Args:
        content: Original chapter content
        profile: User profile with skill levels
        focus_areas: Optional topics to emphasize

    Returns:
        Personalization prompt for LLM
    """
    # Skill level descriptions
    skill_labels = {1: "Beginner", 2: "Basic", 3: "Intermediate", 4: "Advanced", 5: "Expert"}

    # Hardware context
    hardware_context = []
    if profile.has_gpu:
        hardware_context.append("NVIDIA GPU for training")
    if profile.has_jetson:
        hardware_context.append("NVIDIA Jetson for edge deployment")
    if profile.has_robot:
        hardware_context.append("Physical robot for testing")
    if profile.cloud_only:
        hardware_context.append("Cloud-only environment (no local hardware)")

    hardware_str = ", ".join(hardware_context) if hardware_context else "Cloud-only"

    # Build prompt
    prompt = f"""Personalize the following robotics/AI chapter content for a student with these characteristics:

**Student Skill Levels:**
- AI/ML Knowledge: {skill_labels[profile.ai_level]} (Level {profile.ai_level}/5)
- Machine Learning: {skill_labels[profile.ml_level]} (Level {profile.ml_level}/5)
- ROS Expertise: {skill_labels[profile.ros_level]} (Level {profile.ros_level}/5)
- Python Proficiency: {skill_labels[profile.python_level]} (Level {profile.python_level}/5)
- Linux Skills: {skill_labels[profile.linux_level]} (Level {profile.linux_level}/5)

**Available Hardware:**
{hardware_str}

**Personalization Instructions:**
1. Adjust technical depth to match skill levels (simplify for beginners, add detail for experts)
2. Provide practical exercises matching available hardware:
   - GPU exercises if they have GPU
   - Jetson deployment examples if they have Jetson
   - Physical robot code if they have a robot
   - Cloud/simulation alternatives for cloud-only users
3. Add skill-appropriate code examples:
   - Beginner: Well-commented, step-by-step
   - Intermediate: Balanced comments, some advanced patterns
   - Advanced/Expert: Minimal comments, production-ready patterns
4. Include relevant prerequisites or skip basics based on skill level
5. Maintain markdown formatting and preserve all headings/structure
"""

    if focus_areas:
        prompt += f"\n**Focus Areas (emphasize these topics):**\n"
        for area in focus_areas:
            prompt += f"- {area}\n"

    prompt += f"\n**Original Chapter Content:**\n\n{content}\n\n**Personalized Content:**"

    return prompt


def _get_hardware_recommendations(profile: UserProfile) -> list[str]:
    """
    Generate hardware-specific recommendations based on user profile.

    Args:
        profile: User profile with hardware flags

    Returns:
        List of hardware recommendation strings
    """
    recommendations = []

    if profile.has_gpu:
        recommendations.append("GPU-accelerated training exercises included")
    if profile.has_jetson:
        recommendations.append("Jetson deployment examples provided")
    if profile.has_robot:
        recommendations.append("Physical robot testing instructions included")
    if profile.cloud_only:
        recommendations.append("Cloud/simulation alternatives provided")

    return recommendations or ["General exercises for all hardware configurations"]
