"""
Translation Service

Handles chapter translation to Urdu using OpenRouter's free Gemini model.
Implements "Focus Mode" - faithful translation with technical term preservation.

Features:
- Translates educational content to Urdu
- Preserves technical terms in English or provides transliteration
- Maintains code blocks unchanged
- Preserves markdown formatting
- Supports multiple target languages (currently Urdu)
- Retry logic with exponential backoff for rate limiting
"""

from typing import Dict, Any, Optional, List
import time
from openai import OpenAI

from src.config import settings
from src.db.models import UserProfile

# Initialize OpenRouter client
openrouter_client = OpenAI(
    base_url=settings.openrouter_base_url,
    api_key=settings.openrouter_api_key,
)


def translate_chapter(
    chapter_id: str,
    chapter_content: str,
    target_language: str = "ur",  # ISO 639-1 code for Urdu
    mode: str = "focus",  # "focus" mode: faithful translation, no commentary
    focus_areas: Optional[List[str]] = None,  # Optional topics to emphasize
) -> Dict[str, Any]:
    """
    Translate chapter content to target language using Focus Mode.

    Args:
        chapter_id: Unique identifier for the chapter
        chapter_content: Original chapter content in English
        target_language: Target language code (default: "ur" for Urdu)
        mode: Translation mode (default: "focus" for faithful translation)
        focus_areas: Optional list of topics to emphasize in translation

    Returns:
        Dictionary containing:
        - chapter_id: Original chapter ID
        - translated_content: Translated markdown content
        - target_language: Target language code
        - translation_metadata: Metadata about the translation

    Raises:
        ValueError: If target language is not supported
        Exception: If translation API call fails
    """
    # Validate target language
    supported_languages = {"ur": "Urdu"}
    if target_language not in supported_languages:
        raise ValueError(f"Unsupported target language: {target_language}. Supported: {list(supported_languages.keys())}")

    language_name = supported_languages[target_language]

    # Build translation prompt based on mode
    if mode == "focus":
        prompt = _build_focus_mode_prompt(chapter_content, language_name, focus_areas)
    else:
        raise ValueError(f"Unsupported translation mode: {mode}")

    # Retry logic with exponential backoff for rate limiting
    max_retries = 3
    retry_delay = 2  # Start with 2 seconds

    for attempt in range(max_retries):
        try:
            # Call OpenRouter with free Gemini model
            response = openrouter_client.chat.completions.create(
                model=settings.openrouter_llm_model_translate,  # Free Gemini model
                messages=[
                    {
                        "role": "system",
                        "content": (
                            f"You are an expert technical translator specializing in educational content. "
                            f"Your task is to translate technical educational material to {language_name} "
                            f"with 100% fidelity. CRITICAL: Output ONLY the translated content. "
                            f"Do NOT add any commentary, explanations, or meta-text."
                        )
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=settings.max_output_tokens,
                temperature=0.2,  # Lower temperature for consistent, faithful translation
            )

            translated_content = response.choices[0].message.content

            # Extract preserved technical terms (for metadata)
            preserved_terms = _extract_technical_terms(chapter_content)

            # Build response
            return {
                "chapter_id": chapter_id,
                "translated_content": translated_content,
                "target_language": target_language,
                "translation_metadata": {
                    "source_language": "en",
                    "target_language": target_language,
                    "target_language_name": language_name,
                    "mode": mode,
                    "model_used": settings.openrouter_llm_model_translate,
                    "preserved_terms_count": len(preserved_terms),
                    "preserved_terms_sample": preserved_terms[:10],  # First 10 terms
                    "original_length": len(chapter_content),
                    "translated_length": len(translated_content),
                    "retries": attempt,
                    "focus_areas": focus_areas if focus_areas else [],
                }
            }

        except Exception as e:
            error_message = str(e)

            # Check if it's a rate limit error
            if "429" in error_message or "rate" in error_message.lower():
                if attempt < max_retries - 1:
                    # Wait with exponential backoff before retrying
                    print(f"Rate limited on attempt {attempt + 1}/{max_retries}. Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                    continue
                else:
                    # All retries exhausted
                    raise Exception(
                        f"Translation service is temporarily unavailable due to rate limiting. "
                        f"Please try again in a few minutes. (Attempts: {max_retries})"
                    )
            else:
                # Non-rate-limit error, don't retry
                raise Exception(f"Translation failed: {error_message}")

    # Should never reach here, but just in case
    raise Exception("Translation failed after all retry attempts")


def _build_focus_mode_prompt(content: str, language_name: str, focus_areas: Optional[List[str]] = None) -> str:
    """
    Build Focus Mode translation prompt.

    Focus Mode characteristics:
    - Faithful translation only
    - No additional explanations or commentary
    - Preserve technical terms in English
    - Preserve code blocks unchanged
    - Maintain markdown formatting
    - Optional focus areas for emphasis
    """
    # Build focus areas instruction if provided
    focus_instruction = ""
    if focus_areas and len(focus_areas) > 0:
        areas_list = ", ".join(f'"{area}"' for area in focus_areas)
        focus_instruction = f"""

7. **Focus Areas - Pay Special Attention To:**
   The user has specifically requested focus on these topics: {areas_list}
   - Ensure these topics are translated with extra clarity and precision
   - Add brief explanatory notes in {language_name} for these specific topics if needed
   - Maintain technical accuracy while making these areas especially accessible
"""

    prompt = f"""Translate the following educational content to {language_name} with 100% accuracy.

**CRITICAL INSTRUCTIONS - YOU MUST FOLLOW THESE EXACTLY:**

1. **Output Format**: Return ONLY the translated content. Do NOT add:
   - Introductory phrases like "Here is the translation:"
   - Commentary or explanations
   - Meta-text about the translation process
   - Any text that wasn't in the original

2. **Technical Term Preservation**:
   - Keep technical terms in English when they are commonly used internationally
   - Examples to preserve: "Physical AI", "ROS 2", "Gazebo", "Python", "GPU", "API", "HTTP", "REST"
   - For terms that benefit from translation, provide: "Original Term (اردو ترجمہ)"
   - Use this format: "Machine Learning (مشین لرننگ)"

3. **Code Block Preservation**:
   - Keep ALL code blocks EXACTLY as they appear
   - Do NOT translate code, comments inside code, or variable names
   - Preserve code fence markers (```)
   - Keep programming language specifiers (```python, ```bash, etc.)

4. **Markdown Formatting**:
   - Preserve ALL markdown syntax: #, ##, -, *, [], (), etc.
   - Keep links in English: [link text](url)
   - Preserve image syntax: ![alt text](image-url)
   - Maintain table structure

5. **Translation Quality**:
   - Use formal, educational {language_name}
   - Maintain the technical accuracy of the original
   - Preserve the structure and flow of the content
   - Keep numbered lists, bullet points, and formatting intact

6. **Examples to Preserve in English**:
   - File paths: /usr/bin, ~/documents
   - Commands: ros2 run, python script.py
   - Package names: numpy, tensorflow, ros2
   - URLs and domain names
   - Version numbers: Python 3.10, ROS 2 Humble
   - Acronyms: AI, ML, GPU, CPU, API, SDK
{focus_instruction}
**ORIGINAL CONTENT TO TRANSLATE:**

{content}

**YOUR RESPONSE (translated content only, no other text):**"""

    return prompt


def _extract_technical_terms(content: str) -> List[str]:
    """
    Extract technical terms from content for metadata.

    This is a simple extraction for metadata purposes.
    Returns common technical terms found in the content.
    """
    # Common technical terms in Physical AI education
    common_terms = [
        "Physical AI", "ROS", "ROS 2", "Gazebo", "Isaac Sim", "Python",
        "GPU", "CPU", "API", "REST", "HTTP", "HTTPS", "JSON", "YAML",
        "Machine Learning", "Deep Learning", "Neural Network", "Reinforcement Learning",
        "Simulation", "Robot", "Sensor", "Actuator", "Control System",
        "Ubuntu", "Linux", "Docker", "Kubernetes", "Git", "GitHub",
        "TensorFlow", "PyTorch", "NumPy", "OpenCV", "PCL",
        "SLAM", "Navigation", "Perception", "Planning", "Control",
        "Lidar", "Camera", "IMU", "Odometry", "Kinematics", "Dynamics",
    ]

    found_terms = []
    content_lower = content.lower()

    for term in common_terms:
        if term.lower() in content_lower:
            found_terms.append(term)

    return found_terms


def translate_with_context(
    chapter_id: str,
    chapter_content: str,
    user_profile: Optional[UserProfile] = None,
    target_language: str = "ur",
) -> Dict[str, Any]:
    """
    Translate chapter with optional user context.

    This is an advanced version that can adjust translation style
    based on user's skill level (future enhancement).

    Args:
        chapter_id: Unique identifier for the chapter
        chapter_content: Original chapter content
        user_profile: Optional user profile for context-aware translation
        target_language: Target language code

    Returns:
        Translation response dictionary
    """
    # For now, just call the basic translation
    # Future: Adjust formality/complexity based on user_profile skill levels
    return translate_chapter(
        chapter_id=chapter_id,
        chapter_content=chapter_content,
        target_language=target_language,
        mode="focus"
    )
