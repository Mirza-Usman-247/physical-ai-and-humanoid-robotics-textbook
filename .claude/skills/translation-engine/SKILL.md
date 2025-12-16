# Translation Engine Skill

## Summary

The **translation-engine** skill performs precise English ↔ Urdu translation for Physical AI and Humanoid Robotics educational content. It preserves technical accuracy, maintains markdown formatting, and ensures robotics/AI terminology remains consistent across both languages.

**Core Function:** Translate text between English and Urdu while preserving meaning, technical terms, and formatting.

## Inputs

The skill accepts the following inputs:

1. **Source Text** (required)
   - The content to translate
   - Can be plain text or markdown formatted
   - May contain technical robotics/AI terminology
   - Can be single sentence or full chapters

2. **Direction** (required)
   - `english-to-urdu` or `urdu-to-english`
   - Must be explicitly specified

3. **Content Type** (optional)
   - `textbook-chapter` - Educational content with examples
   - `specification` - Technical specs and requirements
   - `notes` - Student notes or explanations
   - `documentation` - Docusaurus pages or docs
   - Default: `general`

4. **Preserve Formatting** (optional)
   - `true` - Keep markdown syntax (headings, lists, code blocks)
   - `false` - Plain text output
   - Default: `true`

## Outputs

The skill produces:

1. **Translated Text**
   - Accurate translation in target language
   - Technical terms preserved or appropriately translated
   - Markdown formatting maintained (if requested)

2. **Translation Notes** (when applicable)
   - Technical terms kept in English with Urdu explanation
   - Ambiguous terms that needed interpretation
   - Cultural or contextual adaptations made

## Behaviour Rules

### Core Principles

1. **Accuracy Over Style**
   - Preserve exact meaning from source
   - Do not add explanations or simplifications
   - Do not change tone or difficulty level
   - Translate literally unless culturally inappropriate

2. **Technical Term Handling**
   - Keep English technical terms when widely used in Urdu robotics education
   - Provide Urdu transliteration for key concepts
   - Examples: "robot", "sensor", "actuator", "AI" typically stay in English
   - For terms like "forward kinematics" → keep English + add Urdu explanation in parentheses

3. **Formatting Preservation**
   - Maintain markdown structure exactly
   - Keep code blocks in original language (usually English)
   - Preserve heading hierarchy
   - Maintain list formatting and indentation

4. **No Content Modification**
   - DO NOT simplify or expand
   - DO NOT change examples
   - DO NOT rewrite for different audience
   - DO NOT add personal opinions or context

### Translation Standards

**For English → Urdu:**
- Use formal Urdu appropriate for technical education
- Write in Urdu script (not Roman Urdu)
- Keep mathematical notation unchanged
- Preserve URLs and file paths exactly

**For Urdu → English:**
- Translate to clear, technical English
- Maintain academic tone
- Keep any English technical terms that were in original
- Preserve mathematical and code elements

## When to Use This Skill

Use the **translation-engine** skill when you need to:

1. **Translate Textbook Content**
   - Converting Physical AI chapters from English to Urdu
   - Making bilingual versions of robotics lessons
   - Creating Urdu versions of educational materials

2. **Convert Documentation**
   - Translating Docusaurus pages for bilingual site
   - Converting README files between languages
   - Translating technical specifications

3. **Process Student Materials**
   - Converting Urdu notes to English for publication
   - Translating English specs to Urdu for students
   - Making course materials accessible in both languages

4. **Maintain Consistency**
   - Ensuring terminology is consistent across translations
   - Creating parallel content for bilingual learners
   - Preserving technical accuracy across languages

## When NOT to Use This Skill

**DO NOT** use this skill when the request involves:

1. **Tone Changes**
   - "Make this friendlier" → Use personalization-engine
   - "Rewrite in expert tone" → Use personalization-engine
   - "Make it more formal" → Use personalization-engine

2. **Content Modification**
   - "Simplify this for beginners" → Use personalization-engine
   - "Expand this explanation" → Use personalization-engine
   - "Make this more detailed" → Use personalization-engine

3. **Rewriting Tasks**
   - "Improve the clarity" → Use personalization-engine
   - "Shorten this text" → Use personalization-engine
   - "Add more examples" → Use personalization-engine

4. **Non-Translation Work**
   - Code refactoring → General coding tasks
   - Creating new content → Content creation tasks
   - Answering questions → General assistance

**Rule of Thumb:** If the request involves changing MEANING, STYLE, or LEVEL → DO NOT USE this skill. This skill ONLY changes LANGUAGE.

## Step-by-Step Process

### Phase 1: Input Validation

1. **Verify Translation Direction**
   - Confirm source and target language
   - Detect language if not specified
   - Error if direction is unclear

2. **Analyze Content Type**
   - Identify if textbook, spec, notes, or documentation
   - Check for markdown formatting
   - Note technical term density

3. **Check Formatting Requirements**
   - Determine if markdown should be preserved
   - Identify code blocks to keep unchanged
   - Note special formatting elements

### Phase 2: Translation

1. **Segment Content**
   - Break into logical sections (paragraphs, headings, lists)
   - Identify technical terms that should remain in English
   - Mark code blocks and URLs to preserve

2. **Translate Each Segment**
   - Translate text accurately
   - Preserve technical terminology appropriately
   - Maintain academic/educational tone
   - Keep markdown syntax intact

3. **Handle Special Elements**
   - Keep code blocks unchanged
   - Preserve URLs and file paths
   - Maintain mathematical notation
   - Translate image alt-text but keep file paths

### Phase 3: Quality Assurance

1. **Verify Accuracy**
   - Check meaning preservation
   - Ensure technical terms are appropriate
   - Confirm tone matches original

2. **Check Formatting**
   - Validate markdown renders correctly
   - Ensure code blocks are intact
   - Verify list and heading structure

3. **Final Review**
   - Read through for natural flow
   - Check for consistent terminology
   - Note any translation decisions made

### Phase 4: Output Delivery

1. **Present Translated Text**
   - Provide complete translation
   - Maintain original structure

2. **Add Translation Notes** (if needed)
   - List technical terms kept in English
   - Explain any ambiguous translation choices
   - Note cultural adaptations made

## File Operations

### Reading Source Files

When translating file contents:

1. **Read the source file**
   ```
   Read file at specified path
   Extract text content
   Preserve file structure and formatting
   ```

2. **Identify file type**
   - `.md` files → Preserve markdown
   - `.txt` files → Plain text translation
   - Docusaurus pages → Preserve frontmatter and formatting

### Writing Translated Files

When saving translations:

1. **Use appropriate naming**
   - English file: `chapter-1-intro.md`
   - Urdu version: `chapter-1-intro.ur.md`
   - Or separate directory: `ur/chapter-1-intro.md`

2. **Preserve structure**
   - Keep same directory structure
   - Maintain frontmatter in Docusaurus files
   - Preserve all metadata

3. **Create side-by-side versions**
   - Original in `/en/` directory
   - Translation in `/ur/` directory
   - Link between versions if needed

## Example Usage

### Example 1: Textbook Chapter Translation

**User Request:**
> Translate this robotics chapter from English to Urdu

**Source Text:**
```markdown
# Chapter 1: Introduction to Humanoid Robots

Humanoid robots are designed to mimic human form and behavior. They typically have a head, torso, two arms, and two legs.

## Key Components
- **Sensors**: Cameras, IMUs, force sensors
- **Actuators**: Motors that enable movement
- **Controller**: Brain of the robot
```

**Skill Output:**
```markdown
# باب 1: Humanoid Robots کا تعارف

Humanoid robots انسانی شکل اور رویے کی نقل کرنے کے لیے ڈیزائن کیے گئے ہیں۔ ان میں عام طور پر ایک سر، دھڑ، دو بازو اور دو ٹانگیں ہوتی ہیں۔

## اہم اجزاء
- **Sensors**: Cameras، IMUs، force sensors
- **Actuators**: Motors جو حرکت کو ممکن بناتے ہیں
- **Controller**: Robot کا دماغ
```

**Translation Notes:**
- Technical terms (Humanoid robots, Sensors, Actuators, etc.) kept in English as commonly used in Urdu robotics education
- Structure and formatting preserved exactly
- Academic tone maintained

### Example 2: Urdu Notes to English

**User Request:**
> Translate these Urdu student notes to English

**Source Text:**
```
Forward Kinematics روبوٹ کے joints کی positions سے end-effector کی position calculate کرنے کا طریقہ ہے۔ یہ direct calculation ہے۔
```

**Skill Output:**
```
Forward Kinematics is the method of calculating the end-effector position from the robot's joint positions. This is a direct calculation.
```

**Translation Notes:**
- Technical term "Forward Kinematics" preserved from original
- Other English technical terms maintained
- Clean technical English output

### Example 3: Specification Translation

**User Request:**
> Translate this feature spec from English to Urdu, preserve markdown

**Source Text:**
```markdown
## Requirements

1. Robot must detect obstacles within 2 meters
2. Response time: < 100ms
3. Accuracy: ≥ 95%
```

**Skill Output:**
```markdown
## ضروریات

1. Robot کو 2 میٹر کے اندر رکاوٹوں کا پتہ لگانا ضروری ہے
2. Response time: < 100ms
3. Accuracy: ≥ 95%
```

**Translation Notes:**
- Technical measurements kept unchanged (2 meters, 100ms, 95%)
- Markdown structure preserved
- Technical terms (Robot, Response time, Accuracy) kept in English

## Error Handling

### Invalid Input Errors

**Error:** Translation direction not specified
```
ERROR: Please specify translation direction:
  - english-to-urdu
  - urdu-to-english
```

**Error:** Empty or missing source text
```
ERROR: No source text provided for translation.
Please provide the text to translate.
```

**Error:** Unsupported language pair
```
ERROR: This skill only supports English ↔ Urdu translation.
For other languages, please use a different tool.
```

### Content Errors

**Error:** Request includes personalization
```
ERROR: This skill ONLY translates language.
Your request includes tone/style changes: "make it friendlier"

For personalization tasks, use: personalization-engine skill
For translation only, rephrase your request without style requirements.
```

**Error:** Request asks for content modification
```
ERROR: This skill does NOT modify content.
Your request asks to: "simplify for beginners"

For content modification, use: personalization-engine skill
For translation only, provide text to translate without modification requests.
```

### Processing Warnings

**Warning:** Technical term translation ambiguous
```
WARNING: Technical term "kinematics" has multiple Urdu equivalents.
Using: Keep in English with Urdu explanation
Alternatives considered: حرکیات، حرکاتی علم
```

**Warning:** Cultural adaptation made
```
WARNING: Idiom translated for cultural context
Original: "hit the ground running"
Translated to equivalent Urdu concept rather than literal
```

### Recovery Actions

1. **Clarify ambiguous requests**
   - Ask user to specify translation direction
   - Request confirmation on technical term handling
   - Verify if personalization is needed (if so, redirect to other skill)

2. **Handle partial content**
   - If some sections can't be translated, translate what's possible
   - Mark untranslatable sections clearly
   - Provide explanation for limitations

3. **Formatting issues**
   - If markdown breaks during translation, fix structure
   - Preserve as much formatting as possible
   - Note any formatting changes in output

## Version

**Skill Version:** 1.0.0
**Created:** 2025-12-09
**Last Updated:** 2025-12-09
**Compatibility:** Claude Code CLI
**Author:** Custom Skill for Physical AI & Humanoid Robotics Textbook Project
