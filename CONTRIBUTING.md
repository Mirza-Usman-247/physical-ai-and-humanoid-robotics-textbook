# Contributing to Physical AI & Humanoid Robotics Textbook

Thank you for your interest in contributing to this open educational resource! This document provides guidelines for contributing content, code examples, fixes, and improvements.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How to Contribute](#how-to-contribute)
- [Chapter Creation Workflow](#chapter-creation-workflow)
- [Style Guide](#style-guide)
- [Code Examples](#code-examples)
- [Pull Request Process](#pull-request-process)
- [Reporting Issues](#reporting-issues)

---

## Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](https://www.contributor-covenant.org/). By participating, you are expected to uphold this code. Please report unacceptable behavior to the project maintainers.

**Key Principles**:
- Be respectful and inclusive
- Welcome newcomers and beginners
- Focus on constructive feedback
- Prioritize educational value over complexity

---

## How to Contribute

### Types of Contributions

We welcome the following contributions:

1. **Content Improvements**
   - Fix typos, grammar, or technical errors
   - Clarify explanations or add examples
   - Update outdated information or links

2. **New Code Examples**
   - Add alternative implementations
   - Contribute examples for different platforms (Jetson, cloud, etc.)
   - Add unit tests for existing examples

3. **Diagrams and Visualizations**
   - Improve existing Mermaid diagrams
   - Add new diagrams for complex concepts
   - Create interactive visualizations

4. **Glossary Additions**
   - Add missing technical terms
   - Improve definitions
   - Add cross-references between related terms

5. **Translations**
   - Translate content to other languages
   - Maintain consistency with English version

6. **Bug Fixes**
   - Fix broken links
   - Correct code errors
   - Resolve build issues

---

## Chapter Creation Workflow

If you're contributing a new chapter or major section, follow this structured workflow:

### 1. Plan Your Chapter

- **Identify Topic**: Choose a topic that fits within the textbook structure
- **Check Existing Content**: Ensure no duplication with existing chapters
- **Define Learning Objectives**: List 3-5 clear learning outcomes
- **Outline Sections**: Plan 12 sections following the chapter template

### 2. Create Chapter File

**File Location**: `docs/module-X-name/chapter-Y-title.mdx`

**Frontmatter Template**:
```yaml
---
sidebar_position: Y
title: Chapter Y - Your Title
description: Brief chapter description (1-2 sentences)
keywords: [keyword1, keyword2, keyword3, robotics, physical-ai]
---
```

### 3. Chapter Structure (12 Sections Required)

All chapters must follow this 12-section structure:

1. **Introduction** - Overview and motivation
2. **Theoretical Foundations** - Core concepts and math
3. **Architecture/Framework** - System design
4. **Implementation Details** - Technical specifics
5. **Code Example** - Working implementation
6. **Practical Applications** - Real-world use cases
7. **Best Practices** - Dos and don'ts
8. **Common Pitfalls** - Troubleshooting
9. **Advanced Topics** - Extensions and variants
10. **Tools and Libraries** - Ecosystem overview
11. **Exercises** - Practice problems
12. **Summary** - Key takeaways

### 4. Add Code Examples

**Location**: `examples/module-X-name/chapter-Y-example.py`

**Requirements**:
- Include docstrings for all functions
- Add type hints (Python 3.10+)
- Include requirements.txt for dependencies
- Test code before submitting
- Keep examples under 200 lines (split if longer)

### 5. Create Diagrams

**Location**: `static/img/module-X-name/chapter-Y-diagram-name.mmd`

**Requirements**:
- Use Mermaid syntax (flowchart, sequence, class diagrams)
- Minimum 2 diagrams per chapter
- Include descriptive alt-text
- Keep diagrams focused (max 15 nodes)

### 6. Update Glossary

**Location**: `docs/glossary.md`

**Requirements**:
- Add 5-10 new terms per chapter
- Follow alphabetical ordering
- Include category tags (robotics, ai, mathematics, etc.)
- Cross-reference related terms

---

## Style Guide

### Markdown Formatting

- Use `#` for chapter titles (H1)
- Use `##` for major sections (H2)
- Use `###` for subsections (H3)
- Use **bold** for important terms on first mention
- Use `code` for inline code, commands, file paths
- Use triple backticks for code blocks with language identifier

**Example**:
````markdown
## Forward Kinematics

**Forward kinematics** computes the position of the end-effector given joint angles.

```python
def forward_kinematics(joint_angles: np.ndarray) -> np.ndarray:
    """Compute end-effector pose from joint angles."""
    return transformation_matrix
```
````

### Technical Writing

- **Be Concise**: Aim for clarity over verbosity
- **Use Active Voice**: "The robot moves" not "The robot is moved"
- **Define Acronyms**: First mention: "Robot Operating System (ROS)"
- **Provide Context**: Explain "why" not just "how"
- **Include Units**: "10 kg" not "10", "5 ms" not "5"

### Mathematical Notation

- Use LaTeX for equations: `$f(x) = mx + b$`
- Reference `docs/notation.md` for standard notation
- Define all variables before using them
- Number important equations

**Example**:
```markdown
The forward kinematics equation is:

$$
^0T_n = \prod_{i=1}^{n} A_i(\theta_i)
$$

where $^0T_n$ is the transformation from base to end-effector.
```

---

## Code Examples

### Python Style

Follow [PEP 8](https://pep8.org/) and these additional guidelines:

- **Type Hints**: Use for all function parameters and return values
- **Docstrings**: Google-style docstrings
- **Imports**: Group stdlib, third-party, local (separated by blank lines)
- **Line Length**: Max 100 characters
- **ROS 2 Integration**: Use `rclpy` for all ROS examples

**Example**:
```python
#!/usr/bin/env python3
"""
Forward kinematics solver for 2-DOF robot arm.

This module implements DH parameter-based forward kinematics
for educational purposes in Physical AI textbook Chapter 4.
"""

import numpy as np
from typing import Tuple


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Compute homogeneous transformation matrix from DH parameters.

    Args:
        theta: Joint angle (radians)
        d: Link offset (meters)
        a: Link length (meters)
        alpha: Link twist (radians)

    Returns:
        4x4 homogeneous transformation matrix

    Example:
        >>> T = dh_transform(np.pi/2, 0, 0.5, 0)
        >>> print(T.shape)
        (4, 4)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])
```

### Testing Code Examples

Before submitting, test your code:

1. **Install Dependencies**:
   ```bash
   pip install -r examples/module-X-name/requirements.txt
   ```

2. **Run Example**:
   ```bash
   python examples/module-X-name/chapter-Y-example.py
   ```

3. **Check Linting**:
   ```bash
   flake8 examples/module-X-name/chapter-Y-example.py
   mypy examples/module-X-name/chapter-Y-example.py
   ```

---

## Pull Request Process

### 1. Fork and Branch

```bash
git clone https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook.git
cd Humanoid-physical-ai-textbook
git checkout -b feature/your-contribution-name
```

### 2. Make Changes

- Follow style guide and chapter template
- Test all code examples
- Update relevant documentation
- Add/update glossary terms

### 3. Commit Changes

Use [Conventional Commits](https://www.conventionalcommits.org/):

```bash
git add docs/module-1-ros2/chapter-6-new-topic.mdx
git commit -m "feat(module-1): add chapter 6 on advanced ROS 2 topics

- Add 12-section chapter on ROS 2 advanced patterns
- Include code examples for custom executors
- Add 2 Mermaid diagrams (executor architecture, callback groups)
- Update glossary with 8 new terms"
```

**Commit Types**:
- `feat:` New chapter, section, or major feature
- `fix:` Bug fix, typo, broken link
- `docs:` Documentation improvements
- `style:` Formatting changes (no code logic change)
- `refactor:` Code restructuring (no behavior change)
- `test:` Add or update tests
- `chore:` Build process, dependencies

### 4. Push and Create PR

```bash
git push origin feature/your-contribution-name
```

Then create a Pull Request on GitHub with:

**PR Title**: `[Module X] Your contribution summary`

**PR Description Template**:
```markdown
## Description
Brief description of changes.

## Type of Change
- [ ] New chapter/content
- [ ] Code example addition/fix
- [ ] Diagram improvement
- [ ] Glossary update
- [ ] Bug fix
- [ ] Documentation improvement

## Checklist
- [ ] Follows chapter template (12 sections)
- [ ] Code examples tested and working
- [ ] Diagrams render correctly
- [ ] Glossary updated with new terms
- [ ] Build passes (`npm run build`)
- [ ] No broken links
- [ ] Commit messages follow Conventional Commits

## Related Issue
Closes #issue-number (if applicable)
```

### 5. Review Process

- Maintainers will review within 5-7 business days
- Address feedback by pushing new commits to your branch
- Once approved, maintainers will merge your PR

---

## Reporting Issues

### Bug Reports

Use the [Bug Report Template](https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook/issues/new?template=bug_report.md):

- **Title**: Concise description (e.g., "Broken link in Chapter 5")
- **Description**: Steps to reproduce, expected vs actual behavior
- **Environment**: Browser, OS, Node.js version
- **Screenshots**: If applicable

### Feature Requests

Use the [Feature Request Template](https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook/issues/new?template=feature_request.md):

- **Title**: Feature summary (e.g., "Add chapter on MuJoCo integration")
- **Motivation**: Why this feature is valuable
- **Proposed Solution**: How you envision implementing it
- **Alternatives**: Other approaches considered

### Content Questions

- Open a [Discussion](https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook/discussions) for questions
- Use GitHub Discussions for broader topics not suitable for issues

---

## Recognition

Contributors will be recognized in:

- `docs/about.md` (Contributors section)
- GitHub Contributors page
- Release notes for major contributions

---

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file).

---

## Questions?

- **General Questions**: Open a [Discussion](https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook/discussions)
- **Technical Issues**: Open an [Issue](https://github.com/YOUR-USERNAME/Humanoid-physical-ai-textbook/issues)
- **Urgent Matters**: Contact maintainers via email (see `docs/about.md`)

---

**Thank you for contributing to open education in Physical AI and Humanoid Robotics!** 🤖
