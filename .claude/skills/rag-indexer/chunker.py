"""
Semantic markdown chunker for RAG indexing.
Chunks content by H2/H3 headings with configurable size and overlap.
"""

import re
from typing import List, Dict, Any
import tiktoken
import logging

logger = logging.getLogger(__name__)


class MarkdownChunker:
    """
    Semantic markdown chunker that splits by heading structure.

    Preserves document structure by splitting at H2/H3 headings
    while respecting token limits.
    """

    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 128):
        """
        Initialize markdown chunker.

        Args:
            chunk_size: Target chunk size in tokens (default: 512)
            chunk_overlap: Number of overlapping tokens between chunks (default: 128)
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def chunk_markdown(
        self,
        content: str,
        file_path: str = ""
    ) -> List[Dict[str, Any]]:
        """
        Chunk markdown content by heading structure.

        Args:
            content: Markdown content to chunk
            file_path: Source file path (for metadata)

        Returns:
            List of chunk dictionaries with content and metadata
        """
        # Parse markdown structure
        sections = self._parse_markdown_structure(content)

        if not sections:
            logger.warning(f"No sections found in {file_path}, treating as single chunk")
            return self._chunk_text(content, file_path)

        # Generate chunks from sections
        chunks = []
        for section in sections:
            section_chunks = self._chunk_section(section, file_path)
            chunks.extend(section_chunks)

        logger.info(f"Generated {len(chunks)} chunks from {len(sections)} sections")
        return chunks

    def _parse_markdown_structure(self, content: str) -> List[Dict[str, Any]]:
        """
        Parse markdown into hierarchical sections based on headings.

        Args:
            content: Markdown content

        Returns:
            List of section dictionaries with heading hierarchy
        """
        lines = content.split("\n")
        sections = []
        current_section = None
        current_h1 = ""
        current_h2 = ""
        current_h3 = ""
        line_number = 1

        for line in lines:
            # Match markdown headings
            h1_match = re.match(r"^#\s+(.+)$", line)
            h2_match = re.match(r"^##\s+(.+)$", line)
            h3_match = re.match(r"^###\s+(.+)$", line)

            if h1_match:
                # Save previous section
                if current_section:
                    sections.append(current_section)

                current_h1 = h1_match.group(1).strip()
                current_section = {
                    "h1": current_h1,
                    "h2": "",
                    "h3": "",
                    "content": "",
                    "line_start": line_number
                }

            elif h2_match:
                # Save previous section
                if current_section:
                    sections.append(current_section)

                current_h2 = h2_match.group(1).strip()
                current_section = {
                    "h1": current_h1,
                    "h2": current_h2,
                    "h3": "",
                    "content": "",
                    "line_start": line_number
                }

            elif h3_match:
                # Save previous section
                if current_section:
                    sections.append(current_section)

                current_h3 = h3_match.group(1).strip()
                current_section = {
                    "h1": current_h1,
                    "h2": current_h2,
                    "h3": current_h3,
                    "content": "",
                    "line_start": line_number
                }

            else:
                # Add content to current section
                if current_section is not None:
                    current_section["content"] += line + "\n"

            line_number += 1

        # Add final section
        if current_section:
            sections.append(current_section)

        # Set line_end for each section
        for i, section in enumerate(sections):
            if i < len(sections) - 1:
                section["line_end"] = sections[i + 1]["line_start"] - 1
            else:
                section["line_end"] = line_number

        return sections

    def _chunk_section(
        self,
        section: Dict[str, Any],
        file_path: str
    ) -> List[Dict[str, Any]]:
        """
        Chunk a section into smaller pieces if it exceeds chunk_size.

        Args:
            section: Section dictionary with content and metadata
            file_path: Source file path

        Returns:
            List of chunk dictionaries
        """
        content = section["content"].strip()
        if not content:
            return []

        # Count tokens
        tokens = self.encoding.encode(content)
        token_count = len(tokens)

        # If section fits in one chunk, return as-is
        if token_count <= self.chunk_size:
            return [{
                "content": content,
                "h1": section["h1"],
                "h2": section["h2"],
                "h3": section["h3"],
                "line_start": section["line_start"],
                "line_end": section["line_end"],
                "token_count": token_count,
                "tags": self._extract_tags(content)
            }]

        # Split into smaller chunks with overlap
        chunks = []
        start_idx = 0

        while start_idx < len(tokens):
            # Extract chunk tokens
            end_idx = min(start_idx + self.chunk_size, len(tokens))
            chunk_tokens = tokens[start_idx:end_idx]

            # Decode back to text
            chunk_text = self.encoding.decode(chunk_tokens)

            # Calculate approximate line numbers
            chunk_line_start = section["line_start"] + int(
                (start_idx / len(tokens)) * (section["line_end"] - section["line_start"])
            )
            chunk_line_end = section["line_start"] + int(
                (end_idx / len(tokens)) * (section["line_end"] - section["line_start"])
            )

            chunks.append({
                "content": chunk_text,
                "h1": section["h1"],
                "h2": section["h2"],
                "h3": section["h3"],
                "line_start": chunk_line_start,
                "line_end": chunk_line_end,
                "token_count": len(chunk_tokens),
                "tags": self._extract_tags(chunk_text)
            })

            # Move start position with overlap
            start_idx += self.chunk_size - self.chunk_overlap

        logger.info(f"Split section into {len(chunks)} chunks (original: {token_count} tokens)")
        return chunks

    def _chunk_text(
        self,
        content: str,
        file_path: str
    ) -> List[Dict[str, Any]]:
        """
        Chunk plain text without heading structure.

        Args:
            content: Text content
            file_path: Source file path

        Returns:
            List of chunk dictionaries
        """
        tokens = self.encoding.encode(content)
        chunks = []
        start_idx = 0
        line_number = 1

        while start_idx < len(tokens):
            end_idx = min(start_idx + self.chunk_size, len(tokens))
            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = self.encoding.decode(chunk_tokens)

            chunks.append({
                "content": chunk_text,
                "h1": "",
                "h2": "",
                "h3": "",
                "line_start": line_number,
                "line_end": line_number + chunk_text.count("\n"),
                "token_count": len(chunk_tokens),
                "tags": self._extract_tags(chunk_text)
            })

            line_number += chunk_text.count("\n")
            start_idx += self.chunk_size - self.chunk_overlap

        return chunks

    def _extract_tags(self, content: str) -> List[str]:
        """
        Extract relevant tags from content for better retrieval.

        Args:
            content: Text content

        Returns:
            List of extracted tags
        """
        tags = []

        # Extract common robotics/AI terms
        keywords = [
            "kinematics", "dynamics", "control", "ros", "robot", "sensor",
            "actuator", "dh", "jacobian", "trajectory", "planning", "vision",
            "deep learning", "reinforcement learning", "neural network",
            "simulation", "isaac", "gazebo", "mujoco"
        ]

        content_lower = content.lower()
        for keyword in keywords:
            if keyword in content_lower:
                tags.append(keyword)

        return tags[:5]  # Limit to 5 tags
