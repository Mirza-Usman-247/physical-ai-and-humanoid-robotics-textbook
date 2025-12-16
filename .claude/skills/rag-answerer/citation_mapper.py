"""
Citation mapping and validation utilities.
Maps inline [N] citations to source references with validation.
"""

import re
import logging
from typing import List, Dict, Any, Tuple, Optional

logger = logging.getLogger(__name__)


class CitationMapper:
    """
    Map and validate inline citations in generated answers.

    Handles:
    - Parsing [N] citations from text
    - Mapping citation numbers to source references
    - Validating citation correctness
    - Formatting citation links
    """

    def __init__(self):
        """Initialize citation mapper."""
        self.citation_pattern = re.compile(r'\[(\d+)\]')

    def parse_citations(self, text: str) -> List[int]:
        """
        Extract all citation numbers from text.

        Args:
            text: Text containing [N] citations

        Returns:
            List[int]: List of citation numbers found

        Example:
            >>> mapper = CitationMapper()
            >>> mapper.parse_citations("Forward kinematics [1] is used with DH parameters [2].")
            [1, 2]
        """
        matches = self.citation_pattern.findall(text)
        citations = [int(n) for n in matches]
        return citations

    def validate_citations(
        self,
        text: str,
        max_sources: int
    ) -> Tuple[bool, List[str]]:
        """
        Validate that all citations in text are valid.

        Args:
            text: Text containing citations
            max_sources: Maximum valid citation number

        Returns:
            Tuple[bool, List[str]]: (is_valid, list_of_errors)

        Example:
            >>> mapper.validate_citations("Text with [1] and [2]", max_sources=2)
            (True, [])
            >>> mapper.validate_citations("Text with [5]", max_sources=2)
            (False, ["Citation [5] exceeds available sources (max: 2)"])
        """
        citations = self.parse_citations(text)
        errors = []

        # Check for invalid citation numbers
        for citation_num in citations:
            if citation_num < 1:
                errors.append(f"Invalid citation number: [{citation_num}] (must be >= 1)")
            elif citation_num > max_sources:
                errors.append(
                    f"Citation [{citation_num}] exceeds available sources (max: {max_sources})"
                )

        # Check for missing citations (if sources provided but none cited)
        if max_sources > 0 and not citations:
            logger.warning("No citations found in answer text despite having sources")

        is_valid = len(errors) == 0
        return is_valid, errors

    def map_citations_to_sources(
        self,
        text: str,
        sources: List[Dict[str, Any]]
    ) -> Dict[int, Dict[str, Any]]:
        """
        Map each citation number to its corresponding source.

        Args:
            text: Text containing citations
            sources: List of source dicts (with id, file_path, section, etc.)

        Returns:
            Dict[int, Dict[str, Any]]: Mapping of citation number to source info

        Example:
            >>> sources = [
            ...     {"id": 1, "file_path": "docs/ch1.md", "section": "Intro"},
            ...     {"id": 2, "file_path": "docs/ch2.md", "section": "Methods"}
            ... ]
            >>> mapper.map_citations_to_sources("Text [1] and [2]", sources)
            {1: {"id": 1, "file_path": "docs/ch1.md", ...}, 2: {...}}
        """
        citations = self.parse_citations(text)
        citation_map = {}

        # Build source lookup by id
        source_lookup = {src["id"]: src for src in sources}

        for citation_num in set(citations):  # Unique citations only
            if citation_num in source_lookup:
                citation_map[citation_num] = source_lookup[citation_num]
            else:
                logger.warning(f"Citation [{citation_num}] not found in sources")

        return citation_map

    def format_citation_links(
        self,
        text: str,
        sources: List[Dict[str, Any]],
        link_format: str = "markdown"
    ) -> str:
        """
        Convert [N] citations to clickable links.

        Args:
            text: Text containing [N] citations
            sources: List of source dicts
            link_format: "markdown" | "html" | "plain"

        Returns:
            str: Text with citations converted to links

        Example (markdown):
            Input: "Forward kinematics [1] is important."
            Output: "Forward kinematics [[1]](docs/ch1.md:45-67) is important."

        Example (html):
            Input: "Forward kinematics [1] is important."
            Output: "Forward kinematics <a href='#source-1'>[1]</a> is important."
        """
        citation_map = self.map_citations_to_sources(text, sources)

        def replace_citation(match):
            citation_num = int(match.group(1))
            if citation_num not in citation_map:
                return match.group(0)  # Keep original if not found

            source = citation_map[citation_num]

            if link_format == "markdown":
                file_path = source.get("file_path", "")
                line_range = source.get("line_range", "")
                link = f"[{citation_num}]({file_path}:{line_range})"
                return f"[{link}]"

            elif link_format == "html":
                source_id = source.get("id", citation_num)
                return f'<a href="#source-{source_id}" class="citation-link">[{citation_num}]</a>'

            else:  # plain
                return match.group(0)

        result = self.citation_pattern.sub(replace_citation, text)
        return result

    def extract_cited_sources(
        self,
        text: str,
        sources: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Extract only the sources that were actually cited in the text.

        Args:
            text: Text containing citations
            sources: Full list of sources

        Returns:
            List[Dict[str, Any]]: List of cited sources only

        Example:
            >>> sources = [{"id": 1, ...}, {"id": 2, ...}, {"id": 3, ...}]
            >>> text = "Information from [1] and [3]."
            >>> mapper.extract_cited_sources(text, sources)
            [{"id": 1, ...}, {"id": 3, ...}]  # Source 2 not cited
        """
        citations = set(self.parse_citations(text))
        cited_sources = [src for src in sources if src.get("id") in citations]
        cited_sources.sort(key=lambda x: x.get("id", 0))
        return cited_sources

    def get_citation_statistics(
        self,
        text: str,
        sources: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Get statistics about citations in the text.

        Args:
            text: Text containing citations
            sources: List of sources

        Returns:
            Dict[str, Any]: Citation statistics

        Example:
            >>> stats = mapper.get_citation_statistics(text, sources)
            >>> print(stats)
            {
                "total_citations": 5,
                "unique_citations": 3,
                "sources_available": 5,
                "sources_cited": 3,
                "citation_coverage": 0.6,
                "uncited_sources": [4, 5]
            }
        """
        all_citations = self.parse_citations(text)
        unique_citations = set(all_citations)

        available_source_ids = {src.get("id") for src in sources}
        cited_source_ids = {c for c in unique_citations if c in available_source_ids}
        uncited_source_ids = available_source_ids - cited_source_ids

        citation_coverage = len(cited_source_ids) / len(available_source_ids) if available_source_ids else 0

        return {
            "total_citations": len(all_citations),
            "unique_citations": len(unique_citations),
            "sources_available": len(available_source_ids),
            "sources_cited": len(cited_source_ids),
            "citation_coverage": round(citation_coverage, 2),
            "uncited_sources": sorted(uncited_source_ids)
        }

    def highlight_citations(
        self,
        text: str,
        sources: List[Dict[str, Any]],
        highlight_style: str = "bold"
    ) -> str:
        """
        Highlight citations in text for better visibility.

        Args:
            text: Text containing citations
            sources: List of sources
            highlight_style: "bold" | "color" | "background"

        Returns:
            str: Text with highlighted citations

        Example (bold):
            Input: "Text [1] and [2]"
            Output: "Text **[1]** and **[2]**"
        """
        citation_map = self.map_citations_to_sources(text, sources)

        def replace_citation(match):
            citation_num = int(match.group(1))
            citation_text = match.group(0)

            if citation_num not in citation_map:
                return citation_text

            if highlight_style == "bold":
                return f"**{citation_text}**"
            elif highlight_style == "color":
                return f'<span style="color: #0066cc;">{citation_text}</span>'
            elif highlight_style == "background":
                return f'<mark>{citation_text}</mark>'
            else:
                return citation_text

        result = self.citation_pattern.sub(replace_citation, text)
        return result

    def renumber_citations(
        self,
        text: str,
        old_to_new: Dict[int, int]
    ) -> str:
        """
        Renumber citations according to a mapping.

        Useful when sources are reordered or filtered.

        Args:
            text: Text containing citations
            old_to_new: Mapping of old citation numbers to new ones

        Returns:
            str: Text with renumbered citations

        Example:
            >>> text = "Info from [1] and [3]"
            >>> mapper.renumber_citations(text, {1: 1, 3: 2})
            "Info from [1] and [2]"
        """
        def replace_citation(match):
            old_num = int(match.group(1))
            new_num = old_to_new.get(old_num, old_num)
            return f"[{new_num}]"

        result = self.citation_pattern.sub(replace_citation, text)
        return result

    def build_source_list_markdown(
        self,
        sources: List[Dict[str, Any]],
        cited_only: bool = False,
        text: Optional[str] = None
    ) -> str:
        """
        Build a formatted markdown source list.

        Args:
            sources: List of source dicts
            cited_only: If True, only include sources that appear in text
            text: Text to check for citations (required if cited_only=True)

        Returns:
            str: Formatted markdown source list

        Example:
            >>> markdown = mapper.build_source_list_markdown(sources)
            >>> print(markdown)
            ## Sources

            [1] **Introduction to ROS2 > Core Concepts**
                `docs/module-1/chapter-4.md:45-67`

            [2] **Kinematics > Forward Kinematics**
                `docs/module-1/chapter-3.md:78-95`
        """
        if cited_only and text:
            sources = self.extract_cited_sources(text, sources)

        if not sources:
            return "## Sources\n\nNo sources cited."

        lines = ["## Sources\n"]
        for src in sources:
            source_id = src.get("id", "?")
            section = src.get("section", "Unknown section")
            file_path = src.get("file_path", "unknown")
            line_range = src.get("line_range", "")

            lines.append(f"[{source_id}] **{section}**")
            lines.append(f"    `{file_path}:{line_range}`")
            lines.append("")

        return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Citation mapper utility")
    parser.add_argument("--text", required=True, help="Text containing citations")
    parser.add_argument("--sources", required=True, help="JSON file with sources")
    parser.add_argument("--validate", action="store_true", help="Validate citations")
    parser.add_argument("--format", choices=["markdown", "html", "plain"], default="markdown", help="Link format")
    parser.add_argument("--stats", action="store_true", help="Show citation statistics")

    args = parser.parse_args()

    # Load sources
    with open(args.sources, 'r') as f:
        sources = json.load(f)

    mapper = CitationMapper()

    if args.validate:
        is_valid, errors = mapper.validate_citations(args.text, len(sources))
        print(json.dumps({
            "valid": is_valid,
            "errors": errors
        }, indent=2))

    elif args.stats:
        stats = mapper.get_citation_statistics(args.text, sources)
        print(json.dumps(stats, indent=2))

    else:
        # Format citations
        formatted = mapper.format_citation_links(args.text, sources, args.format)
        print(formatted)
