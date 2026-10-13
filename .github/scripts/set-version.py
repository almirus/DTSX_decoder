"""Set the compiled version from a validated GitHub release tag."""

import os
from pathlib import Path
import re


def version_from_tag(tag):
    number = r"(?:0|[1-9][0-9]*)"
    prerelease_id = r"(?:0|[1-9][0-9]*|[0-9]*[A-Za-z-][0-9A-Za-z-]*)"
    pattern = (
        rf"v({number}\.{number}\.{number}"
        rf"(?:-{prerelease_id}(?:\.{prerelease_id})*)?"
        rf"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?)"
    )
    match = re.fullmatch(pattern, tag)
    if not match:
        raise ValueError(f"Expected a SemVer tag such as v1.2.3 or v1.2.3-beta.1: {tag!r}")
    return match.group(1)


def main():
    if os.environ.get("GITHUB_REF_TYPE") != "tag":
        print("Using the version stored in app_version.hpp")
        return

    version = version_from_tag(os.environ["GITHUB_REF_NAME"])
    header = Path(__file__).resolve().parents[2] / "src/app/app_version.hpp"
    content, count = re.subn(
        r'inline constexpr const char\* kVersion = "[^"\n]*";',
        f'inline constexpr const char* kVersion = "{version}";',
        header.read_text(encoding="utf-8"),
    )
    if count != 1:
        raise ValueError("Expected exactly one kVersion declaration in app_version.hpp")
    header.write_text(content, encoding="utf-8", newline="\n")
    print(f"Compiled version: {version}")


if __name__ == "__main__":
    main()
