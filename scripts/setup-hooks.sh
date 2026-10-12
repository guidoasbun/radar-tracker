#!/usr/bin/env bash
#
# One-time setup: activate the version-controlled git hooks and check for the
# gitleaks binary. Run from anywhere in the repo:  ./scripts/setup-hooks.sh
#
# This points git at .githooks/ (tracked) instead of .git/hooks/ (untracked), so
# the whole team shares the same secret-scanning pre-commit hook. See ADR-0005.

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

echo "Activating tracked hooks: core.hooksPath -> .githooks"
git config core.hooksPath .githooks
chmod +x .githooks/* 2>/dev/null || true

if command -v gitleaks >/dev/null 2>&1 || [ -x "${HOME}/.local/bin/gitleaks" ]; then
  echo "gitleaks found: $("${HOME}/.local/bin/gitleaks" version 2>/dev/null || gitleaks version)"
else
  cat >&2 <<'EOF'

gitleaks is NOT installed. Install it securely with ONE of these:

  # ---- Official release binary (no root), x86_64 Linux ----
  # The SHA256 is PINNED here (copied once from gitleaks_8.30.1_checksums.txt).
  # Everything is &&-chained so extraction only runs if the checksum matches,
  # and curl -f makes an HTTP error page a failure instead of a saved "file".
  VER=8.30.1
  SHA256=551f6fc83ea457d62a0d98237cbad105af8d557003051f41f3e7ca7b3f2470eb
  TMP="$(mktemp -d)" && \
  curl -fsSL -o "$TMP/gl.tgz" \
    "https://github.com/gitleaks/gitleaks/releases/download/v${VER}/gitleaks_${VER}_linux_x64.tar.gz" && \
  echo "${SHA256}  $TMP/gl.tgz" | sha256sum -c - && \
  mkdir -p "$HOME/.local/bin" && \
  tar -xzf "$TMP/gl.tgz" -C "$HOME/.local/bin" gitleaks; \
  rm -rf "$TMP"

  # ---- macOS ----
  brew install gitleaks

  # ---- Go toolchain (version PINNED, never @latest) ----
  go install github.com/gitleaks/gitleaks/v8@v8.30.1

Ensure the install dir is on your PATH, then re-run this script.
EOF
  exit 1
fi

echo "Done. The pre-commit hook will scan staged changes for secrets."
echo "Bypass a single commit (use sparingly) with: git commit --no-verify"
