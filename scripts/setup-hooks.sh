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

  # ---- Official release binary (no root), x86_64 Linux, WITH checksum verify ----
  VER=8.30.1
  TMP="$(mktemp -d)"                       # private temp dir (no fixed /tmp path)
  BASE="https://github.com/gitleaks/gitleaks/releases/download/v${VER}"
  curl -sSL -o "$TMP/gl.tgz"        "$BASE/gitleaks_${VER}_linux_x64.tar.gz"
  curl -sSL -o "$TMP/checksums.txt" "$BASE/gitleaks_${VER}_checksums.txt"
  ( cd "$TMP" \
    && mv gl.tgz "gitleaks_${VER}_linux_x64.tar.gz" \
    && sha256sum -c --ignore-missing checksums.txt )   # ABORTS on mismatch
  mkdir -p "$HOME/.local/bin"
  tar -xzf "$TMP/gitleaks_${VER}_linux_x64.tar.gz" -C "$HOME/.local/bin" gitleaks
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
