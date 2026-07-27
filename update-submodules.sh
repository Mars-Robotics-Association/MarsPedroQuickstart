#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Collect submodule paths from .gitmodules
SUBMODULES=()
while IFS= read -r path; do
    SUBMODULES+=("$path")
done < <(git config --file "$SCRIPT_DIR/.gitmodules" --get-regexp 'submodule\..*\.path' | awk '{print $2}')

if [[ ${#SUBMODULES[@]} -eq 0 ]]; then
    echo "No submodules found in .gitmodules."
    exit 1
fi

SUB_PATH=""
if [[ $# -ge 1 ]]; then
    REQUESTED="$1"
    for path in "${SUBMODULES[@]}"; do
        if [[ "$path" == "$REQUESTED" || "$(basename "$path")" == "$REQUESTED" ]]; then
            SUB_PATH="$path"
            break
        fi
    done
    if [[ -z "$SUB_PATH" ]]; then
        echo "ERROR: No submodule matching \"$REQUESTED\" found."
        echo "Available submodules:"
        for path in "${SUBMODULES[@]}"; do
            echo "  - $path"
        done
        exit 1
    fi
else
    echo "Available submodules:"
    echo "-----------------------------------------------"
    for i in "${!SUBMODULES[@]}"; do
        NUM=$((i + 1))
        printf "  %d) %s\n" "$NUM" "${SUBMODULES[$i]}"
    done
    echo ""
    while true; do
        read -rp "Select a submodule to update (1-${#SUBMODULES[@]}): " CHOICE
        if [[ "$CHOICE" =~ ^[0-9]+$ ]] && [[ $CHOICE -ge 1 ]] && [[ $CHOICE -le ${#SUBMODULES[@]} ]]; then
            break
        fi
        echo "Invalid selection. Please enter a number between 1 and ${#SUBMODULES[@]}."
    done
    SUB_PATH="${SUBMODULES[$((CHOICE - 1))]}"
fi

# Only init when the submodule is not checked out. Never run
# `git submodule update` on an already-initialized checkout — that would
# reset a local (possibly uncommitted) pointer/hash change back to the
# commit recorded in the parent index.
SUB_ABS="$SCRIPT_DIR/$SUB_PATH"
if [[ ! -e "$SUB_ABS/.git" ]]; then
    echo ""
    echo "Initializing submodule $SUB_PATH (not yet checked out)..."
    if ! git -C "$SCRIPT_DIR" submodule update --init --recursive -- "$SUB_PATH"; then
        echo "ERROR: Failed to initialize submodule $SUB_PATH."
        exit 1
    fi
fi

if [[ ! -e "$SUB_ABS/.git" ]]; then
    echo "ERROR: $SUB_PATH is still not a git checkout after init."
    exit 1
fi

PARENT_TOPLEVEL="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
SUB_TOPLEVEL="$(git -C "$SUB_ABS" rev-parse --show-toplevel)"
if [[ "$SUB_TOPLEVEL" == "$PARENT_TOPLEVEL" ]]; then
    echo "ERROR: $SUB_PATH is not a checked-out submodule (git resolves to the parent repo)."
    echo "       Refusing to continue so we do not fetch/list tags from the wrong repository."
    exit 1
fi

# Informational only: a divergent checkout is preserved until the user
# explicitly selects a tag below.
RECORDED_SHA="$(git -C "$SCRIPT_DIR" ls-tree HEAD "$SUB_PATH" 2>/dev/null | awk '{print $3}')"
CURRENT_SHA="$(git -C "$SUB_ABS" rev-parse HEAD)"
if [[ -n "$RECORDED_SHA" && "$RECORDED_SHA" != "$CURRENT_SHA" ]]; then
    echo ""
    echo "NOTE: $SUB_PATH is currently at ${CURRENT_SHA:0:12},"
    echo "      which differs from the parent-recorded ${RECORDED_SHA:0:12}."
    echo "      Leaving that checkout alone until you pick a tag."
fi

echo ""
echo "Fetching tags from $SUB_PATH..."
cd "$SUB_ABS"
if ! git fetch --prune --force --quiet origin "+refs/tags/*:refs/tags/*"; then
    echo "ERROR: Failed to fetch tags from remote \"origin\"."
    echo "       Check network access, git auth, and safe.directory settings."
    exit 1
fi

# Detect which tag is currently checked out (empty if HEAD is not on a tag)
CURRENT_TAG="$(git describe --tags --exact-match HEAD 2>/dev/null || true)"

# Collect the 10 most recent tags with their commit dates
TAGS=()
while IFS= read -r tag; do
    TAGS+=("$tag")
done < <(git tag --sort=-creatordate | head -10)
COUNT=${#TAGS[@]}

if [[ $COUNT -eq 0 ]]; then
    echo "No tags found in $SUB_PATH."
    exit 1
fi

echo ""
echo "Available $SUB_PATH tags (newest first):"
echo "-----------------------------------------------"
for i in "${!TAGS[@]}"; do
    TAG="${TAGS[$i]}"
    DATE="$(git log -1 --format="%ci" "$TAG" 2>/dev/null)"
    NUM=$((i + 1))
    if [[ "$TAG" == "$CURRENT_TAG" ]]; then
        printf "  %d) %-40s %s  <-- current\n" "$NUM" "$TAG" "$DATE"
    else
        printf "  %d) %-40s %s\n" "$NUM" "$TAG" "$DATE"
    fi
done
echo ""

while true; do
    read -rp "Enter number (1-$COUNT): " CHOICE
    if [[ "$CHOICE" =~ ^[0-9]+$ ]] && [[ $CHOICE -ge 1 ]] && [[ $CHOICE -le $COUNT ]]; then
        break
    fi
    echo "Invalid selection. Please enter a number between 1 and $COUNT."
done

SELECTED_TAG="${TAGS[$((CHOICE - 1))]}"
SELECTED_SHA="$(git rev-parse "refs/tags/$SELECTED_TAG^{commit}")"

# Abort before checkout if the submodule has local uncommitted work that
# checkout would discard. A pure hash/pointer divergence (clean tree at a
# different commit) is fine — selecting a tag is an intentional move.
if [[ -n "$(git status --porcelain)" ]]; then
    echo ""
    echo "ERROR: $SUB_PATH has uncommitted changes. Refusing to checkout $SELECTED_TAG"
    echo "       so those changes are not clobbered. Commit, stash, or discard them first."
    git status --short
    exit 1
fi

if [[ "$CURRENT_SHA" == "$SELECTED_SHA" ]]; then
    echo ""
    echo "$SUB_PATH is already at $SELECTED_TAG ($SELECTED_SHA)."
    echo "No checkout needed."
    cd "$SCRIPT_DIR"
    exit 0
fi

echo ""
echo "Checking out tag: $SELECTED_TAG"

git checkout "tags/$SELECTED_TAG" --quiet

echo "Updating nested submodules..."
git submodule update --init --recursive
cd "$SCRIPT_DIR"

echo "Staging pointer update in parent repo..."
git add "$SUB_PATH"

echo ""
echo "$SUB_PATH updated to $SELECTED_TAG."
echo "The submodule pointer is staged -- review and commit when ready."
