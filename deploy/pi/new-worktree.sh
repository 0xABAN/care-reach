#!/usr/bin/env bash
set -euo pipefail

branch=${1:?Usage: new-worktree.sh teammate/feature}
live="$HOME/worktrees/care-reach/pi-runtime"

git check-ref-format --branch "$branch" >/dev/null
if [[ $branch != */* || $branch == lance/robot-pi || $branch == main ]]; then
  echo 'Use a new namespaced branch such as alex/camera.' >&2
  exit 2
fi

directory="$HOME/worktrees/care-reach/${branch//\//-}"
if [[ -e $directory ]]; then
  echo "Worktree path already exists: $directory" >&2
  exit 1
fi

git -C "$live" fetch origin lance/robot-pi
git -C "$live" worktree add -b "$branch" "$directory" origin/lance/robot-pi
git -C "$directory" branch --unset-upstream
printf 'Work in %s on branch %s\n' "$directory" "$branch"
printf 'To publish that branch: git -C %q push -u origin HEAD\n' "$directory"
