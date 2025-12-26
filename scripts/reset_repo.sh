#!/usr/bin/env bash
set -e

BRANCH=$(git branch --show-current)
if [ "$BRANCH" != "main" ]; then
  echo "Refusing to reset: not on main (current: $BRANCH)"
  exit 1
fi

git fetch origin
git reset --hard origin/main
git clean -fd
