#!/bin/sh
# Alternative to "Sketchbook location = this repo": symlink the bundled
# libraries into your normal Arduino sketchbook libraries folder.
# Usage: ./link-libraries.sh [sketchbook-libraries-dir]
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
if [ -n "$1" ]; then DEST=$1
elif [ "$(uname)" = Darwin ]; then DEST="$HOME/Documents/Arduino/libraries"
else DEST="$HOME/Arduino/libraries"; fi
mkdir -p "$DEST"
for lib in "$REPO"/libraries/*/; do
  name=$(basename "$lib")
  target="$DEST/$name"
  if [ -L "$target" ]; then
    ln -sfn "${lib%/}" "$target"; echo "updated  $target"
  elif [ -e "$target" ]; then
    echo "SKIPPED  $target exists (not a symlink) - remove/rename it to use the repo version"
  else
    ln -s "${lib%/}" "$target"; echo "linked   $target"
  fi
done
