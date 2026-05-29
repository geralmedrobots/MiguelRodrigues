#!/bin/bash

TERM_CMD="bash -c"
# === Configuration ===
CONTAINER_NAME=$1           # First argument: name or ID of the container
NUM_SESSIONS=${2:-3}        # Second argument: number of terminal sessions (default: 3)

# === Check prerequisites ===
if ! docker ps | grep -q "$CONTAINER_NAME"; then
  echo "❌ Container '$CONTAINER_NAME' is not running or doesn't exist."
  exit 1
fi

# === Open terminal sessions ===
echo "🖥️  Opening terminal session $i into container '$CONTAINER_NAME'..."
$TERM_CMD "docker exec -it $CONTAINER_NAME bash" 

