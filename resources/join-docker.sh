#!/bin/bash

# Get the name of the first running container
CONTAINER_NAME=$(docker ps --format "{{.Names}}" | head -1)

if [ -z "$CONTAINER_NAME" ]; then
    echo "No running containers found."
    exit 1
fi

# Execute an interactive Bash shell inside the found container
echo "Joining container $CONTAINER_NAME"
docker exec -it "$CONTAINER_NAME" bash
