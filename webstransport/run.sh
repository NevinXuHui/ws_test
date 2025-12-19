#!/bin/bash
cd "$(dirname "$0")"

case "$1" in
    server)
        cd server && go run main.go
        ;;
    client)
        cd client && go run main.go $2
        ;;
    *)
        echo "Usage: $0 {server|client [local|remote]}"
        exit 1
        ;;
esac
