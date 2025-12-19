#!/bin/bash
cd "$(dirname "$0")"

case "$1" in
    server)
        cd server && go run main.go
        ;;
    client)
        cd client && go run main.go
        ;;
    *)
        echo "Usage: $0 {server|client}"
        exit 1
        ;;
esac
