#!/bin/sh
cd "$(dirname "$0")/web" && echo "Open http://localhost:8090 in Chrome" && python3 -m http.server 8090
