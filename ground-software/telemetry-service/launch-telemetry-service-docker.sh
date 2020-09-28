#!/bin/sh

# Check to see if a data directory is provided. If not, use the current directory.
if [ "$1" != "" ]; then
    DATA_FILE_DIR=$1
else
    DATA_FILE_DIR=$PWD
fi
echo "Saving telemetry data files to: ${DATA_FILE_DIR}"

docker build -t telemetry-service:latest .

docker run -d -it \
    --name telemetry-service \
    --restart unless-stopped \
    --mount type=bind,src=$DATA_FILE_DIR,dst=/run/telemetry \
    -p 8000:8000 \
    telemetry-service:latest
