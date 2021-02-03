#!/usr/bin/bash

# This script uploads files to the ESP8266 server via the http upload service.
IP=192.168.1.182

for f in data/*
do
  echo "Erasing ${f}"
  curl http://${IP}/erase?path=$(basename ${f})
  echo
  echo "Uploading ${f}"
  curl -F "file=@${f}" ${IP}/
  echo 
done

echo
echo "${IP}/list:"
curl http://${IP}/list
