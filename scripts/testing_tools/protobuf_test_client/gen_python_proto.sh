#!/bin/bash

SRC=${GRASPIT}/protocols
DST=${GRASPIT}/scripts/testing_tools/protobuf_test_client

protoc --proto_path=${SRC} --python_out=${DST} ${SRC}/GraspitMessage.proto 
protoc --proto_path=${SRC} --python_out=${DST} ${SRC}/Drawable.proto 
