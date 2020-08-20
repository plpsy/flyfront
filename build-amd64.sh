#!/bin/bash

export GO111MODULE="off"
export GOOS="linux"
export GOARCH="amd64"
export CGO_ENABLED="0"

go build



