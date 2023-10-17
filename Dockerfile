FROM alpine:latest

RUN apk update && apk add cmake build-base doxygen

COPY . /app
