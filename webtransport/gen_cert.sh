#!/bin/bash
# 生成自签名证书用于测试
openssl req -x509 -newkey rsa:2048 -keyout server/key.pem -out server/cert.pem \
    -days 365 -nodes -subj "/CN=localhost"
echo "证书已生成: server/cert.pem, server/key.pem"
