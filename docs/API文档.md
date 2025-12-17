# API 接口文档

## 1. 获取WebSocket连接地址

### 接口信息
- **URL**: `https://business.homibot.komect.com:9443/robot/business/api/device/client/connect/url`
- **方法**: POST
- **Content-Type**: application/json

### 请求参数
```json
{
  "deviceId": "设备ID",
  "macId": "MAC地址", 
  "sn": "设备序列号",
  "cmei": "设备CMEI",
  "deviceType": "设备类型",
  "firmwareVersion": "固件版本",
  "softwareVersion": "软件版本",
  "applicationVersion": {},
  "sdkVersion": "SDK版本",
  "sign": "签名",
  "timestamp": 时间戳,
  "innerType": "内部类型",
  "extraParams": {},
  "capabilityParams": {},
  "otaVersions": [
    {
      "currentVersion": "当前版本",
      "versionCode": 版本代码,
      "versionType": 版本类型
    }
  ],
  "fromManagedSdk": true
}
```

### 响应格式
```json
{
  "success": true,
  "code": 0,
  "traceId": "追踪ID",
  "data": {
    "url": "WebSocket连接地址"
  }
}
```

## 2. WebSocket心跳协议

### 设备→平台：心跳信令
```json
{
  "deviceId": "设备ID",
  "domain": "KEEP_ALIVE",
  "event": "keep_alive",
  "eventId": "heartbeat_时间戳",
  "seq": "时间戳"
}
```

### 平台→设备：回复心跳信令
```json
{
  "deviceId": "设备ID", 
  "domain": "KEEP_ALIVE",
  "event": "keep_alive",
  "eventId": "随意",
  "seq": "时间戳"
}
```

## 3. 签名算法

### 签名计算规则
1. 拼接字符串：`sn=xxx&cmei=xxx&timestamp=xxx&snPwd=xxx`
2. 使用SHA-256加密上述字符串
3. 将加密结果用Base64编码

### Go实现示例
```go
func generateSign(sn, cmei, timestamp, snPwd string) string {
    data := fmt.Sprintf("sn=%s&cmei=%s&timestamp=%s&snPwd=%s", sn, cmei, timestamp, snPwd)
    hash := sha256.Sum256([]byte(data))
    return base64.StdEncoding.EncodeToString(hash[:])
}
```
