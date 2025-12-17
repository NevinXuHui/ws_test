package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"strconv"
	"time"

	"github.com/gorilla/websocket"
)

type DeviceConfig struct {
	DeviceID         string
	MacID           string
	SN              string
	CMEI            string
	DeviceType      string
	FirmwareVersion string
	SoftwareVersion string
	SDKVersion      string
	InnerType       string
	SNPwd           string
}

func main() {
	config := DeviceConfig{
		DeviceID:         "1222004229866666660001505",
		MacID:           "FC23CD911857", 
		SN:              "1222004229866666660001505",
		CMEI:            "866666660001505",
		DeviceType:      "591884",
		FirmwareVersion: "1.0.1",
		SoftwareVersion: "1.0.7",
		SDKVersion:      "1.0.7",
		InnerType:       "1",
		SNPwd:           "14mrrGJH",
	}

	// 自动重连循环
	for {
		fmt.Printf("[%s] 尝试连接...\n", time.Now().Format("2006-01-02 15:04:05.000"))
		
		err := connectWithRetry(config)
		if err != nil {
			fmt.Printf("[%s] 连接失败: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
		}
		
		fmt.Printf("[%s] 连接断开，5秒后重连...\n", time.Now().Format("2006-01-02 15:04:05.000"))
		time.Sleep(5 * time.Second)
	}
}

func connectWithRetry(config DeviceConfig) error {
	wsURL, err := getSignalURL(config)
	if err != nil {
		return err
	}

	fmt.Printf("[%s] 获取到WebSocket URL\n", time.Now().Format("2006-01-02 15:04:05.000"))

	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		return err
	}
	defer conn.Close()

	fmt.Printf("[%s] WebSocket连接成功\n", time.Now().Format("2006-01-02 15:04:05.000"))

	// 立即发送心跳
	sendHeartbeat(conn, config)

	// 心跳定时器
	heartbeatTicker := time.NewTicker(10 * time.Second)
	defer heartbeatTicker.Stop()

	// 发送心跳
	go func() {
		for range heartbeatTicker.C {
			if err := sendHeartbeat(conn, config); err != nil {
				fmt.Printf("[%s] 心跳发送失败: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
				return
			}
		}
	}()

	// 读取消息
	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			fmt.Printf("[%s] 连接断开: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
			return err
		}
		fmt.Printf("[%s] 收到消息: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), message)
	}
}

func sendHeartbeat(conn *websocket.Conn, config DeviceConfig) error {
	heartbeat := map[string]interface{}{
		"deviceId": config.DeviceID,
		"domain":   "KEEP_ALIVE",
		"event":    "keep_alive",
		"eventId":  fmt.Sprintf("heartbeat_%d", time.Now().UnixMilli()),
		"seq":      fmt.Sprintf("%d", time.Now().UnixMilli()),
	}
	heartbeatData, _ := json.Marshal(heartbeat)
	
	err := conn.WriteMessage(websocket.TextMessage, heartbeatData)
	if err == nil {
		fmt.Printf("[%s] 心跳已发送\n", time.Now().Format("2006-01-02 15:04:05.000"))
	}
	return err
}

// 其他函数保持不变...
func generateSign(sn, cmei, timestamp, snPwd string) string {
	data := fmt.Sprintf("sn=%s&cmei=%s&timestamp=%s&snPwd=%s", sn, cmei, timestamp, snPwd)
	hash := sha256.Sum256([]byte(data))
	return base64.StdEncoding.EncodeToString(hash[:])
}

func getSignalURL(config DeviceConfig) (string, error) {
	timestamp := time.Now().UnixMilli()
	sign := generateSign(config.SN, config.CMEI, strconv.FormatInt(timestamp, 10), config.SNPwd)

	applicationVersion := map[string]interface{}{
		"algorithm": map[string]interface{}{
			"version": "1.0.7",
		},
		"business": map[string]interface{}{
			"version": "1.0.7",
		},
		"version": "1.0.7",
	}

	requestData := map[string]interface{}{
		"deviceId":           config.DeviceID,
		"macId":             config.MacID,
		"sn":                config.SN,
		"cmei":              config.CMEI,
		"deviceType":        config.DeviceType,
		"firmwareVersion":   config.FirmwareVersion,
		"softwareVersion":   config.SoftwareVersion,
		"applicationVersion": applicationVersion,
		"sdkVersion":        config.SDKVersion,
		"sign":              sign,
		"timestamp":         timestamp,
		"innerType":         config.InnerType,
		"extraParams":       make(map[string]interface{}),
		"capabilityParams":  make(map[string]interface{}),
		"otaVersions": []map[string]interface{}{
			{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 301},
			{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 302},
			{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 303},
			{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 304},
		},
		"fromManagedSdk": true,
	}

	jsonData, _ := json.Marshal(requestData)
	apiURL := "https://business.homibot.komect.com:9443/robot/business/api/device/client/connect/url"
	
	resp, err := http.Post(apiURL, "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()

	body, _ := io.ReadAll(resp.Body)
	var result map[string]interface{}
	json.Unmarshal(body, &result)

	if data, ok := result["data"].(map[string]interface{}); ok {
		if url, ok := data["url"].(string); ok {
			return url, nil
		}
	}
	return "", fmt.Errorf("获取URL失败")
}
