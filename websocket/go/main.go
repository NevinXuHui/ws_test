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

type OtaVersion struct {
	CurrentVersion string `json:"currentVersion"`
	VersionCode    int    `json:"versionCode"`
	VersionType    int    `json:"versionType"`
}

type RequestData struct {
	DeviceID           string                 `json:"deviceId"`
	MacID             string                 `json:"macId"`
	SN                string                 `json:"sn"`
	CMEI              string                 `json:"cmei"`
	DeviceType        string                 `json:"deviceType"`
	FirmwareVersion   string                 `json:"firmwareVersion"`
	SoftwareVersion   string                 `json:"softwareVersion"`
	ApplicationVersion map[string]interface{} `json:"applicationVersion"`
	SDKVersion        string                 `json:"sdkVersion"`
	Sign              string                 `json:"sign"`
	Timestamp         int64                  `json:"timestamp"`
	InnerType         string                 `json:"innerType"`
	ExtraParams       map[string]interface{} `json:"extraParams"`
	CapabilityParams  map[string]interface{} `json:"capabilityParams"`
	OtaVersions       []OtaVersion           `json:"otaVersions"`
	FromManagedSdk    bool                   `json:"fromManagedSdk"`
}

type SignalResponse struct {
	Success bool   `json:"success"`
	Code    int    `json:"code"`
	TraceId string `json:"traceId"`
	Data    struct {
		URL string `json:"url"`
	} `json:"data"`
}

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
			"branch_name": "release_v1.0.7_unitree200",
			"build_timestamp": "20251208_091640",
			"git_commit_hash": "b9a0d2ca",
			"version": "1.0.7",
		},
		"business": map[string]interface{}{
			"build_timestamp": "20251211092248",
			"git_commit_hash": "0413c5de",
			"version": "1.0.7",
		},
		"third_party": map[string]interface{}{
			"version": "2.0.2.1",
		},
		"version": "1.0.7",
	}

	otaVersions := []OtaVersion{
		{CurrentVersion: "1.0.0", VersionCode: 0, VersionType: 301},
		{CurrentVersion: "1.0.0", VersionCode: 0, VersionType: 302},
		{CurrentVersion: "1.0.0", VersionCode: 0, VersionType: 303},
		{CurrentVersion: "1.0.0", VersionCode: 0, VersionType: 304},
	}

	requestData := RequestData{
		DeviceID:           config.DeviceID,
		MacID:             config.MacID,
		SN:                config.SN,
		CMEI:              config.CMEI,
		DeviceType:        config.DeviceType,
		FirmwareVersion:   config.FirmwareVersion,
		SoftwareVersion:   config.SoftwareVersion,
		ApplicationVersion: applicationVersion,
		SDKVersion:        config.SDKVersion,
		Sign:              sign,
		Timestamp:         timestamp,
		InnerType:         config.InnerType,
		ExtraParams:       make(map[string]interface{}),
		CapabilityParams:  make(map[string]interface{}),
		OtaVersions:       otaVersions,
		FromManagedSdk:    true,
	}

	jsonData, err := json.Marshal(requestData)
	if err != nil {
		return "", err
	}

	apiURL := "https://business.homibot.komect.com:9443/robot/business/api/device/client/connect/url"
	
	fmt.Printf("[%s] Request URL: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), apiURL)
	fmt.Printf("[%s] Request JSON: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), string(jsonData))
	
	resp, err := http.Post(apiURL, "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return "", err
	}

	fmt.Printf("[%s] Response Status: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), resp.Status)
	fmt.Printf("[%s] Response Body: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), string(body))

	var result SignalResponse
	if err := json.Unmarshal(body, &result); err != nil {
		return "", err
	}

	if result.Code != 0 {
		return "", fmt.Errorf("API error: %d", result.Code)
	}

	return result.Data.URL, nil
}

func connectWebSocket(wsURL string, config DeviceConfig) error {
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		return err
	}
	defer conn.Close()

	fmt.Printf("[%s] WebSocket connected successfully\n", time.Now().Format("2006-01-02 15:04:05.000"))

	// 发送心跳函数
	sendHeartbeat := func() {
		heartbeat := map[string]interface{}{
			"deviceId": config.DeviceID,
			"domain":   "KEEP_ALIVE",
			"event":    "keep_alive",
			"eventId":  fmt.Sprintf("heartbeat_%d", time.Now().UnixMilli()),
			"seq":      fmt.Sprintf("%d", time.Now().UnixMilli()),
		}
		heartbeatData, _ := json.Marshal(heartbeat)
		
		if err := conn.WriteMessage(websocket.TextMessage, heartbeatData); err != nil {
			fmt.Printf("[%s] Heartbeat send error: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
			return
		}
		fmt.Printf("[%s] Heartbeat sent: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), heartbeatData)
	}

	// 立即发送一次心跳
	sendHeartbeat()

	// 心跳定时器
	heartbeatTicker := time.NewTicker(10 * time.Second)
	defer heartbeatTicker.Stop()

	// 定时发送心跳
	go func() {
		for range heartbeatTicker.C {
			sendHeartbeat()
		}
	}()

	// 读取消息
	go func() {
		for {
			_, message, err := conn.ReadMessage()
			if err != nil {
				fmt.Printf("[%s] Read error: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
				return
			}
			fmt.Printf("[%s] Received: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), message)
		}
	}()

	// 保持连接
	select {}
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

	wsURL, err := getSignalURL(config)
	if err != nil {
		fmt.Printf("[%s] Failed to get signal URL: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
		return
	}

	fmt.Printf("[%s] Got WebSocket URL: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), wsURL)

	if err := connectWebSocket(wsURL, config); err != nil {
		fmt.Printf("[%s] WebSocket connection failed: %v\n", time.Now().Format("2006-01-02 15:04:05.000"), err)
	}
}
