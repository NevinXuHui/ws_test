package main

/*
#include <stdlib.h>
*/
import "C"
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
	"unsafe"

	"github.com/gorilla/websocket"
)

// 全局变量存储连接
var globalConn *websocket.Conn
var isConnected bool = false

// 回调函数类型
type MessageCallback func(*C.char)

var messageCallback MessageCallback

//export SetMessageCallback
func SetMessageCallback(callback uintptr) {
	if callback != 0 {
		messageCallback = *(*MessageCallback)(unsafe.Pointer(&callback))
	} else {
		messageCallback = nil
	}
}

//export ConnectWebSocket
func ConnectWebSocket(deviceId, macId, sn, cmei, deviceType, firmwareVersion, softwareVersion, sdkVersion, innerType, snPwd *C.char) C.int {
	config := DeviceConfig{
		DeviceID:         C.GoString(deviceId),
		MacID:           C.GoString(macId),
		SN:              C.GoString(sn),
		CMEI:            C.GoString(cmei),
		DeviceType:      C.GoString(deviceType),
		FirmwareVersion: C.GoString(firmwareVersion),
		SoftwareVersion: C.GoString(softwareVersion),
		SDKVersion:      C.GoString(sdkVersion),
		InnerType:       C.GoString(innerType),
		SNPwd:           C.GoString(snPwd),
	}

	wsURL, err := getSignalURL(config)
	if err != nil {
		return -1
	}

	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		return -2
	}

	globalConn = conn
	isConnected = true

	// 立即发送心跳
	sendHeartbeat(config)

	// 启动心跳和消息处理
	go heartbeatLoop(config)
	go messageLoop()

	return 0
}

//export SendMessage
func SendMessage(message *C.char) C.int {
	if !isConnected || globalConn == nil {
		return -1
	}

	msg := C.GoString(message)
	err := globalConn.WriteMessage(websocket.TextMessage, []byte(msg))
	if err != nil {
		return -2
	}
	return 0
}

//export Disconnect
func Disconnect() {
	if globalConn != nil {
		globalConn.Close()
		globalConn = nil
		isConnected = false
	}
}

//export IsConnected
func IsConnected() C.int {
	if isConnected {
		return 1
	}
	return 0
}

func sendHeartbeat(config DeviceConfig) {
	if !isConnected || globalConn == nil {
		return
	}

	heartbeat := map[string]interface{}{
		"deviceId": config.DeviceID,
		"domain":   "KEEP_ALIVE",
		"event":    "keep_alive",
		"eventId":  fmt.Sprintf("heartbeat_%d", time.Now().UnixMilli()),
		"seq":      fmt.Sprintf("%d", time.Now().UnixMilli()),
	}
	heartbeatData, _ := json.Marshal(heartbeat)
	globalConn.WriteMessage(websocket.TextMessage, heartbeatData)
}

func heartbeatLoop(config DeviceConfig) {
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("heartbeatLoop recovered from panic: %v\n", r)
		}
	}()
	
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			if !isConnected || globalConn == nil {
				return
			}
			sendHeartbeat(config)
		}
	}
}

func messageLoop() {
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("messageLoop recovered from panic: %v\n", r)
		}
	}()
	
	for {
		if !isConnected || globalConn == nil {
			break
		}

		_, message, err := globalConn.ReadMessage()
		if err != nil {
			isConnected = false
			break
		}

		// 安全地打印消息
		if len(message) > 0 {
			fmt.Printf("[%s] Received: %s\n", time.Now().Format("2006-01-02 15:04:05.000"), string(message))
		}
	}
}

// 内部函数
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
			"version": "1.0.7",
		},
		"business": map[string]interface{}{
			"version": "1.0.7",
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
	resp, err := http.Post(apiURL, "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return "", err
	}

	var result SignalResponse
	if err := json.Unmarshal(body, &result); err != nil {
		return "", err
	}

	if result.Code != 0 {
		return "", fmt.Errorf("API error: %d", result.Code)
	}

	return result.Data.URL, nil
}

func main() {} // 必须有main函数才能编译为C库
