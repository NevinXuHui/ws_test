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
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

var (
	conn      *websocket.Conn
	connected bool
	mu        sync.RWMutex
	stopCh    chan struct{}
)

//export SimpleConnect
func SimpleConnect(deviceId, macId, sn, cmei, deviceType, firmwareVersion, softwareVersion, sdkVersion, innerType, snPwd *C.char) C.int {
	config := map[string]string{
		"deviceId":         C.GoString(deviceId),
		"macId":           C.GoString(macId),
		"sn":              C.GoString(sn),
		"cmei":            C.GoString(cmei),
		"deviceType":      C.GoString(deviceType),
		"firmwareVersion": C.GoString(firmwareVersion),
		"softwareVersion": C.GoString(softwareVersion),
		"sdkVersion":      C.GoString(sdkVersion),
		"innerType":       C.GoString(innerType),
		"snPwd":           C.GoString(snPwd),
	}

	wsURL, err := getURL(config)
	if err != nil {
		return -1
	}

	c, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		return -2
	}

	mu.Lock()
	conn = c
	connected = true
	stopCh = make(chan struct{})
	mu.Unlock()

	// 启动心跳和消息处理
	go startHeartbeat(config)
	go startMessageHandler()

	return 0
}

//export SimpleDisconnect
func SimpleDisconnect() {
	mu.Lock()
	defer mu.Unlock()
	
	if connected {
		connected = false
		close(stopCh)
		if conn != nil {
			conn.Close()
			conn = nil
		}
	}
}

//export IsSimpleConnected
func IsSimpleConnected() C.int {
	mu.RLock()
	defer mu.RUnlock()
	if connected {
		return 1
	}
	return 0
}

func startHeartbeat(config map[string]string) {
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()

	// 立即发送一次心跳
	sendHB(config)

	for {
		select {
		case <-ticker.C:
			mu.RLock()
			if !connected {
				mu.RUnlock()
				return
			}
			mu.RUnlock()
			sendHB(config)
		case <-stopCh:
			return
		}
	}
}

func sendHB(config map[string]string) {
	mu.RLock()
	defer mu.RUnlock()
	
	if !connected || conn == nil {
		return
	}

	hb := map[string]interface{}{
		"deviceId": config["deviceId"],
		"domain":   "KEEP_ALIVE",
		"event":    "keep_alive",
		"eventId":  fmt.Sprintf("hb_%d", time.Now().UnixMilli()),
		"seq":      fmt.Sprintf("%d", time.Now().UnixMilli()),
	}
	
	data, _ := json.Marshal(hb)
	conn.WriteMessage(websocket.TextMessage, data)
}

func startMessageHandler() {
	for {
		mu.RLock()
		if !connected || conn == nil {
			mu.RUnlock()
			return
		}
		c := conn
		mu.RUnlock()

		_, msg, err := c.ReadMessage()
		if err != nil {
			mu.Lock()
			connected = false
			mu.Unlock()
			return
		}

		fmt.Printf("[%s] MSG: %s\n", time.Now().Format("15:04:05.000"), string(msg))
	}
}

func getURL(config map[string]string) (string, error) {
	ts := time.Now().UnixMilli()
	sign := genSign(config["sn"], config["cmei"], strconv.FormatInt(ts, 10), config["snPwd"])

	req := map[string]interface{}{
		"deviceId":         config["deviceId"],
		"macId":           config["macId"],
		"sn":              config["sn"],
		"cmei":            config["cmei"],
		"deviceType":      config["deviceType"],
		"firmwareVersion": config["firmwareVersion"],
		"softwareVersion": config["softwareVersion"],
		"sdkVersion":      config["sdkVersion"],
		"innerType":       config["innerType"],
		"sign":            sign,
		"timestamp":       ts,
		"applicationVersion": map[string]interface{}{"version": "1.0.7"},
		"extraParams":       map[string]interface{}{},
		"capabilityParams":  map[string]interface{}{},
		"otaVersions": []map[string]interface{}{
			{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 301},
		},
		"fromManagedSdk": true,
	}

	data, _ := json.Marshal(req)
	resp, err := http.Post("https://business.homibot.komect.com:9443/robot/business/api/device/client/connect/url", "application/json", bytes.NewBuffer(data))
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
	return "", fmt.Errorf("no url in response")
}

func genSign(sn, cmei, ts, pwd string) string {
	data := fmt.Sprintf("sn=%s&cmei=%s&timestamp=%s&snPwd=%s", sn, cmei, ts, pwd)
	hash := sha256.Sum256([]byte(data))
	return base64.StdEncoding.EncodeToString(hash[:])
}

func main() {} // 必须有main函数
