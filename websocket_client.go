package main

/*
#include <stdlib.h>

typedef void (*MessageCallback)(const char* message);
typedef void (*DisconnectCallback)();

static MessageCallback _msgCb = NULL;
static DisconnectCallback _disconnCb = NULL;

static void setMsgCb(MessageCallback cb) { _msgCb = cb; }
static void setDisconnCb(DisconnectCallback cb) { _disconnCb = cb; }
static void callMsgCb(const char* msg) { if(_msgCb) _msgCb(msg); }
static void callDisconnCb() { if(_disconnCb) _disconnCb(); }
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
	"os"
	"sync"
	"time"
	"unsafe"

	"github.com/gorilla/websocket"
)

var (
	conn      *websocket.Conn
	connected bool
	mu        sync.Mutex
	stopChan  chan struct{}
	deviceCfg map[string]interface{}
)

//export SetMessageCallback
func SetMessageCallback(cb C.MessageCallback) {
	C.setMsgCb(cb)
}

//export SetDisconnectCallback
func SetDisconnectCallback(cb C.DisconnectCallback) {
	C.setDisconnCb(cb)
}

//export ConnectWithConfig
func ConnectWithConfig(configPath *C.char) C.int {
	path := C.GoString(configPath)
	data, err := os.ReadFile(path)
	if err != nil {
		fmt.Printf("[%s] 读取配置文件失败: %v\n", now(), err)
		return -3
	}

	var cfg map[string]interface{}
	if err := json.Unmarshal(data, &cfg); err != nil {
		fmt.Printf("[%s] 解析配置文件失败: %v\n", now(), err)
		return -4
	}

	fmt.Printf("[%s] 加载配置: %s\n", now(), path)
	return doConnect(cfg)
}

//export Connect
func Connect(deviceId, macId, sn, cmei, deviceType, firmwareVersion, softwareVersion, sdkVersion, innerType, snPwd *C.char) C.int {
	cfg := map[string]interface{}{
		"deviceId": C.GoString(deviceId), "macId": C.GoString(macId),
		"sn": C.GoString(sn), "cmei": C.GoString(cmei),
		"deviceType": C.GoString(deviceType), "firmwareVersion": C.GoString(firmwareVersion),
		"softwareVersion": C.GoString(softwareVersion), "sdkVersion": C.GoString(sdkVersion),
		"innerType": C.GoString(innerType), "snPwd": C.GoString(snPwd),
	}
	return doConnect(cfg)
}

func doConnect(cfg map[string]interface{}) C.int {
	mu.Lock()
	defer mu.Unlock()

	deviceCfg = cfg

	wsURL, err := getURL(deviceCfg)
	if err != nil {
		fmt.Printf("[%s] 获取URL失败: %v\n", now(), err)
		return -1
	}

	c, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		fmt.Printf("[%s] 连接失败: %v\n", now(), err)
		return -2
	}

	conn = c
	connected = true
	stopChan = make(chan struct{})

	fmt.Printf("[%s] WebSocket连接成功\n", now())

	sendHB()
	go heartbeatLoop()
	go messageLoop()

	return 0
}

//export Disconnect
func Disconnect() {
	mu.Lock()
	defer mu.Unlock()

	if connected {
		connected = false
		if stopChan != nil {
			close(stopChan)
		}
		if conn != nil {
			conn.Close()
			conn = nil
		}
		fmt.Printf("[%s] 已断开连接\n", now())
	}
}

//export IsConnected
func IsConnected() C.int {
	mu.Lock()
	defer mu.Unlock()
	if connected {
		return 1
	}
	return 0
}

//export SendMsg
func SendMsg(message *C.char) C.int {
	mu.Lock()
	defer mu.Unlock()

	if !connected || conn == nil {
		return -1
	}
	err := conn.WriteMessage(websocket.TextMessage, []byte(C.GoString(message)))
	if err != nil {
		return -2
	}
	return 0
}

func heartbeatLoop() {
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			mu.Lock()
			if !connected {
				mu.Unlock()
				return
			}
			sendHB()
			mu.Unlock()
		case <-stopChan:
			return
		}
	}
}

func messageLoop() {
	for {
		mu.Lock()
		if !connected || conn == nil {
			mu.Unlock()
			return
		}
		c := conn
		mu.Unlock()

		_, msg, err := c.ReadMessage()
		if err != nil {
			fmt.Printf("[%s] 连接断开: %v\n", now(), err)
			mu.Lock()
			connected = false
			mu.Unlock()
			
			// 调用断开回调
			C.callDisconnCb()
			return
		}

		msgStr := string(msg)
		fmt.Printf("[%s] 收到: %s\n", now(), msgStr)

		// 调用消息回调
		cMsg := C.CString(msgStr)
		C.callMsgCb(cMsg)
		C.free(unsafe.Pointer(cMsg))
	}
}

func sendHB() {
	if conn == nil {
		return
	}
	hb := map[string]interface{}{
		"deviceId": getStr(deviceCfg, "deviceId", ""),
		"domain":   "KEEP_ALIVE",
		"event":    "keep_alive",
		"eventId":  fmt.Sprintf("hb_%d", time.Now().UnixMilli()),
		"seq":      fmt.Sprintf("%d", time.Now().UnixMilli()),
	}
	data, _ := json.Marshal(hb)
	conn.WriteMessage(websocket.TextMessage, data)
	fmt.Printf("[%s] 心跳已发送\n", now())
}

func getStr(cfg map[string]interface{}, key, def string) string {
	if v, ok := cfg[key].(string); ok {
		return v
	}
	return def
}

func getURL(cfg map[string]interface{}) (string, error) {
	ts := time.Now().UnixMilli()
	sn := getStr(cfg, "sn", "")
	cmei := getStr(cfg, "cmei", "")
	snPwd := getStr(cfg, "snPwd", "")

	data := fmt.Sprintf("sn=%s&cmei=%s&timestamp=%d&snPwd=%s", sn, cmei, ts, snPwd)
	hash := sha256.Sum256([]byte(data))
	sign := base64.StdEncoding.EncodeToString(hash[:])

	appVersion := getStr(cfg, "applicationVersion", "1.0.7")

	otaVersions := cfg["otaVersions"]
	if otaVersions == nil {
		otaVersions = []map[string]interface{}{{"currentVersion": "1.0.0", "versionCode": 0, "versionType": 301}}
	}

	extraParams := cfg["extraParams"]
	if extraParams == nil {
		extraParams = map[string]interface{}{}
	}
	capabilityParams := cfg["capabilityParams"]
	if capabilityParams == nil {
		capabilityParams = map[string]interface{}{}
	}

	req := map[string]interface{}{
		"deviceId": getStr(cfg, "deviceId", ""), "macId": getStr(cfg, "macId", ""),
		"sn": sn, "cmei": cmei,
		"deviceType": getStr(cfg, "deviceType", ""), "firmwareVersion": getStr(cfg, "firmwareVersion", ""),
		"softwareVersion": getStr(cfg, "softwareVersion", ""), "sdkVersion": getStr(cfg, "sdkVersion", ""),
		"innerType": getStr(cfg, "innerType", ""), "sign": sign, "timestamp": ts,
		"applicationVersion": map[string]interface{}{"version": appVersion},
		"extraParams": extraParams, "capabilityParams": capabilityParams,
		"otaVersions": otaVersions, "fromManagedSdk": true,
	}

	body, _ := json.Marshal(req)
	apiURL := getStr(cfg, "apiUrl", "https://business.homibot.komect.com:9443/robot/business/api/device/client/connect/url")

	fmt.Printf("[%s] 请求URL: %s\n", now(), apiURL)
	fmt.Printf("[%s] 请求Body: %s\n", now(), string(body))

	resp, err := http.Post(apiURL, "application/json", bytes.NewBuffer(body))
	if err != nil {
		fmt.Printf("[%s] 请求失败: %v\n", now(), err)
		return "", err
	}
	defer resp.Body.Close()

	respBody, _ := io.ReadAll(resp.Body)
	fmt.Printf("[%s] 响应状态: %s\n", now(), resp.Status)
	fmt.Printf("[%s] 响应Body: %s\n", now(), string(respBody))

	var result map[string]interface{}
	json.Unmarshal(respBody, &result)

	if d, ok := result["data"].(map[string]interface{}); ok {
		if url, ok := d["url"].(string); ok {
			fmt.Printf("[%s] 获取到WebSocket URL: %s\n", now(), url)
			return url, nil
		}
	}
	return "", fmt.Errorf("无法获取URL")
}

func now() string { return time.Now().Format("2006-01-02 15:04:05.000") }

func main() {}
