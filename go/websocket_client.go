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
	"log"
	"net/http"
	"os"
	"sync"
	"time"
	"unsafe"

	"github.com/gorilla/websocket"
)

var (
	conn        *websocket.Conn
	connID      int
	connected   bool
	connecting  bool
	reconnecting bool
	mu          sync.Mutex
	stopChan    chan struct{}
	wg          sync.WaitGroup
	deviceCfg   map[string]interface{}
	logger      *log.Logger
	logFile     *os.File
)

func initLogger(path string, logToConsole bool) {
	if path == "" {
		logger = log.New(os.Stdout, "", 0)
		return
	}
	// 如果是目录，自动生成带日期的文件名
	if info, err := os.Stat(path); err == nil && info.IsDir() {
		path = fmt.Sprintf("%s/websocket_%s.log", path, time.Now().Format("2006-01-02"))
	} else if err != nil {
		os.MkdirAll(path, 0755)
		path = fmt.Sprintf("%s/websocket_%s.log", path, time.Now().Format("2006-01-02"))
	}
	var err error
	logFile, err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)
	if err != nil {
		logger = log.New(os.Stdout, "", 0)
		return
	}
	if logToConsole {
		logger = log.New(io.MultiWriter(os.Stdout, logFile), "", 0)
	} else {
		logger = log.New(logFile, "", 0)
	}
}

const (
	colorReset  = "\033[0m"
	colorRed    = "\033[31m"
	colorGreen  = "\033[32m"
	colorYellow = "\033[33m"
	colorBlue   = "\033[34m"
	colorCyan   = "\033[36m"
)

func logf(format string, args ...interface{}) {
	if logger == nil {
		logger = log.New(os.Stdout, "", 0)
	}
	logger.Printf("[%s] "+format, append([]interface{}{now()}, args...)...)
}

func logSend(id int, msg string) {
	logf("%s[conn#%d] 发送: %s%s", colorGreen, id, msg, colorReset)
}

func logRecv(id int, msg string) {
	logf("%s[conn#%d] 收到: %s%s", colorCyan, id, msg, colorReset)
}

func logErr(format string, args ...interface{}) {
	logf("%s"+format+"%s", append(append([]interface{}{colorRed}, args...), colorReset)...)
}

func logWarn(format string, args ...interface{}) {
	logf("%s"+format+"%s", append(append([]interface{}{colorYellow}, args...), colorReset)...)
}

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
	mu.Lock()
	if connecting {
		mu.Unlock()
		return -3
	}
	connecting = true
	mu.Unlock()

	path := C.GoString(configPath)
	data, err := os.ReadFile(path)
	if err != nil {
		logf("读取配置文件失败: %v", err)
		mu.Lock()
		connecting = false
		mu.Unlock()
		return -4
	}

	var cfg map[string]interface{}
	if err := json.Unmarshal(data, &cfg); err != nil {
		logf("解析配置文件失败: %v", err)
		mu.Lock()
		connecting = false
		mu.Unlock()
		return -5
	}

	// 初始化日志
	logToConsole := true
	if v, ok := cfg["logToConsole"].(bool); ok {
		logToConsole = v
	}
	initLogger(getStr(cfg, "logPath", ""), logToConsole)

	logf("加载配置: %s", path)
	return doConnectInternal(cfg)
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
	if connecting {
		mu.Unlock()
		return -3
	}
	connecting = true
	mu.Unlock()
	return doConnectInternal(cfg)
}

func doConnectInternal(cfg map[string]interface{}) C.int {
	mu.Lock()
	logf("doConnect 开始")

	// 先关闭旧连接
	if connected || conn != nil {
		reconnecting = true
		connected = false
		logf("设置 reconnecting=true, connected=false")
		if stopChan != nil {
			close(stopChan)
			logf("stopChan 已关闭")
		}
		if conn != nil {
			conn.Close()
			logf("旧连接已关闭")
		}
		conn = nil
		mu.Unlock()
		
		// 等待旧 goroutine 退出
		logf("等待旧 goroutine 退出")
		wg.Wait()
		logf("旧 goroutine 已退出")
		
		mu.Lock()
	}

	deviceCfg = cfg
	reconnecting = false

	logf("开始获取 WebSocket URL")
	mu.Unlock()
	wsURL, err := getURL(deviceCfg)
	mu.Lock()
	if err != nil {
		logf("获取URL失败: %v", err)
		connecting = false
		mu.Unlock()
		return -1
	}

	logf("开始连接 WebSocket")
	c, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		logf("连接失败: %v", err)
		connecting = false
		mu.Unlock()
		return -2
	}

	conn = c
	connected = true
	connecting = false
	connID++
	currentConnID := connID
	stopChan = make(chan struct{})

	logf("[conn#%d] WebSocket连接成功", currentConnID)

	sendHB()
	wg.Add(2)
	go heartbeatLoop(currentConnID)
	go messageLoop(currentConnID)

	mu.Unlock()
	return 0
}

//export Disconnect
func Disconnect() {
	mu.Lock()
	defer mu.Unlock()

	logf("Disconnect 调用, connected=%v", connected)
	if connected {
		connected = false
		if stopChan != nil {
			close(stopChan)
		}
		if conn != nil {
			conn.Close()
			conn = nil
		}
		logf("已断开连接")
	}
	if logFile != nil {
		logFile.Close()
		logFile = nil
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
	msg := C.GoString(message)
	err := conn.WriteMessage(websocket.TextMessage, []byte(msg))
	if err != nil {
		logErr("[conn#%d] 发送失败: %v", connID, err)
		return -2
	}
	logSend(connID, msg)
	return 0
}

func heartbeatLoop(id int) {
	defer wg.Done()
	logf("[conn#%d] heartbeatLoop 启动", id)
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			mu.Lock()
			if !connected {
				logf("[conn#%d] heartbeatLoop 退出: connected=false", id)
				mu.Unlock()
				return
			}
			sendHB()
			mu.Unlock()
		case <-stopChan:
			logf("[conn#%d] heartbeatLoop 退出: stopChan 关闭", id)
			return
		}
	}
}

func messageLoop(id int) {
	defer wg.Done()
	logf("[conn#%d] messageLoop 启动", id)
	for {
		mu.Lock()
		if !connected || conn == nil {
			logf("[conn#%d] messageLoop 退出: connected=%v, conn=%v", id, connected, conn != nil)
			mu.Unlock()
			return
		}
		c := conn
		mu.Unlock()

		_, msg, err := c.ReadMessage()
		if err != nil {
			mu.Lock()
			logErr("[conn#%d] ReadMessage 错误: %v, reconnecting=%v", id, err, reconnecting)
			// 主动重连时不调用断开回调
			if reconnecting {
				logf("[conn#%d] 主动重连中，跳过断开回调", id)
				mu.Unlock()
				return
			}
			connected = false
			mu.Unlock()
			
			logErr("[conn#%d] 连接异常断开，调用断开回调", id)
			C.callDisconnCb()
			return
		}

		msgStr := string(msg)
		logRecv(id, msgStr)

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
	logSend(connID, string(data))
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

	logf("请求URL: %s", apiURL)
	logf("请求Body: %s", string(body))

	resp, err := http.Post(apiURL, "application/json", bytes.NewBuffer(body))
	if err != nil {
		logf("请求失败: %v", err)
		return "", err
	}
	defer resp.Body.Close()

	respBody, _ := io.ReadAll(resp.Body)
	logf("响应状态: %s", resp.Status)
	logf("响应Body: %s", string(respBody))

	var result map[string]interface{}
	json.Unmarshal(respBody, &result)

	if d, ok := result["data"].(map[string]interface{}); ok {
		if url, ok := d["url"].(string); ok {
			logf("获取到WebSocket URL: %s", url)
			return url, nil
		}
	}
	return "", fmt.Errorf("无法获取URL")
}

func now() string { return time.Now().Format("2006-01-02 15:04:05.000") }

func main() {}
