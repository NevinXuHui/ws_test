package main

/*
#include <stdlib.h>

typedef void (*MessageCallback)(const char* message, int reliable);
typedef void (*DisconnectCallback)();

static MessageCallback _msgCb = NULL;
static DisconnectCallback _disconnCb = NULL;

static void setMsgCb(MessageCallback cb) { _msgCb = cb; }
static void setDisconnCb(DisconnectCallback cb) { _disconnCb = cb; }
static void callMsgCb(const char* msg, int reliable) { if(_msgCb) _msgCb(msg, reliable); }
static void callDisconnCb() { if(_disconnCb) _disconnCb(); }
*/
import "C"
import (
	"context"
	"crypto/tls"
	"encoding/json"
	"log"
	"os"
	"sync"
	"unsafe"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/webtransport-go"
)

var (
	session   *webtransport.Session
	stream    *webtransport.Stream
	dialer    *webtransport.Dialer
	connected bool
	mu        sync.Mutex
	ctx       context.Context
	cancel    context.CancelFunc
)

type Config struct {
	ServerUrl string `json:"serverUrl"`
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
	defer mu.Unlock()

	data, err := os.ReadFile(C.GoString(configPath))
	if err != nil {
		log.Printf("read config failed: %v", err)
		return -1
	}
	var cfg Config
	if err := json.Unmarshal(data, &cfg); err != nil {
		log.Printf("parse config failed: %v", err)
		return -1
	}

	dialer = &webtransport.Dialer{
		TLSClientConfig: &tls.Config{
			InsecureSkipVerify: true,
			NextProtos:         []string{"h3"},
		},
		QUICConfig: &quic.Config{EnableDatagrams: true},
	}

	ctx, cancel = context.WithCancel(context.Background())
	_, session, err = dialer.Dial(ctx, cfg.ServerUrl, nil)
	if err != nil {
		log.Printf("dial failed: %v", err)
		return -1
	}

	// 打开可靠流
	stream, err = session.OpenStreamSync(ctx)
	if err != nil {
		log.Printf("open stream failed: %v", err)
		return -1
	}

	connected = true
	go receiveLoop()
	return 0
}

func receiveLoop() {
	// 接收可靠消息
	go func() {
		buf := make([]byte, 65536)
		for {
			n, err := stream.Read(buf)
			if err != nil {
				break
			}
			msg := C.CString(string(buf[:n]))
			C.callMsgCb(msg, 1) // reliable=1
			C.free(unsafe.Pointer(msg))
		}
	}()

	// 接收不可靠消息
	for {
		data, err := session.ReceiveDatagram(ctx)
		if err != nil {
			break
		}
		msg := C.CString(string(data))
		C.callMsgCb(msg, 0) // reliable=0
		C.free(unsafe.Pointer(msg))
	}

	mu.Lock()
	connected = false
	mu.Unlock()
	C.callDisconnCb()
}

//export SendReliable
func SendReliable(message *C.char) C.int {
	mu.Lock()
	defer mu.Unlock()
	if !connected || stream == nil {
		return -1
	}
	_, err := stream.Write([]byte(C.GoString(message)))
	if err != nil {
		return -1
	}
	return 0
}

//export SendUnreliable
func SendUnreliable(message *C.char) C.int {
	mu.Lock()
	defer mu.Unlock()
	if !connected || session == nil {
		return -1
	}
	if err := session.SendDatagram([]byte(C.GoString(message))); err != nil {
		return -1
	}
	return 0
}

//export Disconnect
func Disconnect() {
	mu.Lock()
	defer mu.Unlock()
	if session != nil {
		session.CloseWithError(0, "bye")
		session = nil
	}
	if dialer != nil {
		dialer.Close()
		dialer = nil
	}
	if cancel != nil {
		cancel()
	}
	connected = false
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

func main() {}
