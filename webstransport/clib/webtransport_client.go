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
	"sync/atomic"
	"time"
	"unsafe"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/webtransport-go"
)

var (
	session      atomic.Pointer[webtransport.Session]
	stream       atomic.Pointer[webtransport.Stream]
	dialer       *webtransport.Dialer
	connected    atomic.Bool
	connecting   atomic.Bool
	ctx          context.Context
	cancel       context.CancelFunc
	disconnected atomic.Bool
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
	if !connecting.CompareAndSwap(false, true) {
		return -2 // 正在连接中
	}
	defer connecting.Store(false)

	// 先断开旧连接
	doDisconnect()

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
	log.Printf("connecting to %s", cfg.ServerUrl)

	dialer = &webtransport.Dialer{
		TLSClientConfig: &tls.Config{
			InsecureSkipVerify: true,
			NextProtos:         []string{"h3"},
		},
		QUICConfig: &quic.Config{
			EnableDatagrams: true,
			MaxIdleTimeout:  5 * time.Second,
			KeepAlivePeriod: 2 * time.Second,
		},
	}

	ctx, cancel = context.WithCancel(context.Background())
	disconnected.Store(false)

	_, sess, err := dialer.Dial(ctx, cfg.ServerUrl, nil)
	if err != nil {
		log.Printf("dial failed: %v", err)
		return -1
	}
	session.Store(sess)

	st, err := sess.OpenStreamSync(ctx)
	if err != nil {
		log.Printf("open stream failed: %v", err)
		return -1
	}
	stream.Store(st)

	connected.Store(true)
	log.Println("connected")

	go receiveLoop(sess, st)
	return 0
}

func doDisconnect() {
	connected.Store(false)
	if cancel != nil {
		cancel()
		cancel = nil
	}
	if sess := session.Load(); sess != nil {
		sess.CloseWithError(0, "bye")
		session.Store(nil)
	}
	if dialer != nil {
		dialer.Close()
		dialer = nil
	}
	stream.Store(nil)
}

func onDisconnect() {
	if !disconnected.CompareAndSwap(false, true) {
		return
	}
	connected.Store(false)
	log.Println("disconnected")
	C.callDisconnCb()
}

func receiveLoop(sess *webtransport.Session, st *webtransport.Stream) {
	go func() {
		<-sess.Context().Done()
		onDisconnect()
	}()

	go func() {
		buf := make([]byte, 65536)
		for {
			n, err := st.Read(buf)
			if err != nil {
				onDisconnect()
				return
			}
			msg := C.CString(string(buf[:n]))
			C.callMsgCb(msg, 1)
			C.free(unsafe.Pointer(msg))
		}
	}()

	for {
		data, err := sess.ReceiveDatagram(ctx)
		if err != nil {
			onDisconnect()
			return
		}
		msg := C.CString(string(data))
		C.callMsgCb(msg, 0)
		C.free(unsafe.Pointer(msg))
	}
}

//export SendReliable
func SendReliable(message *C.char) C.int {
	st := stream.Load()
	if !connected.Load() || st == nil {
		return -1
	}
	_, err := st.Write([]byte(C.GoString(message)))
	if err != nil {
		log.Printf("send reliable error: %v", err)
		return -1
	}
	return 0
}

//export SendUnreliable
func SendUnreliable(message *C.char) C.int {
	sess := session.Load()
	if !connected.Load() || sess == nil {
		return -1
	}
	if err := sess.SendDatagram([]byte(C.GoString(message))); err != nil {
		log.Printf("send unreliable error: %v", err)
		return -1
	}
	return 0
}

//export Disconnect
func Disconnect() {
	doDisconnect()
}

//export IsConnected
func IsConnected() C.int {
	if connected.Load() {
		return 1
	}
	return 0
}

func main() {}
