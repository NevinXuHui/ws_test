package main

import (
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"sync"
	"time"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/webtransport-go"
)

type Config struct {
	ServerUrl string `json:"serverUrl"`
}

var (
	cfg    *Config
	dialer *webtransport.Dialer
)

func main() {
	configFile := "config.json"
	if len(os.Args) > 1 {
		configFile = "config." + os.Args[1] + ".json"
	}
	var err error
	cfg, err = loadConfig(configFile)
	if err != nil {
		log.Fatalf("load config failed: %v", err)
	}
	log.Printf("server url: %s", cfg.ServerUrl)

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
	defer dialer.Close()

	for {
		runSession()
		log.Println("reconnecting in 2s...")
		time.Sleep(2 * time.Second)
	}
}

func runSession() {
	ctx := context.Background()
	_, session, err := dialer.Dial(ctx, cfg.ServerUrl, nil)
	if err != nil {
		log.Printf("dial failed: %v", err)
		return
	}
	defer session.CloseWithError(0, "bye")
	log.Println("connected")

	ctx, cancel := context.WithCancel(ctx)
	var once sync.Once
	onDisconnect := func() {
		once.Do(func() {
			log.Println("disconnected")
			cancel()
		})
	}

	go func() {
		<-session.Context().Done()
		onDisconnect()
	}()

	go reliableTransport(ctx, session, onDisconnect)

	<-ctx.Done()
}

func loadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var c Config
	if err := json.Unmarshal(data, &c); err != nil {
		return nil, err
	}
	return &c, nil
}

func reliableTransport(ctx context.Context, session *webtransport.Session, onDisconnect func()) {
	stream, err := session.OpenStreamSync(ctx)
	if err != nil {
		log.Printf("open stream error: %v", err)
		onDisconnect()
		return
	}

	go func() {
		buf := make([]byte, 4096)
		for {
			n, err := stream.Read(buf)
			if err != nil {
				onDisconnect()
				return
			}
			log.Printf("[Stream] response: %s", string(buf[:n]))
		}
	}()

	ticker := time.NewTicker(2 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			msg := fmt.Sprintf("reliable msg at %s", time.Now().Format("15:04:05"))
			if _, err := stream.Write([]byte(msg)); err != nil {
				onDisconnect()
				return
			}
			log.Printf("[Stream] sent: %s", msg)
		}
	}
}

func unreliableTransport(ctx context.Context, session *webtransport.Session, onDisconnect func()) {
	go func() {
		for {
			data, err := session.ReceiveDatagram(ctx)
			if err != nil {
				return
			}
			log.Printf("[Datagram] response: %s", string(data))
		}
	}()

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			msg := fmt.Sprintf("datagram at %s", time.Now().Format("15:04:05"))
			if err := session.SendDatagram([]byte(msg)); err != nil {
				onDisconnect()
				return
			}
			log.Printf("[Datagram] sent: %s", msg)
		}
	}
}
