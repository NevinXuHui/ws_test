package main

import (
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/webtransport-go"
)

type Config struct {
	ServerUrl string `json:"serverUrl"`
}

func loadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var cfg Config
	if err := json.Unmarshal(data, &cfg); err != nil {
		return nil, err
	}
	return &cfg, nil
}

func main() {
	cfg, err := loadConfig("config.json")
	if err != nil {
		log.Fatalf("load config failed: %v", err)
	}
	log.Printf("server url: %s", cfg.ServerUrl)

	dialer := &webtransport.Dialer{
		TLSClientConfig: &tls.Config{
			InsecureSkipVerify: true,
			NextProtos:         []string{"h3"},
		},
		QUICConfig: &quic.Config{
			EnableDatagrams: true,
		},
	}
	defer dialer.Close()

	ctx := context.Background()
	_, session, err := dialer.Dial(ctx, cfg.ServerUrl, nil)
	if err != nil {
		log.Fatalf("dial failed: %v", err)
	}
	defer session.CloseWithError(0, "bye")

	log.Println("connected to server")

	// 可靠传输：双向流
	go reliableTransport(session)

	// 不可靠传输：数据报
	go unreliableTransport(session)

	// 接收响应
	go receiveStreamResponse(session)
	go receiveDatagramResponse(session)

	select {} // 保持运行
}

// 可靠传输 - 使用双向流
func reliableTransport(session *webtransport.Session) {
	stream, err := session.OpenStreamSync(context.Background())
	if err != nil {
		log.Printf("open stream error: %v", err)
		return
	}

	ticker := time.NewTicker(2 * time.Second)
	for range ticker.C {
		msg := fmt.Sprintf("reliable msg at %s", time.Now().Format("15:04:05"))
		_, err := stream.Write([]byte(msg))
		if err != nil {
			log.Printf("stream write error: %v", err)
			return
		}
		log.Printf("[Stream] sent: %s", msg)
	}
}

// 不可靠传输 - 使用数据报
func unreliableTransport(session *webtransport.Session) {
	ticker := time.NewTicker(1 * time.Second)
	for range ticker.C {
		msg := fmt.Sprintf("datagram at %s", time.Now().Format("15:04:05"))
		if err := session.SendDatagram([]byte(msg)); err != nil {
			log.Printf("send datagram error: %v", err)
			continue
		}
		log.Printf("[Datagram] sent: %s", msg)
	}
}

func receiveStreamResponse(session *webtransport.Session) {
	for {
		stream, err := session.AcceptStream(context.Background())
		if err != nil {
			return
		}
		go func(s *webtransport.Stream) {
			buf := make([]byte, 4096)
			for {
				n, err := s.Read(buf)
				if err != nil {
					return
				}
				log.Printf("[Stream] response: %s", string(buf[:n]))
			}
		}(stream)
	}
}

func receiveDatagramResponse(session *webtransport.Session) {
	for {
		data, err := session.ReceiveDatagram(context.Background())
		if err != nil {
			log.Printf("receive datagram error: %v", err)
			return
		}
		log.Printf("[Datagram] response: %s", string(data))
	}
}
