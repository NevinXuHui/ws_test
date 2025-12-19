package main

import (
	"crypto/tls"
	"log"
	"net/http"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/quic-go/http3"
	"github.com/quic-go/webtransport-go"
)

func main() {
	certFile := "cert.pem"
	keyFile := "key.pem"

	server := &webtransport.Server{
		H3: http3.Server{
			Addr:      ":4433",
			TLSConfig: generateTLSConfig(certFile, keyFile),
			QUICConfig: &quic.Config{
				EnableDatagrams: true, // 启用不可靠传输
			},
		},
		CheckOrigin: func(r *http.Request) bool { return true },
	}

	http.HandleFunc("/webtransport", func(w http.ResponseWriter, r *http.Request) {
		session, err := server.Upgrade(w, r)
		if err != nil {
			log.Printf("upgrade failed: %v", err)
			return
		}
		log.Printf("new session from %s", session.RemoteAddr())
		go handleSession(session)
	})

	log.Println("WebTransport server listening on :4433")
	if err := server.ListenAndServeTLS(certFile, keyFile); err != nil {
		log.Fatal(err)
	}
}

func handleSession(session *webtransport.Session) {
	ctx := session.Context()

	// 处理可靠双向流
	go func() {
		for {
			stream, err := session.AcceptStream(ctx)
			if err != nil {
				log.Printf("accept stream error: %v", err)
				return
			}
			go handleStream(stream)
		}
	}()

	// 处理不可靠数据报
	go func() {
		for {
			data, err := session.ReceiveDatagram(ctx)
			if err != nil {
				log.Printf("receive datagram error: %v", err)
				return
			}
			log.Printf("[Datagram] received: %s", string(data))
			// 回显
			if err := session.SendDatagram(data); err != nil {
				log.Printf("send datagram error: %v", err)
			}
		}
	}()

	<-ctx.Done()
	log.Println("session closed")
}

func handleStream(stream *webtransport.Stream) {
	defer stream.Close()
	buf := make([]byte, 4096)
	for {
		n, err := stream.Read(buf)
		if err != nil {
			return
		}
		log.Printf("[Stream] received: %s", string(buf[:n]))
		// 回显
		stream.Write(buf[:n])
	}
}

func generateTLSConfig(certFile, keyFile string) *tls.Config {
	cert, err := tls.LoadX509KeyPair(certFile, keyFile)
	if err != nil {
		log.Fatalf("load cert failed: %v", err)
	}
	return &tls.Config{
		Certificates: []tls.Certificate{cert},
		NextProtos:   []string{"h3"},
	}
}
