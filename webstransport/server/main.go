package main

import (
	"crypto/tls"
	"fmt"
	"log"
	"net/http"
	"time"

	"github.com/quic-go/quic-go"
	"github.com/quic-go/quic-go/http3"
	"github.com/quic-go/webtransport-go"
)

var deviceCounter int

func main() {
	certFile := "cert.pem"
	keyFile := "key.pem"

	server := &webtransport.Server{
		H3: http3.Server{
			Addr:      ":4433",
			TLSConfig: generateTLSConfig(certFile, keyFile),
			QUICConfig: &quic.Config{
				EnableDatagrams: true,
				MaxIdleTimeout:  5 * time.Second,
				KeepAlivePeriod: 2 * time.Second,
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
		deviceCounter++
		deviceID := fmt.Sprintf("Device-%d", deviceCounter)
		addr := session.RemoteAddr().String()
		log.Printf("[%s] connected (%s)", deviceID, addr)
		go handleSession(session, deviceID)
	})

	log.Println("WebTransport server listening on :4433")
	if err := server.ListenAndServeTLS(certFile, keyFile); err != nil {
		log.Fatal(err)
	}
}

func handleSession(session *webtransport.Session, addr string) {
	ctx := session.Context()

	go func() {
		for {
			stream, err := session.AcceptStream(ctx)
			if err != nil {
				return
			}
			go handleStream(stream, addr)
		}
	}()

	go func() {
		for {
			data, err := session.ReceiveDatagram(ctx)
			if err != nil {
				return
			}
			log.Printf("[%s] [Datagram] %s", addr, string(data))
			session.SendDatagram(data)
		}
	}()

	<-ctx.Done()
	log.Printf("[%s] disconnected", addr)
}

func handleStream(stream *webtransport.Stream, addr string) {
	defer stream.Close()
	buf := make([]byte, 4096)
	for {
		n, err := stream.Read(buf)
		if err != nil {
			return
		}
		log.Printf("[%s] [Stream] %s", addr, string(buf[:n]))
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
