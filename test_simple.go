package main

import (
	"fmt"
	"time"
)

func main() {
	fmt.Printf("[%s] Simple test started\n", time.Now().Format("15:04:05.000"))
	
	// 模拟连接过程
	for i := 0; i < 5; i++ {
		fmt.Printf("[%s] Step %d\n", time.Now().Format("15:04:05.000"), i+1)
		time.Sleep(1 * time.Second)
	}
	
	fmt.Printf("[%s] Test completed\n", time.Now().Format("15:04:05.000"))
}
