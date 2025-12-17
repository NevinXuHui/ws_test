package main

// 配置示例 - 复制此文件为 config.go 并填入真实参数
var exampleConfig = DeviceConfig{
	DeviceID:         "1222004229866666660001505", // 设备ID（通常使用SN号）
	MacID:           "FC23CD911857",                // 设备MAC地址
	SN:              "1222004229866666660001505",   // 设备序列号
	CMEI:            "866666660001505",             // 设备CMEI
	DeviceType:      "591884",                      // 设备类型
	FirmwareVersion: "1.0.1",                       // 固件版本
	SoftwareVersion: "1.0.7",                       // 软件版本
	SDKVersion:      "1.0.7",                       // SDK版本
	InnerType:       "1",                           // 内部类型 (1:单语音模块; 2:逗宠机器人; 3:服务机器人; 4:四足机器人; 5:轮式双臂; 6:双足机器人; 7:扫地机器人; 8:扩展类型)
	SNPwd:           "14mrrGJH",                    // 设备密码
}
