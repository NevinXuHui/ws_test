ARCH := $(shell uname -m)
LIB_DIR := lib/$(ARCH)
BIN_DIR := bin/$(ARCH)
ROOT_DIR := $(shell pwd)

all: $(BIN_DIR)/example

$(BIN_DIR)/example: example/example.cpp
	@mkdir -p $(BIN_DIR)
	g++ -o $@ $< -I include -L $(ROOT_DIR)/$(LIB_DIR) -lwebtransport_client -Wl,-rpath,$(ROOT_DIR)/$(LIB_DIR)

run: $(BIN_DIR)/example
	cp config.json $(BIN_DIR)/ && cd $(BIN_DIR) && ./example

clean:
	rm -rf bin
