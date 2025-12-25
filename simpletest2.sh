#!/bin/bash

mavlink-server serial:///dev/ttyS0?baudrate=921600 tcpserver://0.0.0.0:5760 udpclient://127.0.0.1:14540
