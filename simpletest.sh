#!/bin/bash

mavlink-server serial:///dev/ttyAMA0?baudrate=921600 tcpserver://0.0.0.0:5760 udpserver://0.0.0.0:14550
