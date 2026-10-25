# 数值 IK（numeric_ik）x86 验证 Makefile
# 用法：make && ./verify_numeric_ik
CC      ?= gcc
CFLAGS  ?= -std=gnu11 -O2 -Wall -Wextra
# 固件 patch1 源码（robot_kinematics.c / robot.h / usart.h 等）在 ../patch
INC      = -I. -I../patch

all: verify_numeric_ik

numeric_ik.o: numeric_ik.c numeric_ik.h
	$(CC) $(CFLAGS) $(INC) -c numeric_ik.c -o numeric_ik.o

verify_numeric_ik: verify_numeric_ik.c numeric_ik.o numeric_ik.h
	$(CC) $(CFLAGS) $(INC) verify_numeric_ik.c numeric_ik.o -lm -o verify_numeric_ik

run: verify_numeric_ik
	./verify_numeric_ik

clean:
	rm -f numeric_ik.o verify_numeric_ik

.PHONY: all run clean
