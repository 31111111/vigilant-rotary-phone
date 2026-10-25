# patch2（θ1 双分支）x86 验证 Makefile
CC     ?= gcc
CFLAGS ?= -std=gnu11 -O2 -Wall -Wextra
INC     = -I.

all: verify_patch2

verify_patch2: verify_patch2.c robot_kinematics.c robot_kinematics.h robot.h usart.h
	$(CC) $(CFLAGS) $(INC) verify_patch2.c -lm -o verify_patch2

run: verify_patch2
	./verify_patch2

clean:
	rm -f verify_patch2

.PHONY: all run clean
