# patch1（IK 选解位置残差校验）x86 验证 Makefile
CC     ?= gcc
CFLAGS ?= -std=gnu11 -O2 -Wall -Wextra
INC     = -I.

all: verify_patched diag_patch

verify_patched: verify_patched.c robot_kinematics.c robot_kinematics.h robot.h usart.h
	$(CC) $(CFLAGS) $(INC) verify_patched.c -lm -o verify_patched

diag_patch: diag_patch.c robot_kinematics.c robot_kinematics.h robot.h usart.h
	$(CC) $(CFLAGS) $(INC) diag_patch.c -lm -o diag_patch

run: verify_patched
	./verify_patched

clean:
	rm -f verify_patched diag_patch

.PHONY: all run clean
