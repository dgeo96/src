#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y		:= irq.o time.o gpio.o
obj-m		:=
obj-n		:=
obj-		:=

#extra-y	:= head.o

# CPU-specific support
obj-$(CONFIG_CPU_LPC2468) += lpc2468_devices.o
obj-$(CONFIG_CPU_LPC2478) += lpc2468_devices.o
obj-$(CONFIG_BOARD_LPC2468_EA) += lpc2468_ea_board.o
obj-$(CONFIG_BOARD_LPC_E2468) += lpc_e2468.o
obj-$(CONFIG_BOARD_LPC_2478STK) += lpc_2478stk.o



