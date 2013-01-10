PARAMETER VERSION = 2.0.0
PARAMETER HW_SPEC_FILE = mbvanilla.mhs

BEGIN PROCESSOR
 PARAMETER HW_INSTANCE = mblaze
 PARAMETER DRIVER_NAME = cpu
 PARAMETER DRIVER_VER = 1.00.a
 PARAMETER EXECUTABLE = mblaze/code/executable.elf
 PARAMETER COMPILER = mb-gcc
 PARAMETER ARCHIVER = mb-ar
 PARAMETER DEFAULT_INIT = XMDSTUB
 PARAMETER DEBUG_PERIPHERAL = debug_uart
 PARAMETER STDIN = uartlite
 PARAMETER STDOUT = uartlite
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = memcon
 PARAMETER DRIVER_NAME = emc
 PARAMETER DRIVER_VER = 1.00.a
 PARAMETER LEVEL = 1
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = uartlite
 PARAMETER DRIVER_NAME = uartlite
 PARAMETER DRIVER_VER = 1.00.b
 PARAMETER LEVEL = 0
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = timer
 PARAMETER DRIVER_NAME = tmrctr
 PARAMETER DRIVER_VER = 1.00.b
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = intc
 PARAMETER DRIVER_NAME = intc
 PARAMETER DRIVER_VER = 1.00.b
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = debug_uart
 PARAMETER DRIVER_NAME = uartlite
 PARAMETER DRIVER_VER = 1.00.b
 PARAMETER LEVEL = 0
END

BEGIN DRIVER
 PARAMETER HW_INSTANCE = gpio_instance
 PARAMETER DRIVER_NAME = gpio
 PARAMETER DRIVER_VER = 1.00.a
 PARAMETER LEVEL = 0
END