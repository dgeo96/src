EXEC = test_ts #ad_touchscreen
OBJS = test_ts.o #ad_touchscreen.o

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $@.o $(LDLIBS)

romfs:
	$(ROMFSINST) -e CONFIG_USER_TEST_TEST_TS	/bin/test_ts
	#$(ROMFSINST) -e CONFIG_USER_TEST_AD_TOUCHSCREEN	/bin/ad_touchscreen
clean:
	-rm -f $(EXEC) *.elf *.gdb *.o
