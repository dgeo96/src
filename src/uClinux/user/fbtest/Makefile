EXEC = fbtest
OBJS = fbtest.o

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) -lm $(LDLIBS)

romfs:
	$(ROMFSINST)    /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

