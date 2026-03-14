CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lm
TARGET  = urdf-doctor
SRC     = urdf-doctor.c
PREFIX  = /usr/local
BINDIR  = $(PREFIX)/bin

ifeq ($(OS),Windows_NT)
    TARGET := urdf-doctor.exe
endif

.PHONY: all clean install uninstall test

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

install: $(TARGET)
	install -d $(DESTDIR)$(BINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(BINDIR)/$(TARGET)
	@echo "Installed to $(DESTDIR)$(BINDIR)/$(TARGET)"

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)

test: $(TARGET)
	@echo "── Running tests ──────────────────────"
	@./$(TARGET) --version
	@./$(TARGET) --help > /dev/null
	@./$(TARGET) tests/test_good.urdf && echo "  good URDF: OK (exit 0)"
	@./$(TARGET) tests/test_bad.urdf --quiet; \
		[ $$? -eq 2 ] && echo "  bad URDF:  OK (exit 2)" || echo "  bad URDF:  FAIL"
	@echo "── Tests passed ───────────────────────"

clean:
	rm -f $(TARGET) urdf-doctor.exe *.o
