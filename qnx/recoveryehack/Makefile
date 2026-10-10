ARTIFACT = recoveryehack

PLATFORM ?= aarch64le
BUILD_PROFILE ?= debug
CONFIG_NAME ?= $(PLATFORM)-$(BUILD_PROFILE)

OUTPUT_DIR = build/$(CONFIG_NAME)
TARGET = $(OUTPUT_DIR)/$(ARTIFACT)

CC = qcc -Vgcc_nto$(PLATFORM)
CXX = q++ -Vgcc_nto$(PLATFORM)_cxx
LD = $(CC)

CFLAGS += -Wall -Wextra
CXXFLAGS += -Wall -Wextra
LDFLAGS +=

LIBS += -lsocket -lm

SRCS = src/recoveryehack.c
OBJS = $(patsubst %.c,$(OUTPUT_DIR)/%.o,$(SRCS))

.PHONY: all clean rebuild

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) -o $@ $(LDFLAGS) $(OBJS) $(LIBS)

$(OUTPUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build

rebuild: clean
	$(MAKE) all