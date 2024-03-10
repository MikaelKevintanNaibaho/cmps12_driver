# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c99
LDFLAGS = -lm

# Source files
SRC = main.c cmps12_i2c.c

# Object files directory
OBJ_DIR = build/obj

# Executable directory
BIN_DIR = build/bin

# Object files
OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(SRC))

# Executable name
TARGET = $(BIN_DIR)/cmps12_reader

# Formatting and Static Analysis tools
CLANG_FORMAT = clang-format
CPPCHECK = cppcheck

.PHONY: all clean format check

all: $(TARGET)

$(TARGET): $(OBJ) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

format:
	$(CLANG_FORMAT) -i $(SRC)

cppcheck:
	$(CPPCHECK) --enable=all $(SRC)

clean:
	rm -rf build

