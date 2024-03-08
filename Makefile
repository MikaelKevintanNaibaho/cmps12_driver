# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c99
LDFLAGS = -lm

# Source files
SRC = main.c cmps12_i2c.c

# Object files
OBJ = $(SRC:.c=.o)

# Executable name
TARGET = cmps12_reader

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
