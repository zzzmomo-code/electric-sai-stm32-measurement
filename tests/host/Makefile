CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../../Core/USER
TARGET := test_ad7606_state.exe

.PHONY: all test clean

all: $(TARGET)

$(TARGET): test_ad7606_state.c ../../Core/USER/ad7606_state.c ../../Core/USER/ad7606_state.h
	$(CC) $(CFLAGS) test_ad7606_state.c ../../Core/USER/ad7606_state.c -o $(TARGET)

test: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
