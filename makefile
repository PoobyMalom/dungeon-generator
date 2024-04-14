# Compiler
CXX := g++

# Compiler flags
CXXFLAGS := -std=c++11 -Wall -Wextra -pedantic

# Directories
SRCDIR := .
INCDIR := include
LIBDIR := lib
BINDIR := .

# Source files
SRCS := $(wildcard $(SRCDIR)/*.cpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(BINDIR)/%.o)

# Executable name
TARGET := main

# Libraries
LIBS := -lSDL2

# Include directories
INCLUDES := -I$(INCDIR)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$(TARGET) $(OBJS) -L$(LIBDIR) $(LIBS)

$(BINDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

clean:
	$(RM) $(BINDIR)/*.o $(BINDIR)/$(TARGET)