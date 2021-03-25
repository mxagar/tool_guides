EXE = week_2_challenge
OBJS = main.o
CLEAN_RM =

#include ../_make/generic.mk
include ../../utils/_make/generic.mk

# Directory-specific rules:
week_2_challenge: main.cpp
	$(LD) $^ $(LDFLAGS) -o $@

all: week_2_challenge
CLEAN_RM += week_2_challenge
