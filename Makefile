TARGET_ARCH = linux64

MODULE = ./2d_emesh_8x4
SRCS = ./test.cpp
OBJS = $(SRCS:.cc=.o)

include ./Makefile.defs
