XLEN ?= 32
DESTDIR ?= $(CURDIR)
COMMON_DIR = $(abspath common)
SRC_DIR = $(abspath src)

CXXFLAGS += -std=c++11 -Wall -Wextra -Wfatal-errors
CXXFLAGS += -fPIC -Wno-maybe-uninitialized
CXXFLAGS += -I$(CURDIR) -I$(COMMON_DIR)
CXXFLAGS += -DXLEN_$(XLEN)
CXXFLAGS += $(CONFIGS)

LDFLAGS +=

SRCS = $(COMMON_DIR)/util.cpp $(COMMON_DIR)/mem.cpp
SRCS += $(SRC_DIR)/main.cpp $(SRC_DIR)/processor.cpp $(SRC_DIR)/core.cpp $(SRC_DIR)/decode.cpp $(SRC_DIR)/execute.cpp
SRCS += $(SRC_DIR)/gshare.cpp

# Debugigng
ifdef DEBUG
	CXXFLAGS += -g -O0 -DDEBUG_LEVEL=$(DEBUG)
else
	CXXFLAGS += -O2 -DNDEBUG
endif

PROJECT = tinyrv

all: $(DESTDIR)/$(PROJECT)

$(DESTDIR)/$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.depend: $(SRCS)
	$(CXX) $(CXXFLAGS) -MM $^ > .depend;

test: $(DESTDIR)/$(PROJECT)
	$(MAKE) -C tests run

test-g: $(DESTDIR)/$(PROJECT)
	$(MAKE) -C tests run-g

submit:
	@echo "-- ZIPPING ALL THE FILE ---------"
	zip submission.zip src/*

clean:
	rm -rf $(DESTDIR)/$(PROJECT)
