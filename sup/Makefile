# adjust the following line to point to your git checkout of bedrock
BEDROCK_BASE ?= $(HOME)/src/Firmware/bedrock
include $(BEDROCK_BASE)/dir_list.mk

# Possibly common setup / configuration
include $(BUILD_DIR)/top_rules.mk

MERGED_FILE = badgerMerged.v

all: $(MERGED_FILE)

# Packet Badger synthesizable code
RTEFI_CLIENT_LIST = hello.v
include $(BADGER_DIR)/rules.mk

vpath %.v $(DSP_DIR) $(SERIAL_IO_DIR)

$(MERGED_FILE): $(RTEFI_V)
	iverilog -E -o $@ $^

clean:
	rm -f $(RTEFI_CLEAN) $(MERGED_FILE)
