TESTS := $(filter-out rv32ui-p-ma_data.hex rv32ui-p-fence_i.hex, $(wildcard rv32ui-p-*.hex))

all:

run-32ui:
	@for test in  $(TESTS); do ../tinyrv -s $$test || exit 1; done

run: run-32ui

clean: