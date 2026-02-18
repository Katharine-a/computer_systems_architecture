TESTS := $(filter-out rv32ui-p-ma_data.hex rv32ui-p-fence_i.hex, $(wildcard rv32ui-p-*.hex))

all:

run:
	@for test in  $(TESTS); do ../tinyrv -s $$test || exit 1; done

run-g:
	@for test in  $(TESTS); do ../tinyrv -sg $$test || exit 1; done

clean:
