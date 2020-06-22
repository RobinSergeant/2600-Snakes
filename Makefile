ROM ?= snakes
INCLUDE_PATH ?= $(HOME)/tools/machines/atari2600/

%.bin: %.asm
	dasm $< -I$(INCLUDE_PATH) -f3 -v5 -o$@ -l$*.txt

run: $(ROM).bin
	stella $<
