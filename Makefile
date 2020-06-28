ROM ?= snakes
INCLUDE_PATH ?= $(HOME)/tools/machines/atari2600/

%.bin: %.asm
	dasm $< -I$(INCLUDE_PATH) -f3 -o$@ -l$*.lst -s$*.sym

run: $(ROM).bin
	stella $<
