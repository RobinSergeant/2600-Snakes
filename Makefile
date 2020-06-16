ROM ?= snakes

%.bin: %.asm
	/Users/rzzxf5/tools/dasm $< -I/Users/rzzxf5/tools/machines/atari2600/ -f3 -v5 -o$@ -l$*.txt

run: $(ROM).bin
	/Users/rzzxf5/tools/stella.sh $<
