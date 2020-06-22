# 2600-Snakes
Simple little snakes game in 6502 assembler for the Atari 2600.

Play the latest stable version using the excellent online emulator:

https://javatari.org/?ROM=https://github.com/RobinSergeant/2600-Snakes/raw/master/snakes.bin

The lack of available RAM (the 2600 only has 128 bytes!) limits the play area to a 16x16 grid
so that the location of each body segment can be represented by 1 byte.  Playfield graphics
are then manipulated on every scanline to draw the body of the snake using an 8 line kernel.
