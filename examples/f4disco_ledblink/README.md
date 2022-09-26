Blink a LED on the STM32 F4 Discovery board.

LED #2 (a red LED just next to the reset button) is blinked. The LED to blink
is purposefully selected by a global object with a non-trivial constructor
such that it is proven that the contents of `.init_array` is properly
executed.

Build:
```sh
bazel build //examples/ledblink \
  --platforms=@stm32//platforms:f407g_disc1
```

Load onto the board:

In one terminal launch OpenOCD:

```sh
openocd -f board/stm32f469discovery.cfg
```

In another terminal, start GDB with the built ELF file:

```sh
arm-none-eabi-gdb bazel-bin/examples/f4disco_ledblink/f4disco_ledblink.elf
```

Then connect GDB, flash, and run the binary:
```sh
(gdb) target extended-remote :3333
(gdb) load
(gdb) r  # answer with 'y'
```

Note that the build/load loop can be performed without exiting the debugger;
simply rebuild, leave OpenOCD running, halt the execution in the debugger and
issue a `load` command. This will flash a new binary and halt the execution.
To start the program, issue the `r` command and answer `y` to start execution.
