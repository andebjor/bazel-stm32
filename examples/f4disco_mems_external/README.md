Read an external MEMS unit connected over i2c to the STM32 F4 Discovery board.

Build:
```sh
bazel build //examples/f4disco_mems_external \
  --platforms=@stm32//platforms:f407g_disc1
```

Load onto the board:

In one terminal launch OpenOCD:

```sh
openocd -f board/stm32f469discovery.cfg
```

In another terminal, start GDB with the built ELF file:

```sh
arm-none-eabi-gdb bazel-bin/examples/f4disco_mems_external/f4disco_mems_external.elf
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
