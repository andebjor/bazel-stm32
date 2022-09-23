Blink a LED on the STM32 F4 Discovery board.

*Note* This example does not blink a LED as of this commit, but stalls in an
empty `main`.

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
(gdb) r
```
