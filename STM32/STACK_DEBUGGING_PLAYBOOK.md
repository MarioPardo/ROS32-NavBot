# STACK DEBUGGING PLAYBOOK

Companion to `DEBUGGING_HANDOFF.md`. Written after the FreeRTOS stack-overflow bug
found on 2026-09-27, where `defaultTask`'s 512-byte stack overflowed into heap_4's
block header and then into the timer queue's item size, causing a runaway `memcpy`
and a HardFault.

All numbers in this document are from this project. Re-verify addresses after every
rebuild — see §7.

---

## 1. Where a task's stack lives

A task stack is not a named variable. It is a block of RAM carved out of the heap,
and the TCB holds a pointer to its lowest byte.

### 1.1 The ELF, at build time (no target needed)

```bash
arm-none-eabi-nm -nS Debug/ros32bot_stm32.elf | grep -i timer
```

```
2000099c 0000005c b Timer_TCB.1
200009f8 00000400 b Timer_Stack.0
```

Reading the output:

| Flag / column | Meaning |
|---|---|
| `-n` | sort by address, so the output reads as a memory map |
| `-S` | print the size |
| size | **hex bytes** — `0x400` = 1024 |
| `b` / `B` | `.bss`; lowercase = `static` (file-local), uppercase = global |
| `t` / `T` | `.text` (code) |

This only works for **statically allocated** stacks. The timer task's stack is in
`.bss` because FreeRTOS creates it through `vApplicationGetTimerTaskMemory`. The
four `osThreadNew` tasks live in the heap and have **no symbol at all**.

### 1.2 The TCB, on the running target

`osThreadId_t` is `void *`, so a cast is required. In the Expressions view:

```
((TCB_t*)defaultTaskHandle)->pcTaskName
((TCB_t*)defaultTaskHandle)->pxStack
((TCB_t*)defaultTaskHandle)->uxPriority
```

`pxStack` is the **bottom** (lowest address) of the stack.

Always confirm with `pcTaskName` before trusting an address. Casting the wrong
handle yields plausible-looking garbage.

### 1.3 Let the debugger tell you

A stopped CPU shows stack pointers resolved to symbols:

```
psp  0x20000d28 <Timer_Stack.0+816>
```

That single label identified the faulting context immediately. Read the IDE's
label before doing any arithmetic.

---

## 2. How big is the stack

| Situation | How to get the size |
|---|---|
| Static stack | `nm -S` gives it directly |
| Heap stack | derive it, or read what you asked for |

Heap stacks have no symbol. FreeRTOS allocates them in creation order, so
consecutive `pxStack` values differ by *(previous block + 8-byte heap header)*:

```
0x20001680 - 0x20001478 = 0x208 = 520 = 512 + 8
```

That confirmed `defaultTask` had 512 bytes. The authoritative answer is always the
value written in `.stack_size`, which for CMSIS-RTOS2 is in **bytes**.

---

## 3. Has it changed? Has it overflowed?

### 3.1 The 0xA5 fill pattern

FreeRTOS writes `0xA5` into every byte of a new task's stack
(`tskSTACK_FILL_BYTE`, `tasks.c:76`). An untouched byte is `0xA5A5A5A5`. Anything
else means the stack has been there.

Procedure:

1. Read `pxStack` (lowest address) and the size, then compute `top = pxStack + size`.
2. In the Memory view, jump to `pxStack`.
3. Read **upward**. Untouched `A5A5A5A5` comes first, then a boundary, then live data.
4. `peak_used = top - (address of the first non-A5 word)`

Direction is the most common mistake. The stack grows **down**, so the unused
region is at the bottom (low addresses) and the used region is at the top.

### 3.2 Reading the results

| Observation | Meaning |
|---|---|
| `A5A5A5A5` (renders as `¥¥¥¥`) | never touched — the stack has never been that deep |
| anything else | live data |
| non-`A5` bytes **below `pxStack`** | **overflow** — you are inside heap_4's block header |
| `0x0800xxxx` in `A5` territory | a spilled return address; `addr2line` it to learn what ran |
| red text in the Memory view | CDT marks values changed since the last read |
| byte order | the view shows **memory order**: `08020080` means `0x80000208` |

**Byte order trap.** The Eclipse CDT Memory view renders bytes in memory order, not
as values. Reverse every 4-byte group by hand.

### 3.3 The API version

`uxTaskGetStackHighWaterMark(handle)` returns minimum-ever free space **in words**.
The Expressions view cannot call functions, so use `configUSE_TRACE_FACILITY`
(already `1` here) with `uxTaskGetSystemState()`. It fills a `TaskStatus_t[]` with
`usStackHighWaterMark` for every task at once — the best single answer to "which
task is about to die".

### 3.4 Worked example from this project

```
pxStack             0x20001478
top                 0x20001678   (pxStack + 512)
first non-A5        0x20001470   <- BELOW pxStack
peak_used           0x20001678 - 0x20001470 = 0x208 = 520
available           512
```

520 > 512, and the 8 excess bytes landed exactly on the heap block header at
`0x20001470`. That was the whole bug.

---

## 4. Make failures loud

This bug was completely silent. Apply these permanently:

```c
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configRECORD_STACK_HIGH_ADDRESS 1
```

`vApplicationStackOverflowHook()` and `vApplicationMallocFailedHook()` must also be
**defined**, or the project fails to link — FreeRTOS declares them `extern`. Both
hooks must print over **RTT, never `printf`**, because `printf` is what eats the
stack.

| Option | Effect |
|---|---|
| `configCHECK_FOR_STACK_OVERFLOW 2` | checks the lowest 20 bytes of every task stack on each context switch; would have caught this bug on the first overflow |
| `configRECORD_STACK_HIGH_ADDRESS 1` | adds `pxEndOfStack` to the TCB, so `pxEndOfStack - pxStack` gives the size; costs 4 bytes of heap per task |
| `configUSE_MALLOC_FAILED_HOOK 1` | catches `pvPortMalloc` returning `NULL`, which is what happens when a stack is raised without raising the heap |

Also worth adding: a `__DATE__ __TIME__` banner over RTT, so you always know which
build is on the chip.

### 4.1 Known-good starting configuration

| Option | Value now | Note |
|---|---|---|
| `configUSE_TRACE_FACILITY` | `1` | enables `uxTaskGetSystemState` |
| `INCLUDE_uxTaskGetStackHighWaterMark` | `1` | enabled |
| `configSUPPORT_STATIC_ALLOCATION` | `1` | makes the timer task use `.bss`, outside the heap |
| `configUSE_TIMERS` | `1` | creates the timer service task |
| `configTIMER_TASK_STACK_DEPTH` | `256` | **words** → 1024 bytes |
| `configTIMER_QUEUE_LENGTH` | `10` | items are 16 bytes → 160-byte storage |
| `configCHECK_FOR_STACK_OVERFLOW` | **not defined** | add it |
| `configUSE_MALLOC_FAILED_HOOK` | **not defined** | add it |
| `configRECORD_STACK_HIGH_ADDRESS` | **not defined** | add it |

---

## 5. Predicting the stack size you need

### 5.1 Units — this is where hours are lost

| Thing | Unit |
|---|---|
| `osThreadAttr_t.stack_size` (CMSIS-RTOS2) | **bytes** |
| native `xTaskCreate` `usStackDepth` | **words** |
| `configMINIMAL_STACK_SIZE` | **words** |
| `configTIMER_TASK_STACK_DEPTH` | **words** |
| `configTOTAL_HEAP_SIZE` | **bytes** |
| `uxTaskGetStackHighWaterMark` return | **words** |
| `.su` file numbers | **bytes** |

### 5.2 Static estimate from `.su` files

`-fstack-usage` is already enabled in this project.

```bash
grep -n "StartDefaultTask" Debug/Core/Src/main.su
```

```
../Core/Src/main.c:586:6:StartDefaultTask	56	static
```

Format is `file:line:col:function`, then bytes, then a qualifier:
`static` = fixed frame, `dynamic` = varies at runtime, `bounded` = proven maximum.

**56 bytes is misleading on its own.** What matters is the deepest chain, not one
frame: `56 (task frame) + 64 (context frame) + ~456 (printf chain) ≈ 576` against
512 available.

### 5.3 Account for library calls

`printf` with `%6ld` pulls in newlib's entire formatting engine:
`_vfprintf_r` → `__ssputs_r` → `_malloc_r` (**printf can malloc**) → `_write_r` →
the retarget → `SEGGER_RTT_Write`. That chain alone measured **~456 bytes**.

Cheaper alternatives, ascending effort:

1. `SEGGER_RTT_printf` — skips newlib entirely.
2. `snprintf` into a small buffer, then a single RTT write. Still pulls most of the engine.
3. A hand-rolled fixed formatter for the encoder line — cheapest, and the line is only `%6ld %6ld %2d` twice.

### 5.4 Measure, do not estimate

1. Enable the three options from §4.
2. Exercise the **worst-case** path — largest values and fault branches, not the happy path.
3. Halt at the end of the loop and read the `A5` boundary, or print
   `uxTaskGetStackHighWaterMark(h) * 4` over RTT.
4. Required size = `peak_used × 1.5 to 2`. For `defaultTask`, 520 → **1024** was right.

### 5.5 Budget the heap and the chip

```
configTOTAL_HEAP_SIZE                      8192
  4 task stacks  1024 + 512 + 512 + 512     2560
  4 TCB blocks   4 x (92 + 8, padded)        416
  ------------------------------------------------
  used                                     ~2976
  free                                     ~5200

timer task: 1024 stack + 92 TCB   ->  .bss, NOT the heap
```

Chip-level check, easy to forget:

```bash
arm-none-eabi-size Debug/ros32bot_stm32.elf
```

```
   text    data     bss     dec
  30708     148   15172   46028

RAM used = data + bss = 15320 of 20480  ->  ~5 KB left for the MSP
```

The main stack grows **down from `_estack = 0x20005000`** and must never meet
`ucHeap + configTOTAL_HEAP_SIZE`. After any heap increase, re-run `size` and confirm
that gap. Raising a task stack by 512 bytes costs 512 bytes of heap **and** 512
bytes of the 20 KB of RAM.

---

## 6. The HardFault procedure

### 6.1 What the hardware does before your handler runs

1. It pushes **8 words onto whatever stack was in use at that instant** — R0, R1,
   R2, R3, R12, LR, PC, xPSR, in that order going to increasing addresses. After
   the push the stack pointer points at R0.
2. It loads the handler address from the vector table and jumps there.
3. It sets `LR` to **EXC_RETURN**, recording which stack the frame went on:
   `0xFFFFFFF9` = MSP, `0xFFFFFFFD` = PSP, `0xFFFFFFF1` = Handler mode / MSP.

The trap: handler mode **always** uses MSP. So if the fault happened in a task, the
frame is on PSP while the `SP` the debugger shows you is MSP. Reading the frame at
`SP` then reads unrelated memory. **`LR` is the first thing to read, not `SP`.**

### 6.2 Steps

1. **Do not reset.** The evidence is in RAM.
2. Registers view, read `LR`:
   - `0xFFFFFFFD` → frame is on **PSP**
   - `0xFFFFFFF9` → frame is on **MSP + 4** (the handler's own `push {r7}` sits above it)
3. Read the 8-word frame there, **reversing every 4-byte group**:

   | Offset | Register |
   |---|---|
   | `+0x00` `+0x04` `+0x08` `+0x0C` | R0 R1 R2 R3 |
   | `+0x10` | R12 |
   | `+0x14` | LR — return address into the caller |
   | `+0x18` | **PC — the faulting instruction** |
   | `+0x1C` | xPSR (T bit at bit 24 means Thumb) |

4. Resolve the PC:

   ```bash
   arm-none-eabi-addr2line -f -e Debug/ros32bot_stm32.elf 0x08006a74
   ```

5. Fault status registers, via Expressions. These are **hardware-absolute** and
   never move between builds, unlike your symbols:

   ```
   *(volatile unsigned int*)0xE000ED28   // CFSR
   *(volatile unsigned int*)0xE000ED2C   // HFSR
   *(volatile unsigned int*)0xE000ED34   // BFAR
   *(volatile unsigned int*)0xE000ED24   // SHCSR
   *(volatile unsigned int*)0xE000ED04   // ICSR
   ```

   | Field | Meaning |
   |---|---|
   | HFSR bit 30 FORCED | the real cause is in CFSR |
   | CFSR bit 9 PRECISERR | a load/store hit a bad address |
   | CFSR bit 10 IMPRECISERR | needs a `DSB` to pin down |
   | CFSR bit 13 STKERR | stacking failed — the stack pointer was already out of bounds |
   | CFSR bit 15 BFARVALID | BFAR holds the offending address |

   MMFAR and BFAR are only valid when their VALID bits are set. Ignore them after a
   HardFault with no fault recorded.

6. **Reverse-engineer arguments from the frozen registers.** This cracked the bug.
   The `memcpy` loop left `R3 = dst - 1 + k` and `R1 = src + k`, so:

   ```
   k         = R3 - (R0 - 1) = 0x2000FA2F - 0x20000DA3 = 0xEC8C
   src_start = R1 - k        = 0x20010000 - 0xEC8C     = 0x20001374
   len       = R2 - src_start = 0x28001663 - 0x20001374 = 0x080002EF
   ```

   `0x20001374` is exactly `ucStaticTimerQueueStorage.1`. Arithmetic landing on a
   symbol is how you know the reconstruction is right.

### 6.3 Related traps in this project

- `HardFault_Handler` is `push {r7}` then `while(1)`. The `SP` you see is not the
  frame.
- `configASSERT` is defined as `taskDISABLE_INTERRUPTS(); for(;;);` — a hang with
  interrupts off, which looks different from a HardFault but is equally fatal.
- `SHCSR` is never written, so MemManage, BusFault and UsageFault all escalate to
  HardFault. You must consult CFSR to learn which one fired.

---

## 7. Address book discipline

Symbol addresses are a property of a **build**, not of a project. Change a buffer
size, add a global, and the map shifts.

In this project, growing the heap from 3072 to 8192 moved `xFreeBytesRemaining`
from `0x20002078` to `0x20003478` — and that move was itself the proof the change
had taken effect.

Regenerate after every build:

```bash
arm-none-eabi-nm -nS Debug/ros32bot_stm32.elf
```

### Current values (build of 2026-09-27 18:58, heap 8192, `defaultTask` 1024)

| Symbol | Address |
|---|---|
| `ucHeap` | `0x2000146C` |
| `xFreeBytesRemaining` | `0x20003478` |
| `xMinimumEverFreeBytesRemaining` | `0x2000347C` |
| `pxCurrentTCB` | `0x20000E38` |
| `pxReadyTasksLists` | `0x20000E3C` |
| `uxCurrentNumberOfTasks` | `0x2000130C` |
| `Timer_Stack.0` | `0x200009F8` (size `0x400` = 1024) |
| `Timer_TCB.1` | `0x2000099C` (size `0x5C` = 92) |
| `ucStaticTimerQueueStorage.1` | `0x20001374` (size `0xA0` = 160) |
| `xStaticTimerQueue.0` | `0x20001414` (size `0x50` = 80) |
| `defaultTaskHandle` | `0x20000724` |
| `motorControlHandle` | `0x20000728` |
| `StartDefaultTask` | `0x080013DC` |
| `HardFault_Handler` | `0x08001B1C` |
| `pvPortMalloc` | `0x08006200` |
| `vPortFree` | `0x0800639C` |

### Struct layouts (this build)

`TCB_t` = **92 bytes**, no `pxEndOfStack` while `configRECORD_STACK_HIGH_ADDRESS`
is undefined.

| Offset | Field |
|---|---|
| 0 | `pxTopOfStack` |
| 4 | `xStateListItem` |
| 24 | `xEventListItem` |
| 44 | `uxPriority` |
| 48 | `pxStack` |
| 52 | `pcTaskName[16]` |
| 68 | `uxTCBNumber` |
| 72 | `uxTaskNumber` |
| 76 | `uxBasePriority` |
| 80 | `uxMutexesHeld` |
| 84 | `ulNotifiedValue` |
| 88 | `ucNotifyState` |
| 89 | `ucStaticallyAllocated` |

`Queue_t` = **80 bytes**.

| Offset | Field |
|---|---|
| 0 | `pcHead` |
| 4 | `pcWriteTo` |
| 8 | `u.xQueue.pcTail` |
| 12 | `u.xQueue.pcReadFrom` |
| 16 | `xTasksWaitingToSend` |
| 36 | `xTasksWaitingToReceive` |
| 56 | `uxMessagesWaiting` |
| 60 | `uxLength` |
| **64** | **`uxItemSize`** |
| 68 | `cRxLock` |
| 69 | `cTxLock` |
| 70 | `ucStaticallyAllocated` |
| 72 | `uxQueueNumber` |
| 76 | `ucQueueType` |

Regenerate either with:

```bash
arm-none-eabi-gdb -batch -ex "ptype /o TCB_t" Debug/ros32bot_stm32.elf
```

---

## 8. Useful commands

```bash
# address -> file:line
arm-none-eabi-addr2line -f -C -e Debug/ros32bot_stm32.elf 0x08006a74

# symbol table sorted by address, with sizes
arm-none-eabi-nm -nS Debug/ros32bot_stm32.elf

# memory and flash budget
arm-none-eabi-size Debug/ros32bot_stm32.elf

# struct layout with offsets
arm-none-eabi-gdb -batch -ex "ptype /o TCB_t" Debug/ros32bot_stm32.elf

# raw bytes at an address (flash or RAM image)
arm-none-eabi-objdump -s --start-address=0x08000000 --stop-address=0x08000010 Debug/ros32bot_stm32.elf

# disassemble a range
arm-none-eabi-objdump -d --start-address=0x08001b1c --stop-address=0x08001b30 Debug/ros32bot_stm32.elf

# per-function stack frames
grep -n FunctionName Debug/Core/Src/file.su

# rebuild
cd Debug && make all -j4
```

---

## 9. RTT setup

`_SEGGER_RTT` lives in `.bss` and its magic string is written lazily by `_DoInit()`
on the first RTT call. A `rtt setup` before `SEGGER_RTT_Init()` has run will report
"Cannot find control block".

Set the breakpoint **after** `SEGGER_RTT_Init()` in `main.c`, then:

```
telnet localhost 62345
rtt setup 0x20000000 0x5000 "SEGGER RTT"
rtt start
rtt server start 9090 0
```

Read the output on `telnet localhost 9090`.

Standalone OpenOCD cannot run while CubeIDE holds the ST-Link, so use CubeIDE's
own OpenOCD and its GDB server port.

---

## 10. The debugging loop

The method that found this bug, stated once:

1. **State a hypothesis** in one sentence.
2. **Make a falsifiable prediction** — a specific value, address, or register.
3. **Verify the instrument** before trusting it. Confirm flash equals build, confirm
   the byte order, confirm the symbol is what you think it is.
4. **Halt at the moment that preserves evidence.** Do not reset, do not power-cycle.
5. **Measure.**
6. **A mismatch is information, not failure.** Two predictions were wrong in this
   session and both times the mismatch redirected the search.

Corollaries learned the hard way:

- Read the IDE's symbol labels before computing addresses by hand.
- Prefer the Expressions view with explicit casts over hand-computed offsets.
- One variable per experiment.
- Regenerate the address book after every rebuild.
