# Debugging handoff — "ros32bot firmware prints once, then hangs"

**Purpose of this file:** everything a fresh session needs to continue debugging the STM32
firmware hang, *and* a short course in the tools we will use. Written to be self-contained —
no prior chat context required.

---

## 0. TL;DR

- FreeRTOS firmware on an STM32F103C8T6 (BluePill). Encoder reading + velocity was just added.
- **Symptom:** over RTT it prints the banner and **exactly one** encoder line, then nothing.
  The debugger claims the task is stuck at `osDelay(100)` but also says the target is *running*,
  and the Registers view is empty.
- **Hidden second problem:** a build/flash/symbol mismatch is present (disassembly of the target
  does not match `Debug/ros32bot_stm32.elf`). Must be resolved before trusting debugger facts.
- **Prime hypothesis (H1):** `defaultTask`'s stack is only 512 B (≈448 B usable after FreeRTOS's
  64-byte context frame). The new 6-argument `printf` pushes it over; in FreeRTOS the TCB is
  allocated *directly below* its own stack, so the overflow corrupts the task's own list item in
  the heap → first context switch (`osDelay`) dies. There are precise forensic tests for this (§6 Lab B).
- **Constraint:** `configTOTAL_HEAP_SIZE` is only 3072 B with ~564 B free, so **raising a task stack
  requires raising the heap**. `configCHECK_FOR_STACK_OVERFLOW` is off, so overflow is silent.

---

## 1. The system

| Item | Value |
|---|---|
| Board | BluePill, STM32F103C8T6, Cortex-M3, 20 KB SRAM, 64 KB flash |
| Project | `STM32/ros32bot_stm32/` (CubeMX-generated, `ros32bot_stm32.ioc`) |
| RTOS | FreeRTOS via CMSIS-RTOS v2 (`osThreadNew`, `osDelay`, `osDelayUntil`) |
| Heap | `heap_4.c` |
| Toolchain | GNU Tools for STM32 (STM32CubeCLT 1.20.0), `arm-none-eabi-*` on PATH |
| IDE | STM32CubeIDE, launched from a Debug launch config (its own OpenOCD + ST-Link) |
| Prints | SEGGER RTT; `_write()` → `SEGGER_RTT_Write(0, ...)`, `setvbuf(stdout, NULL, _IONBF, 0)` |

### Build (CLI, same build the IDE uses)

```bash
cd STM32/ros32bot_stm32/Debug
make all -j4          # NOTE: bare `make` runs the *clean* target (generated makefile quirk)
```

Regenerated every build (useful artifacts):

| File | What it is |
|---|---|
| `Debug/ros32bot_stm32.elf` | the image |
| `Debug/ros32bot_stm32.list` | full `objdump -h -S` listing — **ground truth for addr↔source↔bytes** |
| `Debug/ros32bot_stm32.map` | linker map (sections, symbols, sizes) |
| `Debug/Core/Src/*.su` | per-function **static stack usage** (`-fstack-usage`) |

### Running it (CubeIDE launch config)

1. Launch the Debug config (it builds, flashes, and **stops at `main`**). Press **Resume (F8)**.
2. OpenOCD ports: GDB `61234`, telnet `62345`, tcl `64567` for that default GDB port — verify with
   `ss -ltnp | grep -i openocd` rather than trusting the formula.
3. Turn RTT on (one telnet session), then read it (second session):

```bash
telnet localhost 62345        # OpenOCD monitor
  rtt setup 0x20000000 0x5000 "SEGGER RTT"
  rtt start
  rtt server start 9090 0
telnet localhost 9090         # the actual RTT stream
```

`_SEGGER_RTT` (RTT control block) currently sits at `0x200000b0`, so
`rtt setup 0x200000b0 0x200 "SEGGER RTT"` also works if the scan is slow.
Keep the debug session alive: ST's `gdb_helper.tcl` calls `shutdown` when the last GDB client
detaches, which kills OpenOCD and any RTT server.

---

## 2. The symptom, verbatim evidence

RTT output (complete):

```
ros32bot: encoder debug over RTT
enc  L pos=     0 vel=     0 dir= 0  |  R pos=     0 vel=     0 dir= 0
```

then silence — no second line.

Facts to read off that:

- The **new** build is running (banner + `pos=/vel=/dir=` format). Good.
- The loop body ran **once**. The loop starts with the print, so it ran at ~tick 0 — *before* any
  100 ms delay. So all-zero counters are **expected** at that instant and are *not* evidence of
  broken encoders. What matters is that the second line (t = +100 ms) never arrives.
- Therefore death happens at/after the **first `osDelay(100)`** → the first context switch out of
  `defaultTask` is the prime suspect window.
- This does **not** yet prove the CPU is hung: an alternative is that the CPU keeps running
  (tick alive, other tasks alive) and only `defaultTask` never gets scheduled again.
- Observed debugger weirdness: "running" but highlighting `osDelay(100)`, step/step-over refused
  ("Cannot execute this command while the target is running"), and an **empty Registers view**
  after Suspend. Treat the IDE as untrusted until re-verified (§5.4).

Disassembly the IDE showed vs. the build on disk (from `Debug/ros32bot_stm32.list`):

| Address | IDE showed | Actual build |
|---|---|---|
| `0x080013dc` | — | `StartDefaultTask` entry: `e92d 43f0 stmdb sp!, {r4-r9, lr}` |
| `0x08001418` | `pop {r7, pc}` | `9302 str r3, [sp, #8]` |
| `0x08001426` | — | `f005 f999 bl 0x800675c <iprintf>` |
| `0x0800142c` | — | `f002 fbdb bl 0x8003be6 <osDelay>` |
| `0x08000dac` | call target labelled `<encoder_get_velocity+22>` | `2b00 cmp r3, #0` (mid-function) |

A `bl` into the middle of a function is impossible in compiled C → either the view was showing
stale/cached bytes, or the flash is not this ELF. **Resolve this first (§6 Lab A.3).**

---

## 3. Static facts already established (no target needed)

### 3.1 Memory / heap

| Symbol | Address | Notes |
|---|---|---|
| `ucHeap` | `0x2000146c` | `configTOTAL_HEAP_SIZE = 3072` (0xC00) |
| `xFreeBytesRemaining` | `0x20002078` | heap_4 static |
| `xMinimumEverFreeBytesRemaining` | `0x2000207c` | low-water mark: best heap diagnostic |
| `pxCurrentTCB` | `0x20000e38` | current task pointer |
| `xTickCount` | `0x20001310` | tick liveness check |
| `uxCurrentNumberOfTasks` | `0x2000130c` | should be 6/7 |
| `defaultTaskHandle` | `0x20000724` | `osThreadId_t` (pointer to TCB) |
| `motorControlHandle` | `0x20000728` | |
| `_SEGGER_RTT` | `0x200000b0` | RTT control block |

Heap arithmetic (heap_4, 8-byte blocks, block header = 8 B, sizes rounded up to 8):

- `ucHeap` is 4-mod-8 aligned, so heap_4 aligns it to `0x20001470` and loses 4 bytes;
  usable region `0x20001470 … 0x20002064` (`pxEnd` at `0x20002064`), first free block = 3060 B.
- Per dynamic task: TCB 92 B → **104 B block**; 128-word stack = 512 B → **520 B block** = **624 B**.
- 4 tasks created in `main.c` order (`defaultTask`, `motorControl`, `microROS`, `gyroTask`)
  = 2496 B → **~564 B left free**. Nothing else takes heap: the Idle and Timer tasks use
  static buffers (`vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory` at
  `0x08003c79` / `0x08003ca9`, provided by the CMSIS-RTOS2 wrapper).

⇒ **Any stack increase requires raising `configTOTAL_HEAP_SIZE`**, or task creation starts failing
(and with the current `configASSERT`, silently at first, then as a dead system).

### 3.2 Predicted task layout (verify in Lab B — predicting first is the point)

| Task | TCB | `pxStack` (stack base) | stack top | 512 B stack ends at |
|---|---|---|---|---|
| defaultTask | `0x20001478` | `0x200014e0` | `0x200016e0` | |
| motorControl | `0x200016e8` | `0x20001750` | `0x20001950` | |
| microROS | `0x20001958` | `0x200019c0` | `0x20001bc0` | |
| gyroTask | `0x20001bc8` | `0x20001c30` | `0x20001e30` | |
| (free remainder) | | `0x20001e30` … `0x20002064` = 564 B | | |

### 3.3 TCB layout (`arm-none-eabi-gdb -batch -ex "ptype /o TCB_t" ros32bot_stm32.elf`)

```
 0  pxTopOfStack          24  xEventListItem      52  pcTaskName[16]     80  uxMutexesHeld
 4  xStateListItem        44  uxPriority          68  uxTCBNumber        84  ulNotifiedValue
                            48  pxStack             72  uxTaskNumber       88  ucNotifyState
                                                                          89  ucStaticallyAllocated
                                                        total size: 92 bytes
```

`xStateListItem` is a `ListItem_t` = `{xItemValue, pxNext, pxPrevious, pvOwner, pvContainer}`
(20 B), so **`pvContainer` = TCB+20**, `pxNext` = TCB+8, `pxPrevious` = TCB+12.

### 3.4 Why a defaultTask stack overflow corrupts FreeRTOS itself

heap_4 allocates the TCB first, then the stack, so the stack's **lowest** addresses sit right on
top of the TCB's **highest** bytes. The stack grows down into its own TCB. Overflow depth *d*
(bytes written below `pxStack`) hits, in order:

| depth d | what gets smashed | visible as |
|---|---|---|
| 1–8 | the stack block's own header at `0x200014d8` | corrupted heap block size |
| 9–12 | TCB padding | – |
| 13–19 | `ucStaticallyAllocated`, `ucNotifyState`, `ulNotifiedValue` | – |
| 23–35 | `uxMutexesHeld`, `uxBasePriority`, `uxTaskNumber`, `uxTCBNumber` | – |
| **36–51** | **`pcTaskName`** | **garbled task name — easiest tell** |
| 55–59 | `pxStack`, `uxPriority` | wrong priority |
| 60–79 | `xEventListItem` | event list corruption |
| **80–99** | **`xStateListItem`** (pvContainer at d=83, pxNext d=95) | **ready-list corruption → hang/crash** |
| 100–103 | `pxTopOfStack` | scheduler loses the task's stack pointer |

So a `printf` that needs ~83 bytes more than the stack has is enough to take the scheduler down
at the very next context switch — exactly the observed "prints once, then stuck at `osDelay`".

### 3.5 Config that makes failures silent (FreeRTOSConfig.h)

| Setting | Current | Consequence |
|---|---|---|
| `configASSERT` | `if ((x)==0) {taskDISABLE_INTERRUPTS(); for(;;);}` | any kernel assert = **dead system with IRQs off** |
| `configCHECK_FOR_STACK_OVERFLOW` | not defined → 0 | **stack overflow is silent** |
| `configUSE_MALLOC_FAILED_HOOK` | not defined → 0 | heap exhaustion is silent (`osThreadNew` just returns NULL, unchecked) |
| `configUSE_TRACE_FACILITY` | 1 | RTOS-aware views can show tasks/stack usage |
| `INCLUDE_uxTaskGetStackHighWaterMark` | 1 | stack usage measurable |
| stack fill | `tskSTACK_FILL_BYTE = 0xA5` | unused stack is `0xA5A5A5A5` → overflow detectable by eye |
| `configRECORD_STACK_HIGH_ADDRESS` | not defined | `pxEndOfStack` is **not** in the TCB (don't look for it) |

---

## 4. Hypotheses, each with its test

| # | Hypothesis | Test | Expected if true |
|---|---|---|---|
| **H1** | `defaultTask` stack overflow during the new 6-arg `printf` → its own TCB/list item corrupted → scheduler dies on first context switch | Read `pcTaskName`, the `0xA5` pattern at `pxStack`, the block header at `0x200014d8`, `xStateListItem` fields (Lab B) | name garbage and/or no `A5A5A5A5` at stack bottom; `pvContainer`/`pxNext` garbage |
| **H2** | HardFault (e.g. in `motorControlFunc` → `encoder_update`, or in HAL) | PC in `HardFault_Handler` (`0x08001b1c`); read fault registers | `CFSR`/`HFSR` non-zero, `BFARVALID`/`MMARVALID` set |
| **H3** | Tick/ISR death (tick stops → everything blocks forever) | Read `xTickCount` twice, 2 s apart; PC in idle/WFI | `xTickCount` frozen; `uxSchedulerSuspended` stuck > 0 |
| **H4** | Debugger/session artifact (target actually fine or different failure; "empty registers" = not stopped) | Attach with standalone OpenOCD (§5.3) and read state; check CubeIDE console for errors | standalone tool halts and reads state that the IDE could not |
| **H5** | Encoders not counting (pos/vel stay 0 while motors spin) | Let it run without printing; check `__HAL_TIM_GET_COUNTER` values, motor power, PA6–PA9 pull-ups/encoder outputs | counters move, or the wiring/pull-up gap is confirmed |

H1 and H2 are not mutually exclusive: an overflow can also walk a garbage pointer into a fault.

---

## 5. Toolbox — what exists and how to use it

### 5.1 Static tools (work without a target — always start here)

```bash
arm-none-eabi-size  Debug/ros32bot_stm32.elf          # flash/RAM footprint
arm-none-eabi-nm -n Debug/ros32bot_stm32.elf          # symbol → address (sorted)
arm-none-eabi-nm -S --size-sort Debug/ros32bot_stm32.elf
arm-none-eabi-objdump -h -S Debug/ros32bot_stm32.elf  # == ros32bot_stm32.list
arm-none-eabi-addr2line -f -e Debug/ros32bot_stm32.elf 0x8003be6   # addr → function:line
arm-none-eabi-gdb -batch -ex "ptype /o TCB_t" Debug/ros32bot_stm32.elf   # struct layout, no target needed
cat Debug/Core/Src/main.su        # per-function static stack usage
grep -n "0x2000" Debug/ros32bot_stm32.map        # memory layout
```

`addr2line` is the workhorse for "the PC is at 0x… — what is it?".

### 5.2 The IDE debug views (Debug perspective)

| View | Use |
|---|---|
| **Debug** | thread/frame tree; the Suspend/Resume/Step toolbar; **Instruction Stepping Mode** toggle (`i→`) |
| **Call Stack** | software call path when halted — read this *first* |
| **Registers** | `R0-R15`, `SP`, `LR`, `PC`, `xPSR`, `PRIMASK`, `BASEPRI`, `CONTROL`. Low 9 bits of `xPSR` = IPSR: 0 thread, 3 HardFault, 11 SVCall, 14 PendSV, 15 SysTick. Empty view = nothing selected / target not stopped |
| **SFRs** | peripheral + core registers from the SVD (look for `SCB` → `CFSR`, `HFSR`, `BFAR`, `MMFAR`). If absent, read the raw addresses in the Memory view |
| **Memory** | add monitors (expressions allowed, e.g. `defaultTaskHandle->pxStack`), 32-bit words, ASCII panel |
| **Expressions** | any C expression: `pxCurrentTCB`, `*defaultTaskHandle`, `defaultTaskHandle->pcTaskName`, `xTickCount`, `xMinimumEverFreeBytesRemaining`. **CDT refuses function calls here** (no `uxTaskGetStackHighWaterMark(...)`) |
| **Disassembly** | instruction-level truth. Only valid while halted |
| **Breakpoints** | hardware breakpoints on `HardFault_Handler`, on `vTaskDelay`, on the stack-overflow hook — catch failures at the scene instead of after the fact |
| Console | build output + GDB transcript (look for flash/download errors here) |
| FreeRTOS views (`FreeRTOS Task List` … if installed) | task names/states/priority/stack usage; requires `configUSE_TRACE_FACILITY=1` (it is on). Look under *Window → Show View → Other…* |

### 5.3 Standalone OpenOCD + GDB (the escape hatch when the IDE misbehaves)

Close the CubeIDE debug session first (only one process may own the ST-Link), then:

```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg
# leave it running; second terminal:
telnet localhost 4444
```

OpenOCD monitor commands worth knowing: `halt`, `resume`, `reset halt`, `reg`, `mdw <addr> [n]`
(read n 32-bit words), `mdh`/`mdb`, `mww`, `bp <addr> 4 hw`, `rbp`, `flash banks`, `rtt …`.

GDB against the same server (`:3333`) gives source-level debugging with full command power:

```bash
arm-none-eabi-gdb Debug/ros32bot_stm32.elf
  target extended-remote :3333
  monitor reset halt
  break HardFault_Handler
  continue
```

Useful GDB commands: `info registers`, `bt`, `p *defaultTaskHandle`, `x/32xw 0x200014e0`,
`ptype /o TCB_t`, `p xFreeBytesRemaining`, `disassemble /r StartDefaultTask`.

One-shot state dump (save as `dump.gdb`, run with `arm-none-eabi-gdb -x dump.gdb`):

```gdb
target extended-remote :3333
monitor halt
printf "PC=%#x  LR=%#x  SP=%#x  xPSR=%#x\n", $pc, $lr, $sp, $xpsr
bt
p *defaultTaskHandle
p xTickCount
p xFreeBytesRemaining
p xMinimumEverFreeBytesRemaining
x/8xw 0x20001478
x/12xw 0x200014d8
x/4xw 0xE000ED28
detach
```

### 5.4 Fault registers (always available, no SVD needed)

| Address | Register | Meaning |
|---|---|---|
| `0xE000ED24` | `SHCSR` | enable/active fault status |
| `0xE000ED28` | `CFSR` | configurable fault status — the important one |
| `0xE000ED2C` | `HFSR` | bit 30 `FORCED` = escalated to HardFault; bit 1 `VECTTBL` = bad vector fetch |
| `0xE000ED34` | `MMFAR` | faulting data address (valid if `CFSR.MMARVALID`) |
| `0xE000ED38` | `BFAR` | faulting instruction address (valid if `CFSR.BFARVALID`) |

`CFSR` byte 1 (`0xE000ED29`) is the bus-fault byte: `PRECISERR` (bit 1) = bad data access,
`IMPRECISERR` (bit 2) = write buffered, address unknown, `IBUSERR` (bit 0) = bad instruction fetch.
`CFSR` byte 3 (`0xE000ED2B`): `UNDEFINSTR`, `INVSTATE`, `NOCP`.

### 5.5 FreeRTOS forensics

- `pxCurrentTCB->pcTaskName` → who is *really* running while the IDE highlights something else.
- `xTickCount` read twice → is the tick alive? (0x20001310)
- `xFreeBytesRemaining` / `xMinimumEverFreeBytesRemaining` → heap headroom and low-water mark.
- Stack usage by eye: unused stack is `0xA5A5A5A5`. Used bytes ≈ 512 − (words of `A5` from
  `pxStack` upward). Compare with `.su` static frames for cross-checking.
- Never call `uxTaskGetStackHighWaterMark()` from the Expressions view (CDT blocks calls) —
  read it via standalone GDB (`p uxTaskGetStackHighWaterMark(defaultTaskHandle)`) or add a print
  in code.

### 5.6 RTT

See §1. RTT writes are non-blocking (`NO_BLOCK_SKIP` default), and `SEGGER_RTT_Conf.h` defines
**no** lock macros, so RTT itself cannot block the CPU — if the system hangs, it is not because
"RTT was busy".

---

## 6. The plan

Rule for the whole session: **halt before believing anything**, **change one thing at a time**,
**write down the prediction before measuring**.

### Lab A — reproduce and capture the truth

1. Build (`make all -j4`), launch the CubeIDE debug config, open the RTT window, **F8**.
2. Confirm the symptom: exactly one `enc` line.
3. **Prove flash == build** (one of):
   - Memory view → `0x08007660` → ASCII panel should read `enc  L pos=%6ld vel=…` (the format
     string literal referenced from `StartDefaultTask+0x58`). If it differs → rebuild, re-flash,
     and re-verify before going further.
   - Disassembly at `0x080013dc` must be `stmdb sp!, {r4-r9, lr}` and at `0x0800142c`
     `bl 0x8003be6 <osDelay>` (compare with `Debug/ros32bot_stm32.list`).
   - Console must show a real reprogram (`** Programming Finished **` / `Download verified`).
4. Let it hang, then **Suspend**. Read and record:
   - `PC`, `LR`, `SP`, `xPSR` (IPSR), `PRIMASK`
   - Call Stack (all frames)
   - `pxCurrentTCB` and `pxCurrentTCB->pcTaskName`
   - `xTickCount` → Resume 2 s → Suspend → `xTickCount` again
   - fault registers `0xE000ED28 0xE000ED2C 0xE000ED34 0xE000ED38`
   - `xFreeBytesRemaining`, `xMinimumEverFreeBytesRemaining`

Decision tree from Lab A:

```
PC in HardFault_Handler (0x08001b1c)  -> H2: read CFSR/HFSR/BFAR, find the faulting instruction
PC inside tasks.c/list.c for(;;)      -> a configASSERT fired: addr2line the PC to get the line
PC inside vTaskDelay / list code loop -> H1/H3: look at the TCB and heap (Lab B)
PC in idle task / WFI                 -> check xTickCount twice (H3)
PC somewhere unrelated in a loop      -> read the loop's function via addr2line; check it's not
                                         an assert/stub; then Lab B
Cannot halt / empty registers         -> debugger session problem: restart the session, or use
                                         standalone OpenOCD (§5.3) and re-run Lab A step 4
```

### Lab B — stack/heap forensics (decides H1 in minutes)

With the target halted right after the hang (do **not** power-cycle or reset first — SRAM keeps
the evidence):

1. `defaultTaskHandle->pcTaskName` — still `"defaultTask"`? If it is garbage, the task's stack has
   already overflowed into its TCB (depth ≥ 36, §3.4).
2. `defaultTaskHandle->uxPriority` (expect 8), `->pxStack` (expect `0x200014e0`), `->pxTopOfStack`.
3. Memory at `defaultTaskHandle->pxStack`, 32-bit words: the first 8 words should still be
   `0xA5A5A5A5`. Any non-`A5` word in the *lowest* words = the stack touched its bottom.
4. Memory around `0x200014d8`: the stack block header (`0x200014d8`) and the TCB tail
   (`0x200014d0`-ish) should be intact; a stack value like a string address or `0x20001…` here
   proves overflow.
5. `defaultTaskHandle->xStateListItem` (`pvContainer` at `0x20001494`): should point to a ready
   list node inside `pxReadyTasksLists` (`0x20000e3c` + 20·priority…), not garbage.
6. Cross-check `xFreeBytesRemaining` (~564 predicted) and `uxCurrentNumberOfTasks` (expect 6 or 7:
   idle + timer + 4 user tasks; if a task failed to be created you will see fewer, and with a
   corrupted heap you may see nonsense).
7. Compare measured stack use with `Debug/Core/Src/main.su` (`StartDefaultTask` static frame was
   56 B before the new print — the newlib `iprintf` path is added on top and is *not* in `.su`).
8. Also predict-then-verify the layout table in §3.2 (TCBs/stacks) — it teaches how the heap
   actually laid things out and whether GC/alignment assumptions hold.

### Lab C — instrumentation (make the next failure loud)

Cheap, high-value, and the basis for the eventual fix.

`Core/Inc/FreeRTOSConfig.h`, in `/* USER CODE BEGIN Defines */` (survives CubeMX regeneration):

```c
/* --- keep these overrides out of CubeMX's reach --- */
#undef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE            ((size_t)8192)   /* was 3072 */
#define configCHECK_FOR_STACK_OVERFLOW   2
#define configUSE_MALLOC_FAILED_HOOK     1
```

Make `configASSERT` report instead of silently spinning (it lives in `USER CODE BEGIN 1`):

```c
#define configASSERT( x ) if ((x) == 0) {                          \
    __disable_irq();                                               \
    SEGGER_RTT_printf(0, "\n!! ASSERT %s:%d\n", __FILE__, __LINE__);\
    for( ;; ); }
```

`Core/Src/main.c`, in `USER CODE BEGIN 4`:

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  __disable_irq();
  SEGGER_RTT_printf(0, "\n!! STACK OVERFLOW in '%s'\n", pcTaskName);
  for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
  __disable_irq();
  SEGGER_RTT_printf(0, "\n!! pvPortMalloc failed, free=%u\n",
                    (unsigned)xPortGetFreeHeapSize());
  for (;;) { }
}
```

Add a build tag so flash/build mismatches are impossible to miss:

```c
SEGGER_RTT_printf(0, "ros32bot " __DATE__ " " __TIME__ "\n");
```

And to *measure* stack headroom (do this once things run):
`uxTaskGetStackHighWaterMark(NULL)` → free words; needed bytes = (stack words − free) × 4.

Two quick bisect experiments (each needs a rebuild + flash; do them one at a time):

- **E1:** replace the 6-arg `printf` with `SEGGER_RTT_printf(0, "enc L %d %d %d | R %d %d %d\n", …)`.
  SEGGER's own formatter uses almost no stack and no newlib. If the hang disappears → H1 confirmed
  (newlib `printf` stack appetite), and you have a cheap workaround while fixing properly.
- **E2:** keep `printf` but delete the `encoder_update(period)` call (or the `motor_set_duty`
  calls) to test whether the death is instead in `motorControlFunc` (H2).

### Lab D — fix and verify

- Raise `defaultTask` stack 128 → 256 words (`osThreadAttr_t.stack_size = 256 * 4`) **and** the
  heap (Lab C) — otherwise task creation fails. `motorControl` should get 256 words too, since
  PID will live there.
- Keep `configCHECK_FOR_STACK_OVERFLOW = 2` and the hooks permanently.
- Verify: boot, print for 10+ minutes, report high-water marks for both tasks, confirm
  `xMinimumEverFreeBytesRemaining` has healthy margin.

---

## 7. Address book (this build; regenerate with `nm` after every build)

| Symbol | Address |
|---|---|
| `_SEGGER_RTT` | `0x200000b0` |
| `defaultTaskHandle` / `motorControlHandle` | `0x20000724` / `0x20000728` |
| `pxCurrentTCB` / `pxReadyTasksLists` / `uxTopReadyPriority` | `0x20000e38` / `0x20000e3c` / `0x20001314` |
| `xTickCount` / `uxCurrentNumberOfTasks` / `xSchedulerRunning` | `0x20001310` / `0x2000130c` / `0x20001318` |
| `xIdleTaskHandle` / `xTimerTaskHandle` | `0x20001330` / `0x2000136c` |
| `xFreeBytesRemaining` / `xMinimumEverFreeBytesRemaining` | `0x20002078` / `0x2000207c` |
| `ucHeap` (3072 B) | `0x2000146c` |
| `_write` / `StartDefaultTask` / `motorControlFunc` | `0x080013b8` / `0x080013dc` / `0x08001438` |
| `HardFault_Handler` / `SysTick_Handler` | `0x08001b1c` / `0x08001b48` |
| `SVC_Handler` / `PendSV_Handler` | `0x08005ed0` / `0x080060b0` |
| `encoder_get` / `_position` / `_velocity` / `_dir` | `0x08000bf0` / `0x08000d6c` / `0x08000d96` / `0x08000dc0` |
| `encoder_get_count` | **not in the image** (`--gc-sections` dropped it: nothing calls it) |
| `osDelay` / `vTaskDelay` / `vTaskDelayUntil` | `0x08003be6` / `0x08004c78` / `0x08004b78` |
| `iprintf` / `_vfiprintf_r` | `0x0800675c` / `0x08006cd0` |
| format string `"enc  L pos=%6ld …"` | `0x08007660` |
| `vApplicationGetIdleTaskMemory` / `…TimerTaskMemory` | `0x08003c79` / `0x08003ca9` |

Image size: text 30708, data 148, bss 10052 (of 20 KB SRAM).

---

## 8. Habits worth keeping

1. **Halt first.** Nothing about a running target is knowable; step/step-over/registers/disassembly
   are only valid when stopped. "Running" + a source highlight tells you nothing.
2. **Prove flash == build** (format-string probe, banner with `__DATE__/__TIME__`, or compare bytes
   with `ros32bot_stm32.list`) before drawing any conclusion.
3. **Predict, then measure.** Write the expected value down; a mismatch is information.
4. **Make failures loud**: assert hooks, malloc-fail hook, stack-overflow hook, `__DATE__` banner.
5. **Breakpoint the stubs**: `HardFault_Handler`, `Error_Handler`, and your hooks. Catch the scene,
   not the aftermath.
6. **Don't reset or power-cycle** before dumping RAM when hunting memory corruption — the evidence
   lives in SRAM.
7. **One variable per experiment**, and rebuild from the CLI (`make all -j4`) so the image you
   reason about is the image in `Debug/`.
8. When the IDE's story stops adding up, **switch tools** (standalone OpenOCD, `addr2line`,
   `objdump`) rather than clicking harder.
