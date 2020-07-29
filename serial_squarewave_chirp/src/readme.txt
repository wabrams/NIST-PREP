pdm_stereo_ldma

This project demonstrates how to get stereo PCM data from a MEMS microphone
using the PDM interface. An LDMA transfer is triggered when there is valid data
in the PDM's hardware FIFO. The LDMA transfers the raw PCM data from the FIFO to
the ping-pong buffers. The main loop converts the raw PCM data in the ping-pong
buffers into left and right stereo audio PCM data. The device enters EM1 when
when the CPU isn't busy.

How To Test:
1. Build the project and download it to the Thunderboard
2. Open the Simplicity Debugger and add "pingBuffer", "pongBuffer", "left", and
   "right" to the Expressions Window
3. Suspend the debugger; observe the data buffers in the Expressions Window

Peripherals Used:
FSRCO     - 20 MHz
HFRCODPLL - 19 MHz

Board:  Silicon Labs EFR32BG22 Thunderboard (BRD4184A)
Device: EFR32BG22C224F512IM40
PA0 - MIC enable
PC6 - PDM clock
PC7 - PDM data

Note:
In order to change this example to use receive mono audio from a single MEMs
microphone, apply the following changes:
1. Remove/comment out line 70 and 71 about GPIO routing of PDM Data 1
2. Change line 74 from "PDM_CFG0_STEREOMODECH01_CH01ENABLE" to
  "PDM_CFG0_STEREOMODECH01_DISABLE"
3. Remove/comment out line 76 about PDM channel 1 polarity
4. Change line 79 from "PDM_CFG0_NUMCH_TWO" to "PDM_CFG0_NUMCH_ONE"

Note: On SLTB010A BRD4184A Rev A01, the PDM signals are suboptimally routed 
next to the High Frequency crystal which causes HFXO and RF performance issues.