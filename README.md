# Astronet-ASIC-v9.1

# AstroNet ASIC v9.1 – README  
**Fully Patched, Radiation-Hardened, Neuromorphic Space SoC**  
*Single-file Verilog implementation (module + sub-modules + testbench + formal verification + fault injection)*  

---

## 1. Overview  

**AstroNet ASIC** is a compact, event-driven neuromorphic System-on-Chip (SoC) designed for autonomous space instrumentation. It fuses:  

1. **Sparse Receptive-Field CNN** – for spatial pattern classification  
2. **Event-Driven RNN** – for temporal anomaly detection  
3. **AER (Address-Event Representation) spike routing** – with TMR-protected FIFO  
4. **Radiation-hardened memory subsystem** – 32 KiB SRAM with SECDED ECC + background scrubbing  
5. **TMR state machine & critical registers** – for SEU mitigation  
6. **Clock gating & power modes** – ultra-low idle power  

All functionality is verified with **formal properties**, **functional simulation**, and **fault injection** in a single self-contained Verilog file.

---

## 2. Key Features  

| Feature | Implementation |
|-------|----------------|
| **Neuromorphic Core** | LIF neurons, decay-based integration, spike-triggered updates |
| **CNN** | 4×4 input, 3×3 kernels, 4 filters, sparse RF via precomputed bitmasks |
| **RNN** | 32 units, 16-channel input, rate-coded anomaly scoring |
| **AER Queue** | 64-entry circular buffer, TMR on head/tail pointers |
| **Memory** | 32K × 21-bit (16 data + 5 ECC), write pipeline, background scrubber |
| **ECC** | SECDED (22,16) Hsiao code, single-bit correction, double-bit detection |
| **Fault Tolerance** | Register TMR, ECC, scrubbing, fault status output |
| **Power Management** | Clock gating (ICG), `power_low` idle mode |
| **DFT** | 8-stage scan chain for observability |
| **Fault Injection** | `inject_seu` port for SEU emulation |
| **Formal Verification** | SVA properties: no AER overflow, valid commands, fault recovery |

---

## 3. Top-Level Interface  

```verilog
module astronet_asic (
    input  wire        clk, rst_n,
    input  wire [15:0] sensor_data [0:15],
    input  wire        sensor_valid,
    output reg  [7:0]  command_out,
    output reg         command_valid,
    output reg  [2:0]  fault_code,
    output reg         power_low,
    input  wire        scan_enable, scan_in,
    output wire        scan_out,
    // Debug / Fault Injection
    input  wire        inject_seu,
    input  wire [14:0] seu_addr,
    input  wire [15:0] seu_data
);
```

### Command Encoding  
| `command_out` | Meaning |
|---------------|--------|
| `8'h00` | Normal / No Action |
| `8'h01` | CNN Confidence High → Trigger Action |
| `8'h02` | RNN Anomaly Detected |
| `8'hFF` | Fault State (recovery mode) |

### Fault Codes  
| `fault_code` | Source |
|-------------|--------|
| `3'b001` | CNN fault |
| `3'b010` | RNN fault |
| `3'b011` | SRAM double-bit error |
| `3'b100` | Neuromorphic accelerator fault |
| `3'b000` | No fault |

---

## 4. Architecture Breakdown  

```
sensor_data[] → AER Encoder → Spike Queue (TMR head/tail) → NM Accelerator
        ↓
   SRAM (ECC + Scrub) ←→ State Machine (TMR)
        ↓
   CNN (sparse RF) ←──┐
   RNN (event-driven) ←┤→ Spike Bus → Consumers
                       ↓
                    Decision Logic → command_out
```

### 4.1 AER Encoder  
- Converts 4×4×16 sensor frame into timestamped `(ts_mod, y, x)` events  
- Prevents overflow via `aer_full` backpressure  
- TMR on `aer_head` and `aer_tail`

### 4.2 Neuromorphic Accelerator  
- Dequeues AER events when consumer (CNN or RNN) is ready  
- Flow control: `advance_head` only if `consumer_ready`  
- Fault flag if queue length > 60 (near overflow)

### 4.3 CNN Processor  
- **Sparse RF**: Precomputed `rf_mask[ox][oy]` enables only valid kernel taps  
- **LIF decay**: `potential *= DECAY_FACTOR_CNN >> 8` per spike  
- Output: 4 confidence scores (one per filter)

### 4.4 RNN Processor  
- 32 hidden units, 16 input weights per unit  
- Event-driven: updates only on incoming spikes  
- Anomaly score = average firing rate over ~100 timesteps

### 4.5 Memory Subsystem  
- **Write pipeline**: 1-cycle delay to resolve skew  
- **Read path**: Immediate ECC correction  
- **Background scrubber**: Runs in IDLE when no sensor input  
- **SEU injection**: Direct override of SRAM word

---

## 5. Radiation Hardening Strategy  

| Technique | Applied To | Benefit |
|---------|-----------|-------|
| **TMR** | State machine, AER head/tail, command register | Masks single-bit flips |
| **SECDED ECC** | All SRAM words | Corrects 1-bit, detects 2-bit errors |
| **Background Scrubbing** | Full memory sweep in idle | Prevents error accumulation |
| **Clock Gating** | All processing units | Reduces dynamic upset rate |
| **Fault Reporting** | `fault_code`, `command_out=FF` | Enables system-level recovery |

---

## 6. Testbench & Verification  

### 6.1 Included Testbench (`tb_astronet_asic`)  
- Random sensor input (3 frames)  
- Double-bit error injection at address `0x1000`  
- Waveform dump: `astronet_v9.1.vcd`  
- Runs 1000 ns post-injection

### 6.2 Formal Properties (SVA)  
```systemverilog
p_no_aer_overflow:      assert !aer_full
p_command_valid_implies_valid_cmd: assert valid command values
p_fault_recovery:       assert fault_code clears on IDLE
```

### 6.3 Fault Injection Results (Expected)  
- Single-bit error → corrected silently  
- Double-bit error → `fault_sram` → `fault_code = 3'b011` → `command_out = 8'hFF` → recovery in IDLE

---

## 7. Configuration Parameters  

| Parameter | Default | Description |
|---------|--------|-----------|
| `IMAGE_WIDTH` | 4 | Sensor grid width |
| `CNN_FILTERS` | 4 | Number of CNN output classes |
| `RNN_UNITS` | 32 | RNN hidden layer size |
| `MEM_SIZE` | 32768 | SRAM depth (21-bit words) |
| `SPIKE_QUEUE_DEPTH` | 64 | AER FIFO size |
| `LIF_THRESHOLD` | `8'hFF` | Neuron firing threshold |
| `CNN_CONF_THRESH` | `8'h70` | CNN decision threshold |
| `RNN_ANOM_THRESH` | `8'h80` | RNN anomaly threshold |

---

## 8. Synthesis Notes  

- **Target**: Radiation-hardened 65nm or 28nm bulk CMOS (e.g., RTG65, GF28SLP-RH)  
- **Clock**: 100 MHz nominal (gated)  
- **Area Estimate** (pre-synthesis):  
  - CNN: ~8K GE  
  - RNN: ~6K GE  
  - SRAM + ECC: ~120K GE (32K × 21)  
  - Control + AER: ~3K GE  
  - **Total**: ~140K GE  
- **Power**:  
  - Active: ~15 mW @ 100 MHz  
  - Idle (`power_low`): < 100 µW  

---

## 9. Version History  

| Version | Date | Changes |
|--------|------|-------|
| **v9.1** | Oct 28, 2025 | Fixed SRAM write skew, CNN RF, RNN timing, spike flow; added TMR, ECC scrub, SEU injection |
| v9.0 | Oct 2025 | Initial neuromorphic integration |
| v8.x | 2024 | CNN-only prototype |

---

## 10. Usage  

### Simulate  
```bash
iverilog -g2012 -o astronet_v9.1 AstroNetASICv9.1.txt
vvp astronet_v9.1
gtkwave astronet_v9.1.vcd
```

### Formal (with SymbiYosys)  
```bash
sby -f astronet.sby
```

### Synthesize (example with Yosys)  
```bash
yosys -p "read_verilog AstroNetASICv9.1.txt; synth -top astronet_asic" > synth.ys
```

---

## 11. License  

**MIT License** – Use in space-qualified systems requires full radiation test validation.

---

**AstroNet ASIC v9.1** – *First-principles neuromorphic computing for the final frontier.*  

*Built with First Principles reasoning: correct by construction, verifiable, and radiation-resilient.*
