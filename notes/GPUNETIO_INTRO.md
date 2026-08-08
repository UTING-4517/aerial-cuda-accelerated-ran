# DOCA GPUNetIO 入門：給有 DPDK 背景的人

**這份文件的定位**：假設你熟悉 DPDK / FlexRAN（lcore、mbuf、`rte_eth_rx_burst`、`rte_flow`），但沒有 GPU 開發經驗。目標是用你已知的東西當座標系，建立正確的 GPUNetIO 心智模型。

**不需要**先懂 CUDA。需要用到的 GPU 概念都在第 3 節從頭解釋。

**對照的程式碼**：本 repo（NVIDIA Aerial cuBB），設定基準 `cuPHY-CP/cuphycontroller/config/cuphycontroller_F08_WNC_DGX.yaml`（DGX Spark / GB10 + 整合式 ConnectX-7）。所有行號都經過實際讀取確認。

> **路徑慣例**：文中所有 `檔案:行號` 都是**相對於 repo 根目錄**（本文件位於 `notes/`，往上一層）。

**姊妹文件**：[`TX_PATH_CPLANE_UPLANE.md`](TX_PATH_CPLANE_UPLANE.md)（發送路徑的完整拆解）

---

## 0. 一句話說清楚差別

| | 收包由誰做 |
|---|---|
| **FlexRAN / 傳統 DPDK** | CPU core 跑 polling loop，`rte_eth_rx_burst()` 把封包撈進 mbuf，CPU 解析 |
| **Aerial GPUNetIO** | **GPU 上的 CUDA kernel 自己呼叫收包函式**，封包直接落在 GPU 讀得到的記憶體，解析、解壓縮、落位全在 GPU |

**上行 U-plane 完全沒有 CPU core 在輪詢網卡。**

從 DPDK 過來最容易誤判的一點：設定檔的 `dpdk_thread: 5` **不是**收包核心。它只用來釘住呼叫 `rte_eal_init()` 的那個 thread，初始化完就把原本的 affinity mask 還原了：

```cpp
// aerial-fh-driver/lib/fronthaul.cpp:123-131  設 affinity 到 dpdk_thread (SCHED_FIFO 95)
CPU_SET(info_.dpdk_thread, &new_cpu_affinity_mask);
sched_setaffinity(0, sizeof(new_cpu_affinity_mask), &new_cpu_affinity_mask);
...
// aerial-fh-driver/lib/fronthaul.cpp:152-157  EAL init 完成後還原
// "Restore the original CPU affinity mask and scheduling policy"
sched_setaffinity(0, sizeof(cpu_affinity_mask), &cpu_affinity_mask);
```

用 `perf` / `top` 去看 cpu5，不會看到收包工作。

---

## 1. 為什麼要這樣做

5G L1 上行一個 slot 的工作量：273 PRB × 12 子載波 × 14 symbol × 4 天線 ≈ **18 萬個複數**，要做通道估計、等化、解調、LDPC 解碼，而且要在 **500 µs** 內完成（μ=1，30 kHz SCS）。

這是「大量相同運算套用在大量獨立資料上」——GPU 擅長、CPU 不擅長的形狀。

既然資料最終一定要進 GPU，就有個選擇題：

| 做法 | 資料路徑 |
|---|---|
| 傳統 | NIC → 主記憶體(mbuf) → CPU 解析 → **複製到 GPU** → GPU 運算 |
| GPUNetIO | NIC → **GPU 可直接讀的記憶體** → GPU 解析 → GPU 運算（同一塊記憶體） |

省下的不只是那次複製。**更重要的是省掉 CPU 逐包解析 O-RAN header 的工作**：4 個 eAxC × 14 symbol，每 slot 數百個封包，每包都要拆 header、查 eAxC 表、算落點、BFP 解壓縮。CPU 做這個會吃掉整個 slot 預算。

---

## 2. 好消息：你的 DPDK 知識大半直接適用

**GPUNetIO 沒有取代 DPDK，它只接管了資料面。控制面全部還是 DPDK。**

| 你熟悉的 | 在 Aerial 裡還在不在 | 位置 |
|---|---|---|
| `rte_eal_init()` + hugepage + PCI 綁定 | **完全一樣** | `aerial-fh-driver/lib/fronthaul.cpp:38-70` |
| `rte_eth_dev_configure()` / MTU / offloads | **完全一樣** | `aerial-fh-driver/lib/nic.cpp:365-397` |
| **`rte_flow` 規則** | **完全一樣，而且是核心** | `aerial-fh-driver/lib/peer.cpp:528-693` |
| `rte_flow_isolate()` | **完全一樣** | `aerial-fh-driver/lib/nic.cpp:853-864` |
| `rte_eth_tx_burst()` | **C-plane 還在用** | `aerial-fh-driver/lib/queue.cpp:122` |
| mbuf / mempool | **C-plane 還在用** | `aerial-fh-driver/lib/nic.cpp:458-533` |
| `RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP` | **完全一樣** | `aerial-fh-driver/lib/nic.cpp:374-384` |
| mlx5 PMD devargs（`dv_flow_en=2` 等） | **完全一樣** | `aerial-fh-driver/lib/nic.cpp:242` |

被換掉的**只有**：上行 U-plane 的 `rte_eth_rx_burst()`，以及下行 U-plane 的送出。

### 兩者是縫在一起的

`aerial-fh-driver/lib/queue.cpp:304-325` 的 `Rxq` 建構子：

```cpp
if(!(fh->get_info().cuda_device_ids.empty()) && !fh->get_info().cpu_rx_only) {
    // DOCA GPUNetIO 路徑
    doca_create_rx_queue(&doca_rx_h, ..., DOCA_GPU_MEM_TYPE_CPU_GPU, ...);   // :313-315
    rte_pmd_mlx5_external_rx_queue_id_map(port_id,
                                          doca_rx_h.dpdk_queue_idx,
                                          doca_rx_h.hw_queue_idx);            // :319  ★ 縫合點
} else {
    rte_eth_rx_queue_setup(port_id, id_, size_, socket_id, nullptr, mp);      // :327  傳統 DPDK
}
```

**`:319` 是理解整套架構的關鍵**：DOCA 建了一個硬體 queue，然後把它註冊回 DPDK 的 queue index 空間，這樣 `rte_flow` 規則才有辦法用 `RTE_FLOW_ACTION_TYPE_QUEUE` 指到它。

> **正確的心智模型**：DPDK 決定封包去哪個 queue，GPUNetIO 決定誰來消費那個 queue。

---

## 3. 要新建立的 GPU 概念

只有五個，而且都有 DPDK 的類比。

### 3.1 執行單位：thread / warp / block / SM

| GPU 概念 | 是什麼 | 類比 |
|---|---|---|
| **thread** | 最小執行單位 | 一個 lcore 上的一次迭代 |
| **warp** | **32 個 thread 綁在一起，鎖步執行同一條指令** | 像 SIMD lane，但每個 lane 有自己的暫存器，可以分支 |
| **block (CTA)** | 一群 thread，可共用 shared memory、可互相同步 | 一個 lcore 的工作單位 |
| **SM (Streaming Multiprocessor)** | 實體運算單元，一顆 GPU 有數十個 | **實體 CPU core** |

**關鍵直覺**：同一個 warp 內的 32 個 thread 若走不同分支會「發散」（divergence），兩條路徑都要各跑一遍。所以 GPU code 會盡量讓同一個 warp 做同構的事。

本 repo 的例子：order kernel 是「**一個 warp 處理一個封包**」——32 個 thread 協作處理同一個封包的不同 PRB，天然同構。

### 3.2 kernel 與 launch

**kernel** = 一個 `__global__` 函式，由 CPU 「發射」到 GPU 執行，發射時指定用多少 block、每 block 多少 thread：

```cpp
// cuphydriver/src/uplink/order_cuda_kernels.cu:5863
order_kernel_doca_single_subSlot_pingpong<false, 0, 0, 320, 2>
    <<< cudaBlocks, ORDER_KERNEL_PINGPONG_NUM_THREADS, 0, stream >>>( ... );
//      ^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//      幾個 block   每 block 幾個 thread (= 320，見 :39)
```

`<<< >>>` 是 CUDA 特有語法。本設定是 **每個 cell 一個 block、每 block 320 threads（= 10 warps）**（`cudaBlocks = num_order_cells`，`:5769`）。

> ⚠️ `:5769` 的註解 `//# of Thread blocks should be twice the number of cells` 是**過期註解**，程式碼實際是 `int cudaBlocks = (num_order_cells);`。

**對 DPDK 使用者最違反直覺的地方**：這個 kernel 不是「處理一批就結束」。它裡面有 `while(1)` 迴圈，**持續執行數百微秒**，一邊收包一邊處理，直到收滿預期 PRB 數或逾時才退出。

CPU 每個 UL slot 發射它一次：

```cpp
// cuphydriver/src/uplink/order_entity.cpp:1387
launch_order_kernel_doca_single_subSlot(first_strm, ...);
```

發射時機是 **slot 邊界前 500 µs**：

```cpp
// cuphydriver/include/constant.hpp:132
static constexpr uint32_t UL_TASK1_ORDER_LAUNCH_OFFSET_FROM_T0_NS = 500000;
///< Order kernel launch: T0 - 500us (PUSCH/PUCCH ordering)
```

概念上等同 DPDK 的 polling loop，差別是：跑在 GPU 上、320 個 thread 同時跑、每個 slot 重啟一次。

### 3.3 stream：GPU 的執行佇列

**stream** 是一串照順序執行的 GPU 工作。不同 stream 之間可並行。

```cpp
// aerial-fh-driver/lib/gpu_comm.cpp:52
cudaStreamCreateWithPriority(&cstream_, cudaStreamNonBlocking, -5);
```

把 stream 想成一條 pipeline。下行送包在一條 stream 上依序跑 memset → pre_prepare → prepare；壓縮 kernel 在另一條（cell 的 DL stream）跑。兩條之間用 **event** 同步：

```cpp
// cuphydriver/src/downlink/task_function_dl_aggr.cpp:1338
cudaStreamWaitEvent(..., comm_preprep_stop_evt, ...);
```

類比 DPDK ring 的生產者/消費者，只是同步的是「工作順序」而非「資料」。

### 3.4 記憶體種類 — 最容易踩坑的地方

| 種類 | 誰能存取 | 能否被網卡 DMA |
|---|---|---|
| **device memory**（`cudaMalloc`） | 只有 GPU | 需要 GPUDirect RDMA |
| **host memory**（`malloc`） | 只有 CPU | **不行**（OS 可能換頁） |
| **pinned host memory** | CPU 直接存取，GPU 也能讀寫 | **可以** |

一般 `malloc` 的記憶體 OS 可以隨時換頁（swap / 移動實體位址），網卡 DMA 不能用。要讓硬體 DMA，必須先「釘住」（pin / page-lock）。DPDK 的 hugepage mempool 就是在解決同一個問題。

DOCA 的兩種型別，在本 repo 的分岔點：

```cpp
// aerial-fh-driver/lib/doca_obj.cpp:154-160
if(mtype == DOCA_GPU_MEM_TYPE_CPU_GPU){
    doca_gpu_mem_alloc(..., DOCA_GPU_MEM_TYPE_CPU_GPU, &gpu_pkt_addr, &cpu_pkt_addr);
} else {
    doca_gpu_mem_alloc(..., DOCA_GPU_MEM_TYPE_GPU, &gpu_pkt_addr, NULL);
}

// aerial-fh-driver/lib/doca_obj.cpp:167-197
if (!enable_gpu_comm_via_cpu) {
    doca_gpu_dmabuf_fd(...);                              // 先試 dmabuf
    // 失敗則退回 nvidia-peermem: doca_mmap_set_memrange(gpu_pkt_addr, ...)
} else {
    doca_mmap_set_memrange(mmap, item->cpu_pkt_addr, ...); // :192  用 CPU 位址註冊
}
```

**`DOCA_GPU_MEM_TYPE_GPU` 要求把 GPU 記憶體的位址註冊給網卡做 DMA — 這就是 GPUDirect RDMA**，需要 GPU 透過 PCIe BAR 把記憶體暴露出來，並靠 dmabuf 或 `nvidia-peermem` 核心模組建立映射。

### 3.5 MPS 與 SM 配額 — 你的 `mps_sm_*` 設定

GPU 預設一次跑一個 context，其他排隊。**MPS（Multi-Process Service）** 讓多個工作真正並行，並可指定**每個 context 最多用幾個 SM**。

Aerial 為每個功能建一個 context：

```cpp
// cuphydriver/src/common/context.cpp:757-800（非 green-context 分支）
puschMpsCtx    = new MpsCtx(..., getMpsSmPusch());      // mps_sm_pusch: 40
pdschMpsCtx    = new MpsCtx(..., getMpsSmPdsch());      // mps_sm_pdsch: 46
ulMpsCtx       = new MpsCtx(..., getMpsSmUlOrder());    // mps_sm_ul_order: 12
gpuCommsMpsCtx = new MpsCtx(..., getMpsSmGpuComms());   // mps_sm_gpu_comms: 16
```

**這就是 GPU 版的 CPU core 隔離。** 你在 FlexRAN 用 `isolcpus` 把核心切給不同功能、避免互搶；Aerial 用 MPS 把 SM 切給不同 channel，避免 PDSCH 的大 kernel 把收包 kernel 餓死。

實作（`cuphydriver/src/common/mps.cpp:47-67`）：

```cpp
cuDeviceGetAttribute(&actualDevSmCount, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, cuDev);
if (actualDevSmCount < devSmCount) {                                            // :50
    throw std::runtime_error("Requested N SMs in cuCtxCreate_v3() but GPU has max M SMs.");
}
CUexecAffinityParam affinityPrm;
affinityPrm.type = CU_EXEC_AFFINITY_TYPE_SM_COUNT;                              // :58
affinityPrm.param.smCount.val = devSmCount;                                     // :59
cuCtxCreate_v3(&cuCtx, &affinityPrm, 1, CU_CTX_SCHED_SPIN|CU_CTX_MAP_HOST, cuDev); // :67
```

**兩個重要釐清**：

1. **檢查的是單一 context 的值，不是所有 `mps_sm_*` 的總和。** 本設定各值加起來 = 154，遠大於實體 SM 數 — 這是**正常的**，各 context 超額訂閱，由硬體排程。若啟動時 throw，是某**單一**值超過 GPU 的 SM 總數。

2. `mps.cpp:59` 的註解直說：SM 數合法時若 `cuCtxCreate_v3` 仍回 **CUDA error 224**，那只可能是 **MPS daemon 沒在跑**。

**哪個旋鈕管哪個 kernel**（常見誤解）：
- 上行收包 order kernel → **`mps_sm_ul_order`**
- 下行送包 GPU comm kernel → `mps_sm_gpu_comms`
- DL BFP 壓縮 kernel → **`mps_sm_pdsch`**（它跑在 cell 的 DL stream，`context.cpp:782` 的 `dlMpsCtx = pdschMpsCtx`），**不是** `mps_sm_gpu_comms`

---

## 4. 一個上行封包的完整旅程

```
RU 送出封包
    ↓
NIC 硬體比對 rte_flow 規則                                  ← 純 DPDK
    ETH(dst/src MAC, full mask) + VLAN(TCI full mask, 含 PCP)
    + eCPRI(msg type=IQ_DATA, pc_id=eAxC)
    peer.cpp:528-693
    ↓  比中 → QUEUE action；沒比中 → rte_flow_isolate 硬體丟棄（nic.cpp:853-864）
DOCA cyclic packet buffer（DOCA_ETH_RXQ_TYPE_CYCLIC）        ← 取代 mbuf pool
    doca_obj.cpp:126, 154-197
    ↓
GPU order kernel 呼叫 doca_gpu_dev_eth_rxq_recv()            ← 取代 rte_eth_rx_burst()
    order_cuda_kernels.cu:3983
    一次最多 512 包（ul_order_max_rx_pkts）/ 100 µs 逾時
    ↓
同一個 kernel 內，一個 warp 一個封包：
    解析 O-RAN header → 用 section_id 分流 PUSCH/PRACH
    → 查 eAxC 表得天線 index → 算落點 → BFP-9 解壓 → 寫入 IQ tensor
    ↓
cuPHY PUSCH pipeline 直接讀那塊 tensor（零複製）
    ↓
結果 D2H 複製回 pinned host memory → L2 (FAPI)
```

**落點計算**（`aerial-fh-driver/include/aerial-fh-driver/oran.hpp:1633-1638`）：

```cpp
offset = flow_index * symbols_x_slot * prbs_per_symbol * prb_size   // 天線平面
       + symbol_id  * prbs_per_symbol * prb_size                    // symbol 平面
       + startPrb   * prb_size;                                     // PRB 偏移
```

佈局是 **[天線][symbol][PRB]**。`flow_index` 由 eCPRI 的 `pc_id` 查 eAxC map 得到。

**收斂與逾時**：per-symbol PRB 數達標 → 標記該 symbol 完成；整個 slot 達標 → 正常退出。逾時分兩層：3 ms 完全沒收到第一包，或第一包之後 1.5 ms 沒收完。逾時的 slot 不會被當有效結果送出 — 該 cell 所有 TB 的 CRC 會被強制標為錯誤（`phypusch_aggr.cpp:1089-1099`），在 L2 看起來就是 CRC.indication 失敗。

---

## 5. 下行送包（摘要）

完整拆解見 `TX_PATH_CPLANE_UPLANE.md`。這裡只講與 DPDK 直覺的差異：

| 平面 | 誰組包 | 怎麼送 |
|---|---|---|
| **C-plane** | CPU | `rte_eth_tx_burst()`（`queue.cpp:122`）— 完全是你熟的 DPDK |
| **U-plane (DL IQ)** | **GPU kernel 寫 WQE** | GPU 直接構造 mlx5 的 Work Queue Entry，再「按門鈴」通知網卡 |

**WQE 與 doorbell** 是 DPDK 幫你隱藏掉的底層：`rte_eth_tx_burst()` 內部就是在做「填 WQE + 寫 doorbell 暫存器」。GPUNetIO 把這兩步搬到 GPU 上做（或部分做）。

隔離是強制的 — `Txq::send()` 一旦發現自己是 GPU queue 就直接 throw：

```cpp
// aerial-fh-driver/lib/queue.cpp:114-117
if(is_gpu())
    THROW_FH(ENOTSUP, ... << " because it's a GPU-init comm queue");
```

---

## 6. FlexRAN 直覺會誤導你的五件事

### ① 「加 CPU core 就能解決收包問題」— 不適用

上行收包沒有 CPU core 參與。收包吃緊要調的是 **`mps_sm_ul_order`**（給 order kernel 更多 SM），不是核心配置。

### ② 「DPDK 統計能告訴我收了多少包」— 不能

`rte_eth_stats_get()` / `xstats` 的 RX 計數反映的是 DPDK 自己管理的 queue。U-plane queue 是 DOCA 建的、GPU 消費的，**DPDK 那套統計看不到**。

要看實際收包狀況，看 order kernel 自己維護的計數：

```
[RX Packet Times] { EARLY: n ONTIME: n LATE: n }
```
（`cuphydriver/src/uplink/slot_map_ul.cpp:619`）

或直接讀網卡硬體計數器。

### ③ 「封包丟了會有 log」— 很多時候不會

三個靜默丟包/錯置點：

1. **`rte_flow` 沒比中** → 硬體直接丟，任何軟體層都看不到。規則是 full-mask 比對 MAC + VLAN TCI（**含 PCP**）+ eAxC ID，任一 bit 不對就完全不匹配。
2. **`get_eaxc_index()` 查不到** → 回傳 **0**，不是錯誤碼：
   ```cpp
   // order_cuda_kernels.cu:64-73
   for(i...) if(eAxC_map[i] == eAxC_id) return i;
   return 0;                                    // ← 查不到就當天線 0
   ```
   封包會被靜靜寫進天線 0 的位置。
3. **Ta4 時間窗** → **只計數不丟包**。太早或太晚的封包照樣處理，只是分別累加到 EARLY / LATE 計數。

### ④ 「延遲來自 CPU 排程」— 來源變了

新的延遲來源：
- **kernel launch 開銷** → 所以有 CUDA graph（`enable_ul_cuphy_graphs: 1`），把整個 slot 的 kernel 序列預先錄好，每 slot 只 replay
- **SM 爭用** → 所以有 MPS 配額
- **kernel 逾時** → `ul_order_timeout_gpu_ns: 3000000`（3 ms）

### ⑤ 「gdb / perf 能看到問題」— 工具換了

見第 7 節。

---

## 7. 工具對照

| 你熟悉的 | GPU 對應 | 用途 |
|---|---|---|
| `perf` / `top` | **Nsight Systems**（`nsys`） | 時間軸：kernel 何時跑、跑多久、誰在等誰 |
| `perf annotate` | **Nsight Compute**（`ncu`） | 單一 kernel 內部的瓶頸分析 |
| `gdb` | **`cuda-gdb`** | 進 kernel 設中斷點 |
| `printf` 除錯 | kernel 內 `printf()` | **可用但嚴重影響時序** — 這就是 Aerial 的 `printf` 幾乎都在錯誤路徑上的原因 |
| `rte_eth_stats_get()` | order kernel 的計數 + 網卡硬體計數器 | 見 6-② |

---

## 8. 術語速查

| 術語 | 意思 |
|---|---|
| **CUDA** | NVIDIA 的 GPU 程式設計平台 |
| **kernel** | 在 GPU 上執行的函式（`__global__`） |
| **launch** | CPU 發射 kernel 到 GPU（`<<<blocks, threads>>>`） |
| **thread / warp(32) / block / grid** | 執行階層，由小到大 |
| **SM** | Streaming Multiprocessor，實體運算單元 ≈ CPU core |
| **stream** | GPU 上的一條順序執行佇列 |
| **event** | stream 之間的同步點 |
| **device / host memory** | GPU 記憶體 / CPU 記憶體 |
| **pinned memory** | 被釘住不會換頁的 host 記憶體，可被硬體 DMA |
| **H2D / D2H** | Host-to-Device / Device-to-Host 複製 |
| **GPUDirect RDMA** | 網卡直接 DMA 進 GPU 記憶體，不經主記憶體 |
| **MPS** | Multi-Process Service，讓多 context 並行並可限制 SM 數 |
| **CUDA graph** | 預先錄好的 kernel 序列，重播以省 launch 開銷 |
| **WQE** | Work Queue Entry，網卡的工作描述符 |
| **doorbell** | 寫入網卡暫存器，通知「有新的 WQE 了」 |
| **UAR** | User Access Region，doorbell 暫存器的記憶體映射 |
| **CPU proxy mode** | GPU 無法直接寫 doorbell 時，由 CPU 代寫的模式 |

---

## 9. 本平台（DGX Spark / GB10）的特殊性

**這點很重要，因為網路上的 GPUNetIO 教學多半不適用於你的平台。**

NVIDIA 官方文件明文寫著：

> "Due to hardware topology limitations, **DGX Spark does not support GPUDirect RDMA**."
>
> "…can still execute on these systems by utilizing CPU pinned memory (`DOCA_GPU_MEM_TYPE_CPU_GPU`) instead of GPU memory."
>
> "When creating Rx or Tx queues with DOCA Ethernet, you must use the setters in `doca_eth_rxq_gpu_data_path.h` and `doca_eth_txq_gpu_data_path.h` to allocate queues on the CPU-GPU shared memory."
>
> "**For Tx queues, you must also enable CPU proxy mode to handle transmission**."
>
> — [GPUNetIO Installation and Setup](https://networking-docs.nvidia.com/doca/archive/3-4-0/gpunetio-installation-and-setup)

在 Aerial 裡，這一切由**一個設定旗標**打開：

```yaml
gpu_init_comms_via_cpu: 1
```

對應的實作：

| 官方要求 | Aerial 程式碼 |
|---|---|
| Rx queue 用 CPU-GPU shared memory | `doca_eth_rxq_gpu_set_rq_mem_type(..., CPU_GPU)` `doca_obj.cpp:131` |
| Tx queue 同上 | `doca_eth_txq_gpu_set_sq_mem_type(..., CPU_GPU)` `doca_obj.cpp:626` |
| **Tx 必須開 CPU proxy** | `doca_eth_txq_gpu_set_uar_on_cpu()` `doca_obj.cpp:633` + 跳過 GPU trigger kernel `gpu_comm.cpp:465` + `doca_eth_txq_gpu_cpu_proxy_progress()` `gpu_comm.cpp:318-323` |

### 實務意涵

1. **沒開這個旗標會怎樣**：DOCA 嘗試註冊 GPU 記憶體給網卡，回 **EFAULT (errno 14)**，`doca_mmap_start` 失敗，NIC 註冊失敗，程式 FATAL exit。這不是 bug，是平台限制。

2. **收送兩個方向的代價不對稱**：
   - **接收**：只是換記憶體位置，**不多一次複製**
   - **發送**：每個 symbol 多一趟 GPU→CPU 的 D2H 搬移（`gpu_comm.cpp:236-278`），且 doorbell 由 CPU 敲

3. **抄範例要小心**：網路上的 GPUNetIO 範例多半假設有獨立 GPU + GPUDirect RDMA，直接照抄會踩到上面那個 EFAULT。

---

## 10. 建議的入門順序

由淺入深，每步都能在本 repo 對照：

1. **先確認你已經懂的那半**：讀 `peer.cpp:528-693` 的 `rte_flow` 規則建立。這 100% 是 DPDK，應該很親切。
2. **看 queue 怎麼被建出來**：`queue.cpp:273-331`。一個 `if/else` 分出 DOCA 和 DPDK 兩條路，`:319` 是縫合點。
3. **看 kernel 怎麼被發射**：`order_entity.cpp:1387` 附近。這是 CPU 側，還是普通 C++。
4. **最後才進 kernel 內部**：`order_cuda_kernels.cu:3203` 起。先看 `while(1)` 的骨架（process → timeout check → receive），再看細節。
5. **官方最小範例**：DOCA 的 GPUNetIO sample 比 Aerial 這套簡單一個數量級，適合先建立手感。本 repo 也有 `testBenches/doca_samples/doca_gpunetio_send_wait_time/`。

---

## 11. 延伸閱讀

**官方文件**
- [DOCA GPUNetIO 總覽](https://networking-docs.nvidia.com/doca/archive/3-4-0/doca-gpunetio)
- [GPUNetIO Architecture and Design](https://networking-docs.nvidia.com/doca/archive/3-4-0/gpunetio-architecture-and-design)
- [GPUNetIO Installation and Setup](https://networking-docs.nvidia.com/doca/archive/3-4-0/gpunetio-installation-and-setup) — **Spark 相關限制在這頁**
- [GPUNetIO Sample Guide](https://docs.nvidia.com/doca/sdk/gpunetio-sample-guide/index.html)

**本 repo 文件**
- [`TX_PATH_CPLANE_UPLANE.md`](TX_PATH_CPLANE_UPLANE.md) — 發送路徑（C-plane DPDK + U-plane GPU）完整拆解
- [`../5GModel/aerial-cuda-accelerated-ran.pdf`](../5GModel/aerial-cuda-accelerated-ran.pdf) — Aerial 官方文件

---

## 附錄：關鍵檔案地圖

| 檔案 | 負責什麼 |
|---|---|
| `cuPHY-CP/aerial-fh-driver/lib/fronthaul.cpp` | DPDK EAL 初始化、accurate send scheduling 設定 |
| `cuPHY-CP/aerial-fh-driver/lib/nic.cpp` | NIC 探測、port 設定、queue 建立順序、mbuf pool |
| `cuPHY-CP/aerial-fh-driver/lib/queue.cpp` | TXQ / RXQ 建構（**DOCA vs DPDK 分岔點**） |
| `cuPHY-CP/aerial-fh-driver/lib/doca_obj.cpp` | DOCA queue / buffer 建立，記憶體型別與註冊方式 |
| `cuPHY-CP/aerial-fh-driver/lib/peer.cpp` | 每個 O-RU 的 TXQ 擁有者、rte_flow 規則、封包組裝 |
| `cuPHY-CP/aerial-fh-driver/lib/flow.cpp` | 每個 eAxC 的封包 header template |
| `cuPHY-CP/aerial-fh-driver/lib/gpu_comm.cpp` | 下行 GPU 送包的 CPU 側協調 |
| `cuPHY-CP/aerial-fh-driver/lib/gpu_comm_doca.cu` | 下行 GPU 送包的 CUDA kernel |
| `cuPHY-CP/cuphydriver/src/uplink/order_cuda_kernels.cu` | **上行收包 + 解析 + 解壓縮的 CUDA kernel** |
| `cuPHY-CP/cuphydriver/src/uplink/order_entity.cpp` | order kernel 的 CPU 側發射與生命週期 |
| `cuPHY-CP/cuphydriver/src/common/fh.cpp` | cuphydriver 與 FH driver 的接合層、C-plane 內容產生 |
| `cuPHY-CP/cuphydriver/src/common/mps.cpp` | MPS context 與 SM 配額 |
| `cuPHY-CP/cuphydriver/src/common/context.cpp` | 各 channel 的 MPS context 建立 |
