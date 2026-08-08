# DOCA GPUNetIO 入門：給有 DPDK 背景的人

**這份文件的定位**：假設你會用 DPDK 收送封包（`rte_eth_rx_burst` / `tx_burst`、mbuf），但沒有 GPU 開發經驗。目標是用你已知的東西當座標系，建立正確的 GPUNetIO 心智模型。

**不需要**先懂 CUDA。需要用到的 GPU 概念都在第 3 節從頭解釋。

**不需要**先懂 `rte_flow`。但它在 Aerial 裡很關鍵，補在 [`DPDK_ADVANCED.md`](DPDK_ADVANCED.md)。

**對照的程式碼**：本 repo（NVIDIA Aerial cuBB），設定基準 `cuPHY-CP/cuphycontroller/config/cuphycontroller_F08_WNC_DGX.yaml`（DGX Spark / GB10 + 整合式 ConnectX-7）。所有行號都經過實際讀取確認。

> **路徑慣例**：帶目錄的路徑一律**相對於 repo 根目錄**（本文件位於 `notes/`，往上一層）。為了可讀性，重複出現的檔案會簡寫成檔名（`queue.cpp:319`）或只留行號（`:319`，指前一個提到的檔案）——完整路徑見文末的**附錄：關鍵檔案地圖**。

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
// cuPHY-CP/aerial-fh-driver/lib/fronthaul.cpp:126-131  設 affinity 到 dpdk_thread
CPU_SET(info_.dpdk_thread, &new_cpu_affinity_mask);
sched_setaffinity(0, sizeof(new_cpu_affinity_mask), &new_cpu_affinity_mask);

// :138-145  SCHED_FIFO 95 是條件編譯的，不是無條件行為
#ifdef ENABLE_SCHED_FIFO_ALL_RT
pthread_setschedparam(thread_id, new_scheduling_policy, &new_scheduling_params);  // SCHED_FIFO, prio 95
#endif
...
// :154-167  EAL init 完成後還原
// "Restore the original CPU affinity mask and scheduling policy"
sched_setaffinity(0, sizeof(cpu_affinity_mask), &cpu_affinity_mask);          // :155
pthread_setschedparam(thread_id, scheduling_policy, &scheduling_params);      // :162  無條件還原
```

> `ENABLE_SCHED_FIFO_ALL_RT` 是 top-level CMake option，**預設 ON**（`CMakeLists.txt:163`、`:219-220`），所以預設 build 確實會套用 `SCHED_FIFO 95`。但只有**設定**那一步被 macro 包住——affinity 的設定與還原、以及 scheduling policy 的**還原**（`:162`）都是無條件執行的。

用 `perf` / `top` 去看 cpu5，不會看到收包工作。

---

## 1. 為什麼要這樣做

5G L1 上行一個 slot 的工作量：273 PRB × 12 子載波 × 14 symbol × 4 天線 ≈ **18 萬個複數**，要做通道估計、等化、解調、LDPC 解碼。而 μ=1（30 kHz SCS）時 slot 長度是 **500 µs**——每 500 µs 就有新的一批進來。

> ⚠️ **別把「slot 長度 500 µs」讀成「所有處理都要在 500 µs 內做完」。** 各 channel 的實際期限由 HARQ/FAPI timing 與 Aerial 的 task schedule 決定，明顯跨越單一 slot：SRS order kernel 要到 `T0 + 2500 µs` 才 launch（`constant.hpp:133`），early UCI indication 在 `T0 + 1500 µs`（`:138`），order kernel 自己的逾時就設 3 ms（`ul_order_timeout_gpu_ns: 3000000`）。真正的壓力來自**持續吞吐**，不是單一 500 µs 死線。

這是「大量相同運算套用在大量獨立資料上」——GPU 擅長、CPU 不擅長的形狀。

既然資料最終一定要進 GPU，就有個選擇題：

| 做法 | 資料路徑 |
|---|---|
| 傳統 | NIC → 主記憶體(mbuf) → CPU 解析 → **複製到 GPU** → GPU IQ tensor → GPU 運算 |
| GPUNetIO | NIC → **GPU 可直接讀的 DOCA packet buffer** → GPU 解析／解壓縮 → cuPHY IQ tensor → GPU 運算；**無 CPU staging、無 H2D copy** |

> ⚠️ **「GPU 可直接讀」不等於「全程同一塊記憶體」。** 這裡有兩塊不同的 buffer：NIC 寫入的 DOCA cyclic packet buffer（`order_cuda_kernels.cu:3608` 用 `doca_gpu_dev_eth_rxq_get_pkt_addr()` 取址），以及 order kernel 解壓縮後寫入的 PUSCH/PRACH/SRS IQ tensor（`:3735-3766` 選 buffer、`:3782-3795` 寫入）。省掉的是 **CPU 解析、CPU staging 與 H2D copy**，不是「完全沒有資料搬移」。

省下的不只是那次複製。**更重要的是省掉 CPU 逐包解析 O-RAN header 的工作**：4 個 eAxC × 14 symbol，每 slot 數百個封包，每包都要拆 header、查 eAxC 表、算落點、BFP 解壓縮。CPU 若逐包完成這些工作，會顯著侵蝕每個 slot 的持續吞吐與處理裕量。

---

## 2. DPDK 那一半：什麼還在、什麼要補

**GPUNetIO 沒有取代 DPDK。** O-RAN C-plane 封包仍由 CPU/DPDK 收送，DPDK 也繼續負責 EAL、port 設定、`rte_flow` 與 flow isolation。

被換掉的是：上行 U-plane 的 `rte_eth_rx_burst()`，以及下行 U-plane 的送出。

> ⚠️ **但別把它讀成「控制面全部還是 DPDK」。** 這句話只在「控制面」專指 O-RAN C-plane 封包時才成立。GPU queue 的**建立**本身也是 control path，而那是 DOCA 做的：TX 分支呼叫 `doca_create_tx_queue()` 並跳過 `rte_eth_tx_queue_setup()`（`queue.cpp:75-97`），RX 分支呼叫 `doca_create_rx_queue()` 並跳過 `rte_eth_rx_queue_setup()`（`:304-330`）。Aerial 用的是 **DPDK + DOCA 混合的 control path**，GPU kernel 只負責 U-plane data path。

但「還是 DPDK」不代表「你一定用過」。O-RAN fronthaul 用到的 DPDK 功能比一般應用多：

| DPDK 主題 | 一般 DPDK 應用會碰到嗎 | 在 Aerial 的重要性 | 位置 |
|---|---|---|---|
| `rte_eth_rx_burst` / `tx_burst` | ✅ 一定會 | C-plane 送出還在用；**上行收包已被 GPU 取代** | tx `queue.cpp:122`；rx `:340, :357`（本設定不走） |
| mbuf / mempool | ✅ 一定會 | C-plane 用；上行收包不用 | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp:458-533` |
| `rte_eal_init()` + hugepage | ✅ 一定會 | 不變 | `fronthaul.cpp:147`（wrapper），本設定由 `doca_gpu_setup()` `:275` 呼叫 |
| `rte_eth_dev_configure()` | ✅ 一定會 | 不變 | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp:366-398`（呼叫在 `:390`）|
| **`rte_flow`** | ❌ 多數應用不用 | **最高**——上行封包能不能收到全看它 | `cuPHY-CP/aerial-fh-driver/lib/peer.cpp:528-693` |
| **`rte_flow_isolate()`** | ❌ 少見 | **高**——不符規則的封包靜默消失 | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp:854-865` |
| **`SEND_ON_TIMESTAMP`** | ❌ 少見 | 高——O-RAN 時序靠它 | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp:374-384` |
| **mlx5 PMD devargs** | ❌ 少見 | 中——影響 flow 引擎與 zero-copy | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp:239-251` |

> **下半部那四項如果沒接觸過，先看 [`DPDK_ADVANCED.md`](DPDK_ADVANCED.md)。**
> 它們跟 GPU 無關（就算不用 GPUNetIO，做 O-RAN fronthaul 一樣要用），但對除錯的重要性可能比 GPU 那半還高——收不到封包時，第一個要懷疑的是 flow rule 沒比中，不是 GPU。

### 兩者是縫在一起的

`cuPHY-CP/aerial-fh-driver/lib/queue.cpp:273-331` 的 `Rxq` 建構子（以下節錄 DOCA/DPDK 分岔的 `:304-330`）：

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
| **warp** | **32 個 thread 一組，共用指令發射資源** | ≈ 一條 32-wide 的 SIMD 指令（對應 lane 的是單一 thread，不是 warp）|
| **block (CTA)** | 一群 thread，可共用 shared memory、可互相同步 | 一個 lcore 的工作單位（但有兩條硬約束，見下）|
| **grid** | 一次 launch 的所有 block | 整批工作 |
| **SM (Streaming Multiprocessor)** | 實體運算單元，GB10 約 **48 個**（見下方註）| **實體 CPU core**（但一個 SM 能同時常駐數十個 warp）|

> SM 數量不是從本 repo 得到的。要確認自己機器的實際值：`nvidia-smi -q | grep -i multiprocessor`，或看 Aerial 啟動時 MPS 配額檢查的錯誤訊息——`mps.cpp:51` 的 `"...but GPU has max N SMs"` 會直接印出來。這個數字是 §3.5 判斷 `mps_sm_*` 是否超標的依據。

> **block 有兩條硬約束，而它們正是本 repo 的設計前提：**
> 1. **一個 block 只在一個 SM 上執行**，不跨 SM。所以 grid 開幾個 block，就決定了最多能用幾個 SM（見 §6① 為什麼調 `mps_sm_ul_order` 沒用）。
> 2. **block 之間沒有執行順序保證，也不保證同時常駐**——除非 grid 小到全部能一起放進 GPU。
>
> 第 2 點的推論是：**跨 block 的 busy-wait / barrier 很危險**。**lcore 之間可以互等（有 OS 排程救場），block 不行**——不同時 resident 就是死鎖。這是 GPU 新手最容易踩的坑。
>
> **但本設定啟用的 order kernel 沒有踩這個坑。** `ul_order_kernel_mode: 0`（yaml `:47`）走 ping-pong mode，程式碼註解寫得很清楚：「The ping-pong order kernels use a single CTA to both receive and process the packets」（`order_cuda_kernels.cu:37-38`）。一個 cell 一個 CTA，收包與處理都在同一個 block 內完成，**不依賴其他 cell 的 block 同時 resident**——kernel 本體（`:3204-4092`）內只有 block 內的 `__threadfence_block()`，沒有任何跨 block 同步。
>
> 同一個 `.cu` 檔裡確實有用 `barrier_signal = gridDim.x` 做跨 block barrier 的舊 kernel（`:270`、`:6978`、`:7200`），但那些不是本設定走的分支。**讀 `order_cuda_kernels.cu` 時務必先確認自己看的是哪個 mode。**
>
> 反過來說，`:3199-3202` 的 `__launch_bounds__` 註解正好說明設計上**預期** CTA 可能被延後 launch：「寧願兩個 CTA 擠在一個 SM 上，也不要延後其中一個 CTA」——如果真的依賴同時 resident，就不會這樣寫。

**關鍵直覺**：同一個 warp 內的 32 個 thread 若走不同分支會「發散」（divergence），兩條路徑會**序列化各跑一遍**。所以 GPU code 會盡量讓同一個 warp 做同構的事。

> ⚠️ **但不要把 warp 想成「鎖步」。** Volta（compute capability 7.0）之後導入 Independent Thread Scheduling，**每個 thread 有自己的 program counter**，發散後可交錯執行，也不保證自動重新收斂。GB10 當然屬於這一代。
>
> 「發散有效能懲罰」仍然成立，但「warp 內天然同步」**作為正確性假設是錯的**——warp 內要交換資料一律得用 `__syncwarp()` 或 `*_sync()` 系列。本 repo 就是這樣寫的：`order_cuda_kernels.cu:3595` 用 `__shfl_sync(0xffffffff, pkt_idx, 0, 32)` 帶顯式 mask 廣播 packet index，正是因為不能假設隱含同步。

本 repo 的例子：order kernel 是「**一個 warp 處理一個封包**」——32 個 thread 協作處理同一個封包的不同 PRB，天然同構。

### 3.2 kernel 與 launch

**kernel** = 一個 `__global__` 函式，由 CPU 「發射」到 GPU 執行，發射時指定用多少 block、每 block 多少 thread：

```cpp
// cuPHY-CP/cuphydriver/src/uplink/order_cuda_kernels.cu:5976   ← ul_rx_pkt_tracing_level: 0 走這個分支
order_kernel_doca_single_subSlot_pingpong<false, 0, 0, 320, 2>
    <<< cudaBlocks, ORDER_KERNEL_PINGPONG_NUM_THREADS, 0, stream >>>( ... );
//      ^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//      幾個 block   每 block 幾個 thread (= 320，見 :39)
```

`<<< >>>` 是 CUDA 特有語法。本設定是 **每個 cell 一個 block、每 block 320 threads（= 10 warps）**（`cudaBlocks = num_order_cells`，`:5769`）。

> ⚠️ `:5769` 的註解 `//# of Thread blocks should be twice the number of cells` 描述的**不是這個 mode**。它講的是 `ul_order_kernel_mode: 1`（Dual CTA mode，「block 0 to receive, block 1 to process」），那個分支在 `:6015` 確實用 `cudaBlocks * 2`。註解被放在兩個 mode 共用的變數宣告上，很容易誤讀成本 mode 的行為。

**對 DPDK 使用者最違反直覺的地方**：這個 kernel 不是「處理一批就結束」。它裡面有 `while(1)` 迴圈，**持續執行數百微秒**，一邊收包一邊處理，直到收滿預期 PRB 數或逾時才退出。

CPU 每個 UL slot 發射它一次：

```cpp
// cuPHY-CP/cuphydriver/src/uplink/order_entity.cpp:1387
launch_order_kernel_doca_single_subSlot(first_strm, ...);
```

發射時機是 **slot 邊界前 500 µs**：

```cpp
// cuPHY-CP/cuphydriver/include/constant.hpp:132
static constexpr uint32_t UL_TASK1_ORDER_LAUNCH_OFFSET_FROM_T0_NS = 500000;
///< Order kernel launch: T0 - 500us (PUSCH/PUCCH ordering)
```

概念上等同 DPDK 的 polling loop，差別是：跑在 GPU 上、320 個 thread 同時跑、每個 slot 重啟一次。

### 3.3 stream：GPU 的執行佇列

**stream** 是一串照順序執行的 GPU 工作。不同 stream 之間可並行。

```cpp
// cuPHY-CP/aerial-fh-driver/lib/gpu_comm.cpp:52
cudaStreamCreateWithPriority(&cstream_, cudaStreamNonBlocking, -5);
```

> ⚠️ **`-5` 是「高優先」請求，不是最低。** CUDA stream priority **數字越小越優先**，跟你熟悉的 `nice`、SCHED_FIFO（數字大 = 優先，本文 §0 才剛出現過「SCHED_FIFO 95」）**方向相反**。
>
> 但它是不是這張卡的「最高」，要看 `cudaDeviceGetStreamPriorityRange()` 回傳的 `greatestPriority`——只有查出來等於 `-5` 才能這樣講，超出範圍會被 clamp。而且 priority 只是**排程偏好**：不保證搶占已經在跑的 kernel，也不保證不同 stream 的完成順序。
>
> `cudaStreamNonBlocking` 的意思是「不與 legacy default stream 隱含同步」。CUDA 有一個預設的 NULL stream，預設情況下它會跟其他 stream 產生隱含同步——這是 CUDA 最常見的坑之一，這個旗標就是在關掉那個行為。

把 stream 想成一條 pipeline。下行送包在一條 stream 上依序跑 memset → pre_prepare → prepare；壓縮 kernel 在另一條（cell 的 DL stream）跑。兩條之間用 **event** 同步：

```cpp
// cuPHY-CP/cuphydriver/src/downlink/task_function_dl_aggr.cpp:1338
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

> ⚠️ **上表是傳統「獨立顯卡 + PCIe」的模型。你的平台不完全適用。**
> GB10 是 Grace-Blackwell superchip，**CPU 與 GPU 共用同一組實體 LPDDR5X**，沒有傳統獨立顯卡的專屬 VRAM。
>
> 但**別因此推論「反正都一樣」**：allocation type 仍然決定 CPU、GPU、NIC 三方各自的合法存取方式。device allocator 拿到的記憶體不會因為實體共享就自動變成 NIC 可 coherent 存取——這正是 Spark 不支援 GPUDirect RDMA 的原因（見 §9）。
> 這正是後面 §9 說「接收方向換成 CPU pinned memory **不會多一次複製**」的原因——在獨立顯卡上那會是跨 PCIe 的搬移，在 GB10 上不是。
> 先用上表建立「為什麼要 pin」的直覺就好，但別把它當成 GB10 的物理事實。

DOCA 的兩種型別，在本 repo 的分岔點：

```cpp
// cuPHY-CP/aerial-fh-driver/lib/doca_obj.cpp:154-160
if(mtype == DOCA_GPU_MEM_TYPE_CPU_GPU){
    doca_gpu_mem_alloc(..., DOCA_GPU_MEM_TYPE_CPU_GPU, &gpu_pkt_addr, &cpu_pkt_addr);
} else {
    doca_gpu_mem_alloc(..., DOCA_GPU_MEM_TYPE_GPU, &gpu_pkt_addr, NULL);
}

// cuPHY-CP/aerial-fh-driver/lib/doca_obj.cpp:167-197
if (!enable_gpu_comm_via_cpu) {
    doca_gpu_dmabuf_fd(...);                              // 先試 dmabuf
    // 失敗則退回 nvidia-peermem: doca_mmap_set_memrange(gpu_pkt_addr, ...)
} else {
    doca_mmap_set_memrange(mmap, item->cpu_pkt_addr, ...); // :192  用 CPU 位址註冊
}
```

**`DOCA_GPU_MEM_TYPE_GPU` 要求把 GPU 記憶體的位址註冊給網卡做 DMA — 這就是 GPUDirect RDMA**，需要 GPU 透過 PCIe BAR 把記憶體暴露出來，並靠 dmabuf 或 `nvidia-peermem` 核心模組建立映射。

### 3.5 MPS 與 SM 配額 — 你的 `mps_sm_*` 設定

沒有 MPS 時，不同 process 的 CUDA context 是靠 GPU 的 context switching / time-slicing 輪流上場——不是「一個在跑、其他全部乾等」，但跨 process 的 kernel 也很難真正重疊。**MPS（Multi-Process Service）** 讓多個 client 共用執行資源，跨 process 的工作才能並行，並可指定**每個 client context 最多用幾個 SM**。

Aerial 為每個功能建一個 context：

```cpp
// cuPHY-CP/cuphydriver/src/common/context.cpp:757-800（非 green-context 分支）
puschMpsCtx    = new MpsCtx(..., getMpsSmPusch());      // mps_sm_pusch: 40
pdschMpsCtx    = new MpsCtx(..., getMpsSmPdsch());      // mps_sm_pdsch: 46
ulMpsCtx       = new MpsCtx(..., getMpsSmUlOrder());    // mps_sm_ul_order: 12
gpuCommsMpsCtx = new MpsCtx(..., getMpsSmGpuComms());   // mps_sm_gpu_comms: 16
```

**這是 GPU 版的資源限額——但類比要小心方向。**

不是 `isolcpus` 那種**互斥切分**（切給你的別人拿不到），而更像 **cgroup 的 `cpu.max` quota**：設的是**上限**，不是保留。NVIDIA MPS 文件明講：

> "Setting the limit **does not reserve dedicated resources** for any MPS client context."

也就是說 `mps_sm_pusch: 40` 只保證 PUSCH context **最多**用 40 個 SM，**不保證** UL order 一定拿得到它的 12 個。它防的是「單一 kernel 吃光整顆 GPU」，不是保證誰有多少。所以配額不是 SLA——PDSCH 的大 kernel 仍可能把 order kernel 排在後面。

（另註：官方說 SM count 會被「internally **rounded up** to the nearest hardware supported SM count limit」，填 12 實際不一定剛好 12。）

實作（`cuPHY-CP/cuphydriver/src/common/mps.cpp:47-67`）：

```cpp
cuDeviceGetAttribute(&actualDevSmCount, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, cuDev);
if (actualDevSmCount < devSmCount) {                                            // :50
    throw std::runtime_error("Requested N SMs in cuCtxCreate_v3() but GPU has max M SMs.");
}
CUexecAffinityParam affinityPrm;
affinityPrm.type = CU_EXEC_AFFINITY_TYPE_SM_COUNT;                              // :58
affinityPrm.param.smCount.val = devSmCount;                                     // :59
cuCtxCreate_v3(&cuCtx, &affinityPrm, 1, CU_CTX_SCHED_SPIN|CU_CTX_MAP_HOST, cuDev); // :67  ← CUDA<13 分支
```

**兩個重要釐清**：

1. **檢查的是單一 context 的值，不是所有 `mps_sm_*` 的總和。** 實際建立的 context 加起來遠大於實體 SM 數 — 這是**正常的**（見上：配額是上限不是保留）。若啟動時 throw，是某**單一**值超過 GPU 的 SM 總數。

   > 順帶一提，yaml 的 `mps_sm_*` 與實際建立的 context **不是一對一**：
   > - `mps_sm_pdcch` 與 `mps_sm_pbch` 不各自建 context，而是合併成 `dlCtrlMpsCtx`（`context.cpp:154, :771`）
   > - **`mps_sm_pbch` 的 yaml 值被靜默忽略**——`context.cpp:153` 是 `mps_sm_pbch = ctx_cfg.mps_sm_pdcch;`，用的是 pdcch 的值。所以本設定的 `mps_sm_pbch: 4` 完全沒作用，`dl_ctrl` 實際是 `12 + 12 = 24`

2. `mps.cpp:59` 的註解直說：SM 數合法時若建 context 仍回 **CUDA error 224**，那只可能是 **MPS daemon 沒在跑**。

   > ⚠️ **這是註解作者對自身環境的判斷，不是 CUDA API 的語意保證。** 224 一般對應 execution affinity 不受支援（`CUDA_ERROR_UNSUPPORTED_EXEC_AFFINITY`——數值請在 container 內用 `grep -rn UNSUPPORTED_EXEC_AFFINITY /usr/local/cuda/include/cuda.h` 自行確認）。MPS daemon 沒跑是本部署最常見的成因，但 device/driver 是否支援 execution affinity、CUDA 版本組合也可能觸發它。**別憑 224 就排除其他可能。**

3. **實際呼叫的 API 依 CUDA 版本而定**：上面貼的 `cuCtxCreate_v3` 是 `CUDA_VERSION < 13000` 的分支。本專案 container 基底是 `cuda:13.1.1`（`cuPHY-CP/container/aerial_base_recipe.py:38`），所以實際走 `CUctxCreateParams` + **`cuCtxCreate()`**（`mps.cpp:56-68`）。`mps.cpp:52` 的例外訊息字串仍寫死 `cuCtxCreate_v3()`，那是沒同步更新的訊息。

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
    ↓  比中 → QUEUE action；沒比中 → rte_flow_isolate 硬體丟棄（nic.cpp:860）
DOCA cyclic packet buffer（DOCA_ETH_RXQ_TYPE_CYCLIC）        ← 取代 mbuf pool
    doca_obj.cpp:126, 154-197
    ↓
GPU order kernel 呼叫 doca_gpu_dev_eth_rxq_recv()            ← 取代 rte_eth_rx_burst()
    order_cuda_kernels.cu:3984
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

**落點計算**（`cuPHY-CP/aerial-fh-driver/include/aerial-fh-driver/oran.hpp:1633-1638`）：

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
| **U-plane (DL IQ)** | **GPU kernel 寫 WQE** | GPU 直接構造 mlx5 的 Work Queue Entry；**但本平台的門鈴是 CPU 敲的**，見 §9 |

**WQE 與 doorbell** 是 DPDK 幫你隱藏掉的底層：`rte_eth_tx_burst()` 內部就是在做「填 WQE + 寫 doorbell 暫存器」。GPUNetIO 把這兩步搬到 GPU 上做（或部分做）。

隔離是強制的 — `Txq::send()` 一旦發現自己是 GPU queue 就直接 throw：

```cpp
// cuPHY-CP/aerial-fh-driver/lib/queue.cpp:114-117
if(is_gpu())
    THROW_FH(ENOTSUP, ... << " because it's a GPU-init comm queue");
```

---

## 6. FlexRAN 直覺會誤導你的六件事

### ① 「加 CPU core 就能解決收包問題」— 不適用，但也**不是調 `mps_sm_ul_order`**

上行收包沒有 CPU core 參與，所以核心配置動不了它。但別急著去調 `mps_sm_ul_order` ——**那個旋鈕在本設定下已經飽和了**：

- order kernel 的 grid 是 `cudaBlocks = num_order_cells`（`order_cuda_kernels.cu:5769`），kernel 內 `int cell_idx = blockIdx.x;`（`:3306`）→ **一個 cell 一個 block**
- **一個 block 只能在一個 SM 上執行**（CUDA 硬性規則，不跨 SM）
- 本設定 `cell_group_num: 1` → 1 個 cell → **1 個 block → 最多占 1 個 SM**

`mps_sm_ul_order: 12` 遠遠超過實際需求，**調大它不會有任何效果**。

真正決定 order kernel 吞吐的是：
- **每 block 的 warp 數**：320 threads = 10 warps → 同時處理 10 個封包（一個 warp 一包）
- **cell 數**：多 cell 才會用到多個 SM

所以收包吃緊時該看的是 `[RX Packet Times]` 的 LATE 計數、`ul_order_max_rx_pkts` / `ul_order_rx_pkts_timeout_ns` 的收斂行為，以及 RU 側的送出時序——不是 SM 配額。

### ② 「DPDK 統計能告訴我收了多少包」— 不能

要分清楚兩種計數：

- **`rte_eth_stats_get()` 與 per-queue 計數**：mlx5 是把各個 PMD rxq 的**軟體**計數加總。external RxQ 沒有 PMD 的 rxq 控制結構，所以**看不到** GPU queue 的封包。
- **`rte_eth_xstats_get()`**：裡面有 **device 層級的硬體計數器**（`rx_vport_unicast_packets` 等），這些是網卡自己數的，**會**把 GPU queue 的封包算進去。

所以「DPDK 完全看不到」是不準確的說法——**軟體 per-queue 計數看不到，硬體計數器看得到**。這也是 [`DPDK_ADVANCED.md`](DPDK_ADVANCED.md) §5 那個檢查順序第 2 步能成立的原因。

要看實際收包狀況，看 order kernel 自己維護的計數：

```
[RX Packet Times] { EARLY: n ONTIME: n LATE: n }
```
（`cuPHY-CP/cuphydriver/src/uplink/slot_map_ul.cpp:619`）

或直接讀網卡硬體計數器。

### ③ 「封包丟了會有 log」— 很多時候不會

三個靜默丟包/錯置點：

1. **`rte_flow` 沒比中** → 硬體直接丟，任何軟體層都看不到。規則是 full-mask 比對 MAC + VLAN TCI（**含 PCP**）+ eAxC ID，任一 bit 不對就完全不匹配。

   > ⚠️ **但有一條 fallback 會讓 eAxC 不再被硬體過濾。** `peer.cpp:609-613`：若 `rte_flow_create()` 失敗且錯誤訊息含 `eCPRI`，程式會印
   > ```
   > eCPRI parser not supported on NIC {}, retrying without eCPRI
   > ```
   > 然後把 eCPRI pattern 換成 `RTE_FLOW_ITEM_TYPE_END` 再建一次——**這時規則只剩 ETH + VLAN**。看到這行 log 就代表未知 eAxC 的封包不會在 NIC 被擋下，會進 GPU queue，然後在下面第 2 點被靜靜映射成天線 0。除錯時要改查 GPU 端的 eAxC mapping，而不是繼續盯 flow rule。
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
- **kernel launch 開銷** → 所以有 CUDA graph（`enable_ul_cuphy_graphs: 1`）：PUSCH、PUCCH、PRACH、SRS 各自改用 graph processing mode（`phypusch_aggr.cpp:745`、`phypucch_aggr.cpp:328`、`phyprach_aggr.cpp:479`、`physrs_aggr.cpp:537`），把各 channel pipeline 內的 kernel 序列預錄後 replay。
  > ⚠️ **這不代表整個 slot 錄成同一張 graph。** order receive kernel 不在裡面，它仍由 `order_entity.cpp:1387` 每個 slot 單獨 launch。
- **SM 爭用** → 所以有 MPS 配額
- **kernel 逾時** → `ul_order_timeout_gpu_ns: 3000000`（3 ms）

### ⑤ 「API 回錯誤碼我就知道出事了」— CUDA 錯誤是**非同步**的

這是 DPDK 背景的人最大的認知落差，也最會實際害到你。

DPDK 每個 API 當場回傳錯誤碼。CUDA 不是：

- **kernel launch 立刻回傳**，不等它跑完。kernel 內部出的錯要到**下一個同步點**（`cudaStreamSynchronize`、`cudaMemcpy`、event query…）才浮出來
- 因此錯誤常常被歸咎到一個**毫不相干的後續 API 呼叫**上
- **kernel 內沒有例外機制**，出錯時也不會在那一行以 CPU `SIGSEGV` 的形式停下來。有些越界只是安靜地寫壞別人的記憶體；有些會讓 kernel／context 進入 illegal-address 狀態，但要到後續的同步點（`cudaStreamSynchronize`、event query、下一個 CUDA API 呼叫）才非同步回報

實務上：
- `cudaGetLastError()` 在 launch 之後立刻呼叫，抓 launch 參數錯誤（如 block size 超限）
- 除錯時設環境變數 `CUDA_LAUNCH_BLOCKING=1` 強制同步，錯誤才會出現在正確的位置
- 記憶體錯誤用 `compute-sanitizer`（GPU 版的 valgrind/ASAN，舊名 `cuda-memcheck`）

這一點跟 §6③-2 是同一件事的兩面：`get_eaxc_index()` 查不到回傳 0，封包被寫進天線 0 的位置——**不會有任何錯誤，只有結果不對**。

而且**這種錯連 `compute-sanitizer` 也抓不到**：天線 0 是完全合法的 in-bounds 存取，工具無從得知它在業務語意上是錯的。要抓只能靠 not-found sentinel、device 端的 error counter/assert、針對未知 eAxC 的單元測試，或直接檢查輸出 tensor 各天線平面的內容。

### ⑥ 「gdb / perf 能看到問題」— 工具換了

見第 7 節。特別注意：`cuda-gdb` 對這種 persistent kernel + 500 µs 的 slot 節奏的即時系統幾乎不能用——設中斷點等於把整條 pipeline 打死。實務上只能靠 Nsight Systems 的時間軸加上 kernel 自己維護的計數器。

---

## 7. 工具對照

| 你熟悉的 | GPU 對應 | 用途 |
|---|---|---|
| `perf` / `top` | **Nsight Systems**（`nsys`） | 時間軸：kernel 何時跑、跑多久、誰在等誰 |
| `perf annotate` | **Nsight Compute**（`ncu`） | 單一 kernel 內部的瓶頸分析 |
| `gdb` | **`cuda-gdb`** | 進 kernel 設中斷點。**對本專案幾乎不能用**——persistent kernel + 500 µs 的 slot 節奏，斷點=打死 pipeline |
| `valgrind` / ASAN | **`compute-sanitizer`** | 抓越界、race、未初始化等**記憶體正確性**問題。舊名 `cuda-memcheck`。⚠️ 抓不到「index 合法但業務映射錯誤」——寫進天線 0 是 in-bounds 存取，見 §6③-2 |
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
| **shared memory** | block 內所有 thread 共用的高速 scratchpad，位於 SM 上，比 global memory 快一個量級 |
| **register** | thread 私有的最快儲存 |
| **grid** | 一次 launch 的所有 block 的總稱 |
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

> ⚠️ **文件版本與實際版本不一致。** 上面引的是 DOCA 3.4 archive（用來佐證平台限制），但 Aerial 26.1 的 DGX Spark software manifest 列的是 **DOCA 3.2.1**（included in cuBB container）、CUDA Toolkit 13.1.1、DPDK 22.11、**GDRCopy N/A**（`5GModel/aerial-cuda-accelerated-ran.pdf` PDF 實體第 18 頁，文件印刷頁碼 14）。平台限制的敘述兩版一致，但 **API 行為、錯誤碼與 sample 細節請以 container 內實際的 headers/libraries 為準**。

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

1. **沒開這個旗標會怎樣**：DOCA 會走 Spark 不支援的 GPU-memory / GPUDirect RDMA 路徑，queue 或 memory registration 失敗，程式 FATAL exit。這不是 bug，是平台限制。

   > **本機實測**（2026-07-27；DOCA 3.2.1025 / ConnectX-7 `15b3:1021` / driver 590.48.01 / CUDA 13.1）：底層 MR 註冊失敗回 **EFAULT (`errno 14`)**——`Failed to register user memory. Got errno=UNKNOWN-errno14 (14)`，出自 DOCA 內部的 `linux_devx_adapter.cpp:247`（**不在本 repo**）——連鎖成 `doca_mmap_start()` 失敗 → `Failed to setup DOCA GPU RxQ #0 on NIC ...` → `l1_init` 丟 NIC registration error → FATAL exit。
   >
   > 確切的失敗 API 與錯誤碼會隨 DOCA/driver 版本而異，**請以自己的 runtime log 為準**，不要把這個組合當成跨版本的必然結果。除錯時要往前找**第一個** error，不能只看最後 throw 的那行。

2. **收送兩個方向的代價不對稱**：
   - **接收**：只是換記憶體位置，**不多一次複製**
   - **發送**：每個 symbol 多一趟 GPU→CPU 的 D2H 搬移（`gpu_comm.cpp:236-278`），且 doorbell 由 CPU 敲

3. **抄範例要小心**：網路上的 GPUNetIO 範例多半假設有獨立 GPU + GPUDirect RDMA；在 DGX Spark 上直接照抄，可能走入不支援的 GPU-memory registration 路徑而啟動失敗。實際失敗 API 與錯誤碼依軟體版本而異。

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
- [`DPDK_ADVANCED.md`](DPDK_ADVANCED.md) — `rte_flow`、`SEND_ON_TIMESTAMP`、mlx5 PMD（與 GPU 無關但 Aerial 必用）
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
