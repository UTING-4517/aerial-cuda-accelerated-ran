# Aerial cuBB 發送路徑：從 NIC 初始化到 C-plane / U-plane 送出

**基準設定**：`cuPHY-CP/cuphycontroller/config/cuphycontroller_F08_WNC_DGX.yaml`（DGX Spark / GB10、整合式 ConnectX-7 @ `0000:01:00.0`）

關鍵旗標：`gpu_init_comms_dl: 1`、`gpu_init_comms_via_cpu: 1`、`cpu_init_comms: 0`、`mMIMO_enable: 0`、`cell_group_num: 1`、`disable_empw: 0`、`ru_type: 1`(SINGLE_SECT_MODE)

> 本文所有行號都經過實際讀取原始碼確認。無法從本 repo 確認的內容集中在最後一節「未確認事項」，不混在正文裡。

> **路徑慣例**：文中所有 `檔案:行號` 都是**相對於 repo 根目錄**（本文件位於 `notes/`，往上一層）。
>
> **前置閱讀**：若不熟悉 GPU 概念（kernel、stream、SM、pinned memory），先看 [`GPUNETIO_INTRO.md`](GPUNETIO_INTRO.md)。

---

## 0. 一句話總覽

**C-plane 由 CPU 用 DPDK `rte_eth_tx_burst` 送；U-plane 下行 IQ 由 GPU kernel 組成 WQE，但門鈴（doorbell）在本設定下是 CPU 敲的。**

| 平面 | 方向 | 組包者 | 送出機制 | Queue |
|---|---|---|---|---|
| C-plane | DU→RU | CPU | `rte_eth_tx_burst`（`queue.cpp:122`） | CPU DPDK TXQ × **2** |
| U-plane | DU→RU（DL IQ） | GPU kernel | GPU 寫 WQE + **CPU proxy 敲 doorbell** | DOCA GPU TXQ × **1** |
| U-plane | RU→DU（UL IQ） | GPU kernel 直接收 | `doca_gpu_dev_eth_rxq_recv` | DOCA GPU RxQ × **2**（含 SRS） |
| C-plane | RU→DU | — | **不存在**，DU 不收 C-plane | — |

分工的決定點只有一處，`cuPHY-CP/cuphydriver/src/common/fh.cpp:186-198`：

```cpp
if(pdctx->gpuCommDlEnabled()) {        // gpu_init_comms_dl: 1
    txq_gpu = cfg.txq_count_uplane;      // U-plane → GPU queue
    txq_cpu = cfg.txq_count_cplane;      // C-plane → CPU queue
}
if(pdctx->cpuCommEnabled()){           // cpu_init_comms: 0（未啟用）
    txq_gpu = 0;
    txq_cpu = cfg.txq_count_uplane + cfg.txq_count_cplane;   // 全部退回 CPU
}
```

隔離是強制的：`Txq::send()` 一旦發現自己是 GPU queue 就直接 `THROW_FH(ENOTSUP, ...)`（`queue.cpp:114-117`）。CPU 在程式碼層面不可能從 GPU queue 送包，反之亦然。

---

## Part 1：初始化 — 從程式進入點到兩種 TXQ 就緒

### 1.1 Queue 數量在哪裡決定

不是 yaml 直接指定的，是算出來的（`cuPHY-CP/cuphycontroller/examples/cuphycontroller_scf.cpp:421-441`）：

```cpp
auto txq_count_uplane = ctx_cfg.cell_group_num;   // :421  U-plane
auto txq_count_cplane = ctx_cfg.cell_group_num;   // :422  DL C-plane
txq_count_cplane     += ctx_cfg.cell_group_num;   // :423  UL C-plane
if (ctx_cfg.mMIMO_enable) {                       // :425  ← 本設定為 0，整段跳過
    if (dlc_alloc_cplane_bfw_txq || dlc_bfw_enable_divide_per_cell) txq_count_cplane += cell_group_num;
    if (ulc_alloc_cplane_bfw_txq || ulc_bfw_enable_divide_per_cell) txq_count_cplane += cell_group_num;
}
auto rxq_count = ctx_cfg.cell_group_num;                        // :437
if (ctx_cfg.enable_srs) rxq_count += ctx_cfg.cell_group_num;    // :439-441
```

本設定（`cell_group_num: 1`、`mMIMO_enable: 0`、`enable_srs: 1`）：

- **CPU TXQ = 2**（DL C-plane 1 + UL C-plane 1）
- **GPU TXQ = 1**
- **GPU RXQ = 2**（一般 + SRS）
- `rte_eth_dev_configure()` 宣告的 TXQ 總數 = 2 + 1 = **3**（`nic.cpp:370, 386, 390`）

### 1.2 `Nic` 建構順序（`cuPHY-CP/aerial-fh-driver/lib/nic.cpp:46-102`）

```
55   validate_input()                    nic.cpp:275
56   doca_probe_device()                 nic.cpp:204   ← DOCA 開卡、偵測 CX6、devargs、取 port_id
57   validate_driver()                   nic.cpp:1010
58   configure()                         nic.cpp:366   ← rte_eth_dev_configure(rxq, txq_cpu + txq_gpu)
59-64 set_pcie_max_read_request_size()   nic.cpp:819   ← MRRS = 4096（僅 mlx5）
     disable_ethernet_flow_control()     nic.cpp:1045
66   set_mtu()                           nic.cpp:404
67   restrict_ingress_traffic()          nic.cpp:854   ← rte_flow_isolate(port, 1)
68   create_cpu_mbuf_pool()              nic.cpp:458
74   create_tx_request_uplane_pool()     nic.cpp:558
75   create_tx_request_cplane_pool()     nic.cpp:576
76   setup_tx_queues()                   nic.cpp:417   ★ CPU TXQ（rte_eth_tx_queue_setup）
78   setup_rx_queues()                   nic.cpp:443
79   start()                             nic.cpp:1067  ← rte_eth_dev_start()  (:1088)
80   setup_tx_queues_gpu()               nic.cpp:430   ★ GPU TXQ（doca_create_tx_queue）
81   check_physical_link_status()        nic.cpp:594
83   warm_up_txqs()                      nic.cpp:1095  ← 只 warm up CPU TXQ
91-94 GpuComm 建立 + gpu_comm_init_tx_queues()        nic.cpp:1278
95-98 (CX6 only) set_qp_clock_id()       nic.cpp:968
99-100 set_flow_comm_buf()               nic.cpp:983   ★ doca_create_tx_buf（GPU 封包緩衝）
```

**注意順序**：CPU TXQ 在 `rte_eth_dev_start()` **之前**（DPDK 規定），GPU TXQ 在**之後**。`configure()` 已把 CPU+GPU 的總數一次宣告給 port，所以 GPU 佔用的 queue index 是預留好的。

GPU TXQ 不走 DPDK ethdev，而是 `doca_create_tx_queue`（`queue.cpp:80/84` → `doca_obj.cpp:601-697`）自建 SQ 並自行 `doca_ctx_start`（`doca_obj.cpp:684`）。

### 1.3 TX 側資源配置

| 資源 | 函式 | 大小 | 記憶體位置 |
|---|---|---|---|
| CPU mbuf pool | `nic.cpp:458-533` | `rte_align32pow2(196608)-1` = **262143** 個 × **1664 B** droom | hugepage 主記憶體 |
| U-plane TX request | `nic.cpp:558-574` | `rte_align32pow2(64)-1` = **63** 個 `TxRequestUplane` | hugepage |
| C-plane TX request | `nic.cpp:576-592` | **63** 個 `TxRequestCplane` | hugepage |
| GPU 封包緩衝 | `nic.cpp:983-1003` → `doca_obj.cpp:454-598` | `API_MAX_NUM_CELLS × MAX_DL_EAXCIDS × 2048` 包 × 2048 B | **GPU VRAM**（via_cpu 時另配一份 CPU-visible 副本） |

droom 計算：`RTE_ALIGN_MUL_CEIL(1500 + 128 + 14 + 4, 128)` = 1664（`nic.cpp:460`，`kMbufPoolDroomSzAlign=128` 在 `defaults.hpp:28`）

mbuf 建立時就預先設好 `shinfo` 指向 priv 區、`buf_iova = RTE_BAD_IOVA`（`nic.cpp:485-491`）——這是為了 external buffer（IQ 資料直接指向外部記憶體）的 zero-copy。

### 1.4 Peer 與 Flow 的角色

**`Peer` = 每個 O-RU 的 TXQ 擁有者與封包組裝者**（`peer.cpp:39-77`）

TXQ 分派全在 `Peer::request_nic_resources()`（`peer.cpp:165-252`）：

| 用途 | 取得方式 | 行號 |
|---|---|---|
| U-plane（GPU） | `assign_txq(true)` × `txq_count_uplane_gpu` | `peer.cpp:202-211` |
| DL C-plane | `assign_txq(false)` → `txq_dl_cplane_` | `peer.cpp:210-216` |
| UL C-plane | `assign_txq(false)` → `txq_ul_cplane_` | `peer.cpp:216-222` |
| BFW C-plane | 被 `if(info_.txq_bfw_cplane)` 包住，`mMIMO_enable: 0` → **整段跳過**，維持 `nullptr` | `peer.cpp:221-251` |

`QueueManager`（`queue_manager.cpp`）本身很薄，只是兩條獨立的 free list（CPU 一條、GPU 一條，`:31-40`），`assign_txq()` 從尾端 pop（`:56-79`）。用途語意完全由 `Peer` 決定。

**`Flow` = 每個 eAxC × {C-plane, U-plane} 的 header template 持有者**

```cpp
// flow.hpp:45-50
struct PacketHeaderTemplate {
    rte_ether_hdr  eth;
    rte_vlan_hdr   vlan;
    oran_ecpri_hdr ecpri;
} __attribute__((packed));
```

建立於 `Flow::setup_packet_header_template()`（`flow.cpp:215-298`），啟動時建一次：
- src/dst MAC（`:229-230`）、outer EtherType=VLAN（`:232`）、VLAN TCI（`:233`）、inner EtherType=eCPRI（`:234`）
- eCPRI message type：C-plane → `RTE_ECPRI_MSG_TYPE_RTC_CTRL`，U-plane → `IQ_DATA`（`:239-241`）
- `ecpriPcid = eAxC`（`:242`）、`ecpriEbit = 1`（`:245`）

DL U-plane 另外做 GPU 版：`Flow::setup_packet_header_gpu()`（`flow.cpp:315-429`）在 host 上把 **2048 份**完整 header 預先展開（`kMaxPktsFlow=2048`，`defaults.hpp:65`），一次 `cudaMemcpy` 上 GPU（`:400`），之後常駐。

`Flow` **不持有 TXQ**——`Flow::request_nic_resources()`（`flow.cpp:131-151`）只要 RXQ。發送一律用 Peer 的 queue。

---

## Part 2：C-plane 發送（CPU / DPDK 路徑）

### 2.1 呼叫鏈

```
Worker task
 ├─ DL: task_work_function_cplane             downlink/task_function_dl_aggr.cpp:919
 └─ UL: task_work_function_ul_aggr_1_cplane   uplink/task_function_ul_aggr.cpp:1310
      └─ FhProxy::prepareCPlaneInfo           common/fh.cpp:1307
           ├─ sendCPlane_timingCheck          common/fh.cpp:1329 → 935
           ├─ 組 fhproxy_cmsg message_infos[]  common/fh.cpp:2197-2547  ← 非 mMIMO 分支
           └─ aerial_fh::send_cplane          common/fh.cpp:2549
                └─ Peer::send_cplane          aerial-fh-driver/lib/peer.cpp:3593
                     ├─ count_cplane_packets       peer.cpp:3601 → 3392
                     ├─ rte_mempool_get_bulk       peer.cpp:3611
                     ├─ prepare_cplane_message ×N  peer.cpp:3619 → 2878
                     └─ send_cplane_enqueue_nic    peer.cpp:3651 → 3580
                          └─ Txq::send            queue.cpp:108
                               └─ rte_eth_tx_burst  queue.cpp:122
```

> `mMIMO_enable: 0` → `FhProxy::sendCPlaneMMIMO`（`fh.cpp:1254`）與整條 `send_cplane_mmimo` 路徑**永不執行**。

### 2.2 觸發時間點

| | 註冊位置 | task 名 | 執行時間 |
|---|---|---|---|
| DL | `cuphydriver_api.cpp:1820` | `TaskDL1AggrCplane<N>` | `tick_original`（**slot 邊界當下**，`:1582, 1661-1662`） |
| UL | `cuphydriver_api.cpp:1402` | `TaskUL1AggrCplane<N>` | `tick_original + 1×TTI`（μ=1 → **+500 µs**，`:1271, 1287`） |

UL task 因為排在一個 slot 之後，內部用 `exec_slot_ahead = getSlotAhead() - 1` 扣回來（`task_function_ul_aggr.cpp:1397-1398`，該處有註解說明）。

### 2.3 封包組裝（`Peer::prepare_cplane_message`，`peer.cpp:2878-3390`）

```
peer.cpp:2897-2898   memcpy(data, &flow->get_packet_header_template(), sizeof(PacketHeaderTemplate))
peer.cpp:2900-2901   memcpy(common_hdr_ptr, &info.section_common_hdr, common_hdr_size)   ← O-RAN radio app hdr
peer.cpp:2942        memcpy(section_ptr, &info.sections[n], section_size)                ← 逐 section
                     （:2905 是 section_ptr 的位址計算，不是 memcpy）
peer.cpp:2917        塞不下就開新 mbuf（room = mtu - ORAN_CMSG_HDR_OFFSET - common_hdr_size, :2890）
peer.cpp:2947-2951   設 data_len/pkt_len、ecpriSeqid（per-Flow 遞增）、ecpriPayload
```

mbuf 來自 `nic_->get_cpu_mbuf_pool()`（`peer.cpp:3609`），一次 `rte_mempool_get_bulk`（`:3611`）。單次 burst 上限 `kTxPktBurstCplane = 1024`（`defaults.hpp:30`，檢查在 `peer.cpp:3603`）。

### 2.4 Section Type 1 / Type 3 欄位來源（`common/fh.cpp:2197-2553`）

**Section Type 1**（一般通道，`fh.cpp:2269-2278`）

| 欄位 | 來源 | 行號 |
|---|---|---|
| `startPrbc` | `prb_info.common.startPrbc` | `fh.cpp:2247` |
| `numPrbc` | `adjustPrbCount(...)`（=273 → 0 表 all-PRB；>255 → 截 255） | `fh.cpp:2248`，函式 `:864-878` |
| `reMask` / `numSymbol` | `prb_info.common.*` | `fh.cpp:2249-2250` |
| `sectionId` | SRS → `section_id_srs++`；其他 → `section_id++` | `fh.cpp:2272` |
| `beamId` | `beams_array[ap_index / beam_repeat_interval]` | `fh.cpp:2417-2436, 2502` |
| `udCompHdr` | **固定 0**（nvbug 4189837，`fh.cpp:1376-1381` 的 BFP 設定被註解掉（初始化為 0 在 `:1371`）） | `fh.cpp:2276` |

PRB > 255 會切段（`fh.cpp:2303-2315`）。位元佈局在 `oran.hpp:722-793`。

**Section Type 3**（PRACH 專用，`fh.cpp:2254-2268`）

| 欄位 | 來源 | 行號 |
|---|---|---|
| `timeOffset` | `cell_ptr->getSection3TimeOffset()`（yaml `section_3_time_offset: 484`） | `fh.cpp:2259` |
| `frameStructure` | `oran_fft << 4 \| prachparams->mu` | `task_function_ul_aggr.cpp:1404-1411` |
| `cpLength` | DL/UL 呼叫端**都傳 0** | `fh.cpp:2261` |
| `sectionId` | `section_id_prach++` | `fh.cpp:2265` |
| `freqOffset` | `prb_info.common.freqOffset`（L2 adapter 由 PRACH root sequence 填） | `fh.cpp:2266` |

**sectionId 空間分割**（`fh.cpp:1372-1374`）：

```cpp
uint16_t section_id       = 0;                      // [0, 2048)     一般
uint16_t section_id_srs   = start_section_id_srs;   // [3072, 4096)  SRS
uint16_t section_id_prach = start_section_id_prach; // [2048, 3072)  PRACH
```

每個 slot 結束時驗證未越界（`fh.cpp:2543-2547`），撞號會印 `"At least two sections have the same SectionId value"` 並回 `SEND_CPLANE_FUNC_ERROR`。

> ⚠️ **這個檢查只在上行執行**：
>
> ```cpp
> // fh.cpp:2543
> if (direction == DIRECTION_UPLINK && ((section_id >= start_section_id_prach) || ...))
> ```
>
> DL C-plane 走同一個 `prepareCPlaneInfo`，但因 `direction == DIRECTION_DOWNLINK` 被短路跳過——**DL 的 sectionId 撞進 PRACH/SRS 區段不會有任何 log**。
> 另外三個子條件裡也沒有 `section_id_srs` 的上界檢查（`< 4096`），所以 `[3072, 4096)` 的上界在程式碼層面是無保護的。
>
> 順帶一提，這三個範圍**完全由 yaml 決定**（`start_section_id_prach` / `start_section_id_srs`），不是程式碼寫死的常數；12-bit 的上限來自 `oran.hpp:735` 的 `Bitfield<uint32_t,·,12> sectionId`。

> 這正是上行接收端 order kernel 用來分辨 PUSCH / PRACH 的依據——收端沒有其他區分方式。

### 2.5 送出時序

**目標送出時間 `start_tx`**

DL（`task_function_dl_aggr.cpp:1034`）：
```cpp
start_tx = getTaskTsExec(0) + TTI_ns(mu) * slotAhead - (Tcp_adv_dl_ns + T1a_max_up_ns);
```
本設定 `125000 + 339000 = 464000 ns`，**恰好等於** yaml 的 `T1a_max_cp_dl_ns: 464000`。

注意這個等式**沒有任何程式碼在檢查**——`T1a_max_cp_dl_ns` 並不參與 DL 送出時間的計算，是要求使用者手動維持 `Tcp_adv_dl + T1a_max_up == T1a_max_cp_dl` 的一致性。

> `T1a_max_cp_dl_ns` 本身**有消費點**，只是在本設定下不可達：
> - `task_function_dl_aggr.cpp:1043`——與 `T1a_min_cp_dl_ns` 在同一個三元運算式裡，被 `getDlcBfwEnableDividePerCell()` 擋住（yaml 為 0）
> - `cuphydriver_api.cpp:97`——跨 cell 一致性檢查，被 `:83` 的 `mMIMO_enable == 1` 包住

UL（`task_function_ul_aggr.cpp:1399`）：
```cpp
start_tx = getTaskTsExec(0) + TTI_ns(mu) * (slotAhead - 1) - T1a_max_cp_ul_ns;
```
`getTaskTsExec(0)` 本身含 +1 slot，兩者抵消 → `start_tx = 空中 slot 起點 − 392 µs`。**`T1a_max_cp_ul_ns` 是直接消費點。**

**逐 symbol 推進**（`fh.cpp:1369-1370, 2205-2206, 2528-2529`）：每 symbol 推進 `SYMBOL_DURATION_NS = 35714`（`doca_structs.hpp:33`）。
`ru_type: 1` = SINGLE_SECT_MODE 時 DL 只送一個 symbol 的 C-plane 就 `break`（`fh.cpp:2531-2534`）。

**時間戳寫入 mbuf**（`peer.cpp:2908, 2953-2958`）：

```cpp
auto& last_packet_ts = (info.data_direction == DIRECTION_DOWNLINK) ? last_dl_cplane_tx_ts_ : last_ul_cplane_tx_ts_;
if(info.tx_window.tx_window_start > last_packet_ts) {
    mbufs[0]->ol_flags = fhi->get_timestamp_mask_();
    *RTE_MBUF_DYNFIELD(mbufs[0], fhi->get_timestamp_offset(), uint64_t*) = info.tx_window.tx_window_start;
    last_packet_ts = info.tx_window.tx_window_start;
}
```

只給每個 message 的**第一個 mbuf** 打時間戳；`last_*_cplane_tx_ts_` 是 Peer 成員（`peer.hpp:668-669`），確保單調不倒退。其餘 mbuf 顯式清 `ol_flags = 0`（`peer.cpp:2896, 2927`）。

dynfield/dynflag 註冊在 `Fronthaul::setup_accurate_send_scheduling()`（`fronthaul.cpp:345-378`）。

### 2.6 三個送出多載走哪一個

`Peer::send_cplane_enqueue_nic`（`peer.cpp:3580-3590`）：

```cpp
if(!(get_fronthaul()->get_info().cuda_device_ids.empty()))  txq->send(&mbufs[0], num_packets);      // :3584 無鎖
else                                                        txq->send_lock(&mbufs[0], num_packets); // :3588 有鎖
```

本設定 `gpus: [0]` → `cuda_device_ids` 非空 → **走無鎖的 `send()`**。安全的原因：每個 Peer 從 `QueueManager::assign_txq()` 拿到獨佔的 TXQ。

第三個 `send_lock(mbufs, count, tx_window_start, timing)`（`queue.cpp:174`）會在 burst 第一包打時間戳（`:189-194`），但 **C-plane 不用它**（時間戳更早就寫好了）。它只被 RU emulator 與 `fh_generator` 的 U-plane 路徑使用。

### 2.7 timing 保護

`sendCPlane_timingCheck`（`fh.cpp:935-955`）在 `prepareCPlaneInfo` 一進來就檢查（`:1329`）：

```cpp
if((start_tx_time.count() - time_now.count()) < sendCPlane_timing_error_th_ns)   // :941
    ret = SEND_{DL,UL}_CPLANE_TIMING_ERROR;                                      // :946/:950
```

本設定門檻為 **0**，代表只有 `start_tx` 已經過去才報錯。失敗時會 `section_id_ready.store(true)` 解除下游阻塞後直接 return（`fh.cpp:1332-1333`），下游 U-plane 準備會印：

> `prepareUPlanePackets: prb C-Plane sections info not initialized, likely due to sendCPlane Timing error`（`fh.cpp:3069`）

---

## Part 3：U-plane 下行發送（GPU / DOCA GPUNetIO 路徑）

### 3.1 每個 DL slot 的 task 序列

完整的 DL task 註冊順序（`cuphydriver_api.cpp`）：

```
:1790  TaskDL2AggrPrepare(N)   → prepareUPlanePackets（CPU 填 PartialUplaneSlotInfo）
:1796  TaskDL2AggrTx           → GpuComm::send() = memset + pre_prepare + prepare [+ trigger]
:1820  TaskDL1AggrCplane(N)    → C-plane（見 Part 2，不在本節範圍）
:1847  TaskDL1AggrCompression  → kernel_compress<9>（BFP9 壓縮，直接寫進封包 payload）
:1863  TaskDL2RingCpuDoorbell  → GpuComm::cpu_send()   ← 被 :1852 if(gpuCommEnabledViaCpu()) 包住
:1880  TaskDL3Aggr             → buffer cleanup
```

本節只談 U-plane 相關的那幾個；C-plane task（`:1820`）走的是完全不同的路徑。

### 3.2 CUDA stream 與 MPS context

- `cstream_`：`cudaStreamNonBlocking`、priority **-5**（`gpu_comm.cpp:52`）— memset / pre_prepare / prepare / trigger
- `cstream_pkt_copy_`（`gpu_comm.cpp:53`）— 僅 via_cpu 路徑的 D2H copy

兩條 stream 在 `GpuComm` 建構時建立，而 `GpuComm` 建於 `Nic` 建構中（`nic.cpp:92`），此時 current context 已切成 GPU-comms MPS context（`context.cpp:971-980`）。

`mps_sm_gpu_comms: 16` → `gpuCommsMpsCtx = new MpsCtx(..., getMpsSmGpuComms())`（`context.cpp:797`，gate 在 `:795`），透過 `cuCtxCreate_v3` + `CU_EXEC_AFFINITY_TYPE_SM_COUNT`（`mps.cpp:58-67`）。

**涵蓋**：`gpu_comm_doca.cu` 裡跑在這兩條 stream 上的全部 kernel。
**不涵蓋**：`kernel_compress` — 它跑在 cell 的 DL stream，context 是 `dlMpsCtx = pdschMpsCtx`（`context.cpp:782`），吃 **`mps_sm_pdsch: 46`**。

### 3.3 Kernel 鏈

#### ① `gpu_comm_pre_prepare_send_doca`（`gpu_comm_doca.cu:173`）

launcher `gpucomm_pre_prepare_send()`（`:487-515`），grid `dim3(num_cells, 14)`、block 128、on `cstream_`；呼叫端 `gpu_comm.cpp:441`。

做的事：
1. 把 CPU 準備好的 section info 與 `FlowPtrInfo` async 拷進 shared memory（`:198-202`）
2. 寫入該 symbol 的送出時戳 `ts` / `ptp_ts`（`:211-212`）
3. 算每包的 `hdr_addr`（GPU 位址）/ `hdr_addr_tx`（WQE 用的位址）/ `pkt_buff_mkey` / `hdr_stride`（`:308-327`）
4. **就地改寫 header template 的變動欄位**：`ecpriPayload`(`:369`)、`ecpriSeqid`(`:373`)、frame/subframe/slot/symbolId(`:375-378`)、`numPrbu`/`startPrbu`/`rb`/`sectionId`(`:381-384`)
5. 算 `tot_wqebbs` 與前綴和（`:397-422`）
6. **產生 `prb_ptrs[]`**（`:426-464`）：

```cpp
// gpu_comm_doca.cu:461
params.payload_info.prb_ptrs[cell_id][offset + prb] =
        &hdr_ptr[packet_hdr_size + (prb - packet_ptr->start_prbu) * packet_prb_size];
```

這一步是整條路徑的關鍵設計：**壓縮 kernel 之後會直接把 PRB 寫進封包 payload 區，沒有任何額外搬移**。

> GPU **不複製** template，而是用 `hdr_stride + hdr_idx` 算出 ring buffer 內某一格的位址（`:313, 319-327`）後**就地覆寫**變動欄位。

#### ② `gpu_comm_prepare_send_doca`（`gpu_comm_doca.cu:518`，eMPW 路徑）

launcher `gpucomm_prepare_send()`（`:870-927`），`disable_empw: 0` → 走這條（`:876`）；呼叫端 `gpu_comm.cpp:454`。

1. 讀 `eth_txq_gpu->wqe_pi`，`+= previous_wqebbs + previous_waits`（`:577-578`）
2. thread 0 先寫一個 **wait-on-time WQE**：`doca_gpu_dev_eth_txq_wqe_prepare_wait_time(..., symbol_info.ts, ...)`（`:588-589`），`wqebb_idx_start++`（`:616`）
3. 每 thread 一包：`doca_gpu_dev_eth_txq_wqe_prepare_send_empw(..., pkts_in_wqe, rel_pkt_id, hdr_addr_tx, pkt_buff_mkey, packet_size, flag)`（`:680`）
4. **via_cpu 時**：最後一個 block 直接 `doca_gpu_dev_eth_txq_submit_proxy(eth_txq_gpu, final_wqe_pi)`（`:723`），並把 `old_wqe_pi` / `wqebbs_per_cell` 回寫 host 可見的 `h_slot_info`（`:697-715`）
5. 非 via_cpu 時只記 `last_wqebb_ctrl_idx[0]`（`:731-735`），留給 trigger kernel

#### ③ eMPW vs nonEmpw

| | eMPW（本設定） | nonEmpw |
|---|---|---|
| 每包 API | `..._wqe_prepare_send_empw`（`:680`） | `..._wqe_prepare_send`（`:858`） |
| 一個 WQE 容納 | 最多 `EMPW_DSEG_MAX_NUM = 61` 包（`:61`） | 1 包 |
| WQEBB 計算 | `whole_wqes*ceil((61+2)/4) + ceil((rem+2)/4)`（`:402-405`，`SEGMENTS_PER_WQE=4`，`:62`） | 1 WQEBB / 包（`:796`） |
| index 推進 | `previous_wqebbs + previous_waits`（`:578`） | `previous_pkts + previous_waits`（`:796`） |

兩條路都會在每個有封包的 symbol 前插入 wait-on-time WQE（各佔 1 WQEBB）。

#### ④ Trigger kernel — **本設定不執行**

```cpp
// gpu_comm.cpp:465
if(getNic()->get_fronthaul()->get_info().enable_gpu_comm_via_cpu == 0) { ... gpucomm_trigger_send ... }
```

`gpu_init_comms_via_cpu: 1` → **整段跳過，GPU 完全不 ring doorbell**。

（供對照，via_cpu=0 時：`gpu_comm_trigger_send_doca_cx7`（`:981-1010`）會 `update_dbr`（`:991`）→ `atomic_add(wqe_pi)`（`:992`）→ thread 0 spin 等壓縮完成旗標（`:995-996`）→ 最後才 `ring_db`。CX6 版本（`:965`）則靠 tx_pp clock queue。）

> ⚠️ **CX7 的 `ring_db` 在 ARM 上不是平行的**（`gpu_comm_doca.cu:999-1009`）：
>
> ```cpp
> #ifdef __aarch64__
>     // Temporary workaround for https://redmine.mellanox.com/issues/3638896
>     if (threadIdx.x == 0) {
>         while(DOCA_GPUNETIO_VOLATILE(*wait_flag) != wait_val);   // 再 spin 一次
>         for (int i = 0; i < blockDim.x; i++)
>             doca_gpu_dev_eth_txq_ring_db(params.cell_params[i].eth_txq_gpu, ...);
>     }
> #else
>     doca_gpu_dev_eth_txq_ring_db(params.cell_params[threadIdx.x].eth_txq_gpu, ...);
> #endif
> ```
>
> **GB10 是 arm64，所以走上面那條**——由 thread 0 序列化 ring 所有 cell 的 doorbell，而且會多 spin 一次 `wait_flag`。「每 thread 平行」是 x86 才有的行為。
> （本設定 via_cpu=1 時整個 trigger kernel 都不執行，這段只在你把 via_cpu 關掉時才有意義。）

### 3.4 doorbell 由誰敲 — 本設定的關鍵

**是 CPU。** 呼叫鏈：

```
cuphydriver_api.cpp:1852-1866   只有 gpuCommEnabledViaCpu() 才排 "TaskDL2RingCpuDoorbell"
 → task_function_dl_aggr.cpp:2006  task_work_function_dl_aggr_2_ring_cpu_doorbell
 → task_function_dl_aggr.cpp:2079  fhproxy->RingCPUDoorbell(...)
 → common/fh.cpp:3551-3566
 → aerial_fh_driver.cpp:826-830    aerial_fh::ring_cpu_doorbell
 → nic.cpp:1284-1304               Nic::ring_cpu_doorbell
 → gpu_comm.cpp:148                GpuComm::cpu_send
```

`GpuComm::cpu_send`（`gpu_comm.cpp:148-350`）做四件事：
1. `cudaStreamWaitEvent(cstream_pkt_copy_, compression_stop_evt)`（`:172-174`）
2. **逐 symbol（0..13）把 GPU packet buffer D2H 搬到 `cpu_comms_cpu_pkt_addr`**（`:232-279`，預設用 `cuphyBatchedMemcpyHelper` 批次 `cudaMemcpyAsync`）
   > 精確地說是「**最多** 14 次批次提交」，不是固定 14 次 memcpy：該 symbol 若 `copy_params.max_pkts == 0` 會跳過 copy 只 record event（`:221-224`）；非零時是把該 symbol 的每 cell × 每 flow（環形緩衝跨界時再拆 1~2 段）全部塞進 helper，再**一次** launch（`:247-273`）。
3. 用 `cudaEventQuery` pipeline 等每個 symbol copy 完（`:289-313`）
4. 每 cell 呼叫 **`doca_eth_txq_gpu_cpu_proxy_progress(txh[cell]->eth_txq_cpu)`**（`:318-323`）— **這才是真正把 doorbell 寫下去的地方**

> ⚠️ **`gpucomm_ring_doorbell_*` 整組是死碼，別拿它當證據。**
> 精確地說：launcher `gpucomm_ring_doorbell_for_cells`（`gpu_comm_doca.cu:941`，宣告在 `gpu_comm.hpp:138`）在全 repo **沒有任何呼叫端**；kernel `gpucomm_ring_doorbell_per_cell`（`:930`）唯一的呼叫來自那個 launcher 自己（`:954`），另外出現在 `force_loading_gpu_comm_kernels()` 清單（`:1097`）。所以是整組不可達，不是「kernel 沒有呼叫端」。

### 3.5 `gpu_init_comms_via_cpu: 1` 對發送路徑改了什麼

| 層面 | via_cpu = 0 | via_cpu = 1（本設定） |
|---|---|---|
| TXQ SQ 記憶體 | `DOCA_GPU_MEM_TYPE_GPU` | `DOCA_GPU_MEM_TYPE_CPU_GPU`（`queue.cpp:78-85`） |
| UAR（doorbell 暫存器） | GPU 側 | **`doca_eth_txq_gpu_set_uar_on_cpu()`**（`doca_obj.cpp:633`） |
| 封包緩衝 | 一份 GPU VRAM | GPU 一份 + CPU-visible 一份（`cpu_comms_*`，`doca_obj.cpp:549-595`），各有 mkey |
| WQE 指向 | GPU buffer | **CPU buffer**（`hdr_addr_tx`，`gpu_comm_doca.cu:319-321`） |
| 壓縮 kernel 寫入 | GPU buffer | GPU buffer（`hdr_addr`，`:435`）|
| doorbell | GPU trigger kernel | **CPU proxy_progress** |
| 額外成本 | — | **每 symbol 一次 D2H copy** |

**這是與上行接收的重要不對稱**：接收方向 via_cpu 只是換記憶體位置、**不多一次複製**；發送方向則實實在在多了一趟 GPU→CPU 的 D2H 搬移。

#### 官方依據（DOCA 3.4.0 文件）

這整套設定不是 Aerial 自創的 workaround，而是 NVIDIA 對本平台的**指定做法**：

> "Due to hardware topology limitations, **DGX Spark does not support GPUDirect RDMA**."
>
> "DOCA GPUNetIO applications can still execute on these systems by utilizing CPU pinned memory (`DOCA_GPU_MEM_TYPE_CPU_GPU`) instead of GPU memory."
>
> "When creating Rx or Tx queues with DOCA Ethernet, you must use the setters in `doca_eth_rxq_gpu_data_path.h` and `doca_eth_txq_gpu_data_path.h` to allocate queues on the CPU-GPU shared memory."
>
> "**For Tx queues, you must also enable CPU proxy mode to handle transmission**."
>
> — [GPUNetIO Installation and Setup](https://networking-docs.nvidia.com/doca/archive/3-4-0/gpunetio-installation-and-setup)

> "Both APIs support **CPU proxy mode**, a fallback mechanism for systems where **direct DoorBell ringing from the GPU is not possible**."
>
> — [GPUNetIO Architecture and Design](https://networking-docs.nvidia.com/doca/archive/3-4-0/gpunetio-architecture-and-design)

對應到 Aerial 的實作：

| 官方要求 | Aerial 程式碼 |
|---|---|
| Rx queue 用 CPU-GPU shared memory 的 setter | `doca_eth_rxq_gpu_set_rq_mem_type(..., DOCA_GPU_MEM_TYPE_CPU_GPU)`（`doca_obj.cpp:131`） |
| Tx queue 同上 | `doca_eth_txq_gpu_set_sq_mem_type(..., DOCA_GPU_MEM_TYPE_CPU_GPU)`（`doca_obj.cpp:626`） |
| **Tx 必須開 CPU proxy mode** | `doca_eth_txq_gpu_set_uar_on_cpu()`（`doca_obj.cpp:633`）+ 跳過 trigger kernel（`gpu_comm.cpp:465`）+ `doca_eth_txq_gpu_cpu_proxy_progress()`（`gpu_comm.cpp:318-323`） |

**所以 3.4 節「doorbell 由 CPU 敲」不是 Aerial 的設計選擇，是這個平台上 DOCA 強制要求的。** 換句話說，`gpu_init_comms_via_cpu: 1` 在 Spark 上不是可調的效能選項，是唯一能動的組態。

> ⚠️ 用詞注意：官方把 `DOCA_GPU_MEM_TYPE_CPU_GPU` 稱為 **CPU pinned memory**（另一處敘述則說「memory resides on the GPU and is accessible also by the CPU」，兩處說法不一致）。GB10 上 CPU/GPU coherent 共用實體記憶體，這個區別在效能上可能沒有實際意義，但對外描述時應以「CPU pinned memory」為準。

### 3.6 BFP-9 壓縮：`decompress_scale_blockFP` 的反向

- 啟動：`generic_cuda_kernels.cu:274` `launch_kernel_compression()`；`bit_width==9` 且全 cell 一致時特化成 `kernel_compress<9>`（`:308-311`），grid `(max_antennas, num_cells, 14)`、block `COMPRESSION_THREADS = 576`（`comp_kernel.cuh:22`）
- kernel：`comp_kernel.cuh:25`，`comp_meth: 1` → `scale_compress_blockFP`（`:65`）
- 實作：`cuPHY-CP/compression_decompression/comp_decomp_lib/include/gpu_blockFP.h:41`
  - 3 threads/PRB、8 值/thread、10 PRB/warp（`:51-57`）
  - 乘 beta 轉整數（`:113-114`）→ warp 內 3 thread reduce 求 max（`:120-135`）
  - exponent：`shift = max(0, 33 - __clz(maxV) - compbits)`（`:138`）
  - `packOutput<3>(...)`（`:148`）→ `gpu_packing.h:112`，**compParam(exponent) 寫在每個 PRB 的第 0 byte**（`:130-131`）

PRB 大小驗算：`PRB_SIZE(9) = 9*2*12/8 = 27` + 1 byte exponent = **28 bytes**（`oran.hpp:318`，`utils.hpp:227-241`）

**beta 計算**（`cell.cpp:661-685`）：
```
sqrt_fs0 = 2^(dl_bit_width-1) * 2^(2^exponent_dl - 1)
fs       = sqrt_fs0^2 * 2^(-fs_offset_dl)
beta_dl  = sqrt( fs * 10^(ref_dl/10) / (12 * nPrbDlBwp) )
```
本設定 `fix_beta_dl: 0` → 走公式（若為 1，9-bit 會直接寫死 65536，`cell.cpp:674-684`）。cell 0 是 `fs_offset_dl: 7 / exponent_dl: 4 / ref_dl: 0`。

> **但 `beta_dl` 不是最終餵進 kernel 的值。** kernel 讀的是 `params.beta[cell]`（`comp_kernel.cuh:44`），來源是 `task_function_dl_aggr.cpp:130` 的 `cell_ptr->getBetaDlPowerScaling()`，而該函式回傳的是：
>
> ```cpp
> // cell.cpp:1412-1414
> return beta_dl * oam_linear_gain;
> ```
>
> `oam_linear_gain` 初值為 1（`cell.cpp:78`），但 OAM 下 attenuation 指令時會變成 `pow(10, -attenuation_dB/20)`（`cell.cpp:1404`）。所以上面的公式只是**未經 OAM 衰減時**的值。

> `bfw_beta_prescaler` **不參與** U-plane IQ 壓縮，它只餵給 BFW 壓縮（`phydlbfw_aggr.cpp:92`），是另一套 blockFP（`cuPHY/src/compression_decompression/bfw_blockFP.cuh:55`）。

### 3.7 送出時序

`T1a_max_up_ns: 339000` 的消費路徑：

```
task_function_dl_aggr.cpp:2493   start_tx = getTaskTsExec(0) + TTI_ns(mu)*slotAhead - T1a_max_up_ns
 → fh.cpp:2933                   加上 TAI offset
 → fh.cpp:3271-3272              prepare_uplane_gpu_comm(...)
 → peer.cpp:4347-4349            section_info[sym].ptp_ts = cell_start_time + sym*symbol_duration
                                 doca_eth_txq_calculate_timestamp(eth_txq_cpu, ptp_ts, &section_info[sym].ts)
 → gpu_comm_doca.cu:211-212      pre_prepare kernel 抄進 gpu_symbol_info[sym].ts
 → gpu_comm_doca.cu:589          prepare kernel 寫進 wait-on-time WQE
```

**port 層級的 offload**（`nic.cpp:374-384`）：
```cpp
if (is_cx6()) { if(!accu_tx_sched_disable) offloads |= RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP; }
else          { offloads |= RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP; }   // CX7 無條件開
```

**CX6 vs CX7 為何要分開**：偵測在 `nic.cpp:222-232`，`doca_eth_txq_cap_get_wait_on_time_offload_supported()` 回 `DOCA_ETH_WAIT_ON_TIME_TYPE_DPDK` 就判定為 CX6。CX6 的 wait-on-time 靠 mlx5 tx_pp clock queue（devarg `tx_pp=<accu_tx_sched_res_ns>`，`nic.cpp:238-243`，**只加給 CX6**），CX7 是硬體原生。

本機是 CX7 → 不加 `tx_pp` devarg，`accu_tx_sched_res_ns: 500` 在本設定的 DOCA 路徑上**不生效**。

---

## Part 4：容易誤解與無效參數

### 4.1 三個容易誤解的點

1. **doorbell 不是 GPU 敲的**（本設定）。GPU kernel 只呼叫 `submit_proxy` 記下 producer index；真正寫 UAR 的是 CPU 的 `doca_eth_txq_gpu_cpu_proxy_progress`（`gpu_comm.cpp:318-323`）。
2. **DPDK 沒有退場**。即使 U-plane 資料面完全由 GPU 處理，port 初始化、MTU、`rte_flow_isolate`、所有 flow steering 規則、CPU TXQ 都仍是 DPDK。分工是「DPDK 管控制面、GPUNetIO 管資料面」。
3. **`gpucomm_ring_doorbell_*` 整組是死碼**——launcher 無呼叫端，kernel 只被那個 launcher 呼叫。

### 4.2 無效 / 誤導的參數

| 設定 | 實際狀況 |
|---|---|
| `txq_size: 8192` | 只餵給 CPU TXQ 的 `rte_eth_tx_queue_setup`（`queue.cpp:92`）。**GPU TXQ 用寫死的 `QUEUE_DESC = 8192`**（`defaults.hpp:94`），兩者數值巧合相同但無關聯——改 yaml 不會影響 GPU TXQ 深度 |
| `uplane_tx_handles: 64` | **本設定下完全無效**。它建兩個 63-entry 的 pool（`nic.cpp:558-574` U-plane、`:576-592` C-plane），但兩個都不會被取用：U-plane pool 的消費點 `Peer::prepare_uplane`（`peer.cpp:4252`）位於 `fh.cpp:3270` / `:3513` 的 `if(gpuCommDlEnabled())` **else 分支**，`gpu_init_comms_dl: 1` 時不可達；GPU 路徑改用 per-peer 的 `up_tx_request_`（`peer.cpp:430` 動態配置 `kPeerSlotsInfo`=16 格，索引於 `:4326`）。C-plane pool 的消費點 `Peer::prepare_cplane`（`peer.cpp:3542`）只被 ru-emulator 呼叫，**cuphycontroller 完全不走**——`Peer::send_cplane` 是直接從 `get_cpu_mbuf_pool()` 取 mbuf（`peer.cpp:3609-3611`）|
| `accu_tx_sched_res_ns: 500` | ⚠️ **不是「隨便設都沒差」**。它只在 **CX6** 才變成 `tx_pp` devarg（`nic.cpp:239-243`），本機 CX7 → **不影響行為**；但 `Fronthaul::validate_input()`（`fronthaul.cpp:90-96`）在**任何網卡型號下**都檢查：值必須是 0，或落在 `kMinAccuTxSchedResNs`(500) ~ `kMaxAccuTxSchedResNs`(10000)（`defaults.hpp:49-50`），否則 `THROW_FH(EINVAL)` **啟動即失敗**。改成例如 100 會直接起不來 |
| `ulc_alloc_cplane_bfw_txq: 1` | 被 `if (ctx_cfg.mMIMO_enable)` 包住（`cuphycontroller_scf.cpp:425`），`mMIMO_enable: 0` → **不生效** |
| `sendCPlane_dlbfw_backoff_th_ns` | 只在 mMIMO 路徑消費（`task_function_dl_aggr.cpp:981`，gate 在 `:972`）→ **不生效** |
| `sendCPlane_ulbfw_backoff_th_ns` | 同上，但在**另一個檔案**：`task_function_ul_aggr.cpp:1375`（gate 在 `:1365`）→ **不生效** |
| `T1a_min_cp_dl_ns` | 只在 `dlc_bfw_enable_divide_per_cell` 開啟時用（`task_function_dl_aggr.cpp:1042-1044`），本設定為 0 → **無 runtime 效果** |
| `T1a_min_cp_ul_ns` | 同上，但在**另一個檔案**：`task_function_ul_aggr.cpp:1421-1423`（gate 是 `ulc_bfw_enable_divide_per_cell`）→ **無 runtime 效果** |
| `T1a_max_cp_dl_ns` | 唯一的 runtime 消費點（`task_function_dl_aggr.cpp:1043`）與 `T1a_min_cp_dl_ns` 在同一個三元運算式，同樣被 `dlc_bfw_enable_divide_per_cell: 0` 擋住 → **無 runtime 效果**。DL 送出時間實由 `Tcp_adv_dl_ns + T1a_max_up_ns` 決定（見 2.5） |
| `queue.cpp:73` 的 log | 對 GPU queue 印的 descriptor 數是 `txq_size` 而非實際的 `QUEUE_DESC`，除錯時會誤導 |

---

## Part 5：除錯用 log 字串

### 初始化階段

| 訊息 | level | 位置 |
|---|---|---|
| `Initializing NIC {} with {} RX queues and {} TX queues` | I | `nic.cpp:388` ← TXQ 總數（CPU+GPU） |
| `Setting up TXQ #{} on NIC {} with {} descriptors for GPU-init comm {}` | I | `queue.cpp:73` ← 末欄 0/1 區分 CPU/GPU |
| **`Doca Ethernet TxQ created! gpu_addr {} cpu_addr {}`** | I | `queue.cpp:89` ← **GPU TXQ 成功最明確的判準** |
| ~~`Mapping transmit queue buffer ... with dmabuf mode`~~ | C | `doca_obj.cpp:517` — ⚠️ **本設定下不會印**，見下方說明 |
| ~~`Mapping transmit queue buffer ... with nvidia-peermem mode`~~ | C | `doca_obj.cpp:507` — ⚠️ **本設定下不會印** |

> ⚠️ **這兩條 log 在 `gpu_init_comms_via_cpu: 1` 下都不會出現**，不能拿來判斷記憶體映射路徑。
> 它們整段位於 `if(!enable_gpu_comm_via_cpu)` 區塊內（`doca_obj.cpp:496-546`），via_cpu 開啟時直接跳過，連 `pkt_buff_mkey` 都不會設定——實際使用的是 `:549-595` 的 `cpu_comms_*` 分支（`DOCA_GPU_MEM_TYPE_CPU_GPU` + 獨立 mkey）。
> RX 側同理（`doca_obj.cpp:167-197` 的 `Mapping receive queue buffer ...` 也在 `if (!enable_gpu_comm_via_cpu)` 內）。
> 這符合預期——Spark 本來就不走 GPUDirect RDMA 註冊路徑（見 3.5 節官方依據）。要確認走對路，看的是 `gpu_init_comms_via_cpu` 的啟動 log（`GPU-initiated comms (via CPU): 1`），不是 dmabuf/peermem。
| `Setting NIC {} PCIe MRRS to 4096 bytes for NIC` | — | `nic.cpp:821` |

### TXQ 用途分派（確認每條 queue 給了誰）

| 訊息 | 位置 |
|---|---|
| `Requesting {} TXQ for GPU U-plane` | `peer.cpp:173` |
| `... is using NIC {} GPU TXQ #{} for U-plane` | `peer.cpp:204` |
| `... is using NIC {} TXQ #{} for DL C-plane` | `peer.cpp:212` |
| `... is using NIC {} TXQ #{} for UL C-plane` | `peer.cpp:217` |
| `Ran out of TXQs GPU. Please increase TXQ GPU count for NIC {}` | `queue_manager.cpp:63`（THROW） |

### Header template

| 訊息 | 位置 |
|---|---|
| `Setting up header template for {} Flow {}: [Destination MAC...][eCPRI Message Type...]` | `flow.cpp:247` ← 可逐欄核對 MAC/VLAN/eAxC |
| `Registering total packets {} packet_size_rnd {} ... flow num {} aggr_ptr {} flow_ptr {}` | `flow.cpp:406` ← GPU header buffer slice 位址 |

### Runtime 錯誤

| 訊息 | 位置 |
|---|---|
| `prepareUPlanePackets: prb C-Plane sections info not initialized, likely due to sendCPlane Timing error` | `fh.cpp:3069` |
| `At least two sections have the same SectionId value` | `fh.cpp:2545` |
| `Ran out of TxRequestUplane descriptors` | `peer.cpp:4255` |
| `rte_eth_tx_burst timeout` | `queue.cpp:125, 160, 202`（THROW） |

> 注意：`nic.cpp:564/582`（TX request pool）與 `queue.cpp:239`（warm-up）是 **NVLOGD**，預設 log level 看不到，需調高 `FH.NIC` / `FH.QUEUE` verbosity。

`tx_pp` 健康度只能從 xstats 觀察：`metrics.cpp:85-91`（timestamp_past/future errors、jitter、wander）與 `nic.cpp:725`（`tx_pp_sync_lost`）。

---

## Part 6：未確認事項

以下是**無法從本 repo 原始碼確認**的內容，不應當成事實引用：

1. **DOCA device 函式的內部行為**：`doca_gpu_dev_eth_txq_wqe_prepare_send_empw` / `..._prepare_wait_time` / `..._submit_proxy` / `..._ring_db` / `..._update_dbr` 的實作在 `doca_gpunetio_dev_eth_txq.cuh`，不在此 repo。
   - *2026-08-08 部分澄清*：官方文件說明 doorbell 提交函式「allows choosing between GPU and CPU Proxy handlers via the template parameter」，且 `DOCA_GPUNETIO_ETH_NIC_HANDLER_AUTO` 會自動選擇。本 repo 的 RX 路徑（`order_cuda_kernels.cu:3983` 等）正是用 `..._NIC_HANDLER_AUTO`。逐函式的內部語意仍未公開。
2. ~~eMPW 的 WQE bit-layout~~ — **已於 2026-08-08 確認，不再是推論**。依據就在 `gpu_comm.cpp:30` 自己 include 的 `cuPHY-CP/aerial-fh-driver/lib/mlx5.hpp`：
   - `:30, :209` `#define MLX5_WSEG_SIZE 16u` → 一個 segment = 16 B
   - `:185-195` `struct mlx5_wqe` = cseg + eseg + dseg[2]；`:197-199` `struct mlx5_ds_wqe` = dseg[4] → **1 WQEBB = 64 B = 4 × 16 B segment**，即算式裡的 `/4`
   - `:216-218` 註解原文：「Absolute maximum size is **63 (MLX5_DSEG_MAX) segments**, the WQE size field in Control Segment is **6 bits wide**」→ 63 = 61 + 2，正好解釋 `EMPW_DSEG_MAX_NUM = 61` 與算式裡的 `+2`（control segment + eth segment）

   仍未公開的只有 DOCA device 端 `doca_gpu_dev_eth_txq_wqe_prepare_send_empw()` 的內部實作。
3. **GPU TXQ 必須在 `rte_eth_dev_start()` 之後建立的原因**：repo 內無註解、無文件說明。只能確認事實順序（`nic.cpp:79-80`）。約束在 DOCA 函式庫外部。
4. `PrepareCellParams::flow_d_hdr_template_info`（`peer.cpp:411` 配置的 `d_hdr_template_`）在所有 kernel 中未被讀取，用途不明。
5. `PartialUplaneSlotInfo_t::qp_clock_id`（`fronthaul.hpp:232`）在 GPU kernel 中被註解掉（`gpu_comm_doca.cu:210`），CX6 的 qp_clock_id 如何進入送包路徑未確認。
6. 壓縮 kernel 的輸入索引用 `max_num_prb_per_symbol` 當 stride（`comp_kernel.cuh:33-34`），而 `tx_tensor` 用 `ORAN_MAX_PRB=273` 配置（`dlbuffer.cpp:55`）。兩者是否一致取決於 cuPHY channel 寫入時的 stride，未追到底。
7. `rte_eth_tx_burst` 之後到封包真正離開網卡之間，是 mlx5 PMD + 硬體排程的行為，不在本 repo 內。

### 已知的程式碼瑕疵（實際讀到的）

- `Txq::~Txq`（`queue.cpp:99-105`）對 GPU queue 只取出 `gpu_` 指標，`// Destroy TXQ` 是空註解——GPU TXQ 沒有被真正銷毀。
- `Nic::setup_tx_queues` / `setup_tx_queues_gpu`（`nic.cpp:419-421, 432-434`）宣告了 `txq_count_gpu`、`txq_size` 卻沒用到（dead code）。
- `set_pcie_max_read_request_size()`（`nic.cpp:819-853`）用 `popen` 跑 `setpci`，`:842` 直接覆寫讀值最高 nibble 為 `5` 且未檢查格式；`popen` 失敗只 warn（`:828-832`），且需要 root。
- `GpuComm::txq_init()`（`gpu_comm.cpp:110-114`）只做 `cudaStreamSynchronize` 就 return，像是預留的 hook。
