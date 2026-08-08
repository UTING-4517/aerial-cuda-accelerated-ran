# DPDK 進階：rte_flow、精準發送、mlx5 PMD

**這份文件的定位**：假設你會用 `rte_eth_rx_burst()` / `rte_eth_tx_burst()` 收送封包，但沒接觸過 `rte_flow`、TX 時間戳排程、mlx5 驅動參數。這三樣在一般 DPDK 應用裡可以不碰，但在 O-RAN fronthaul 是必須的。

**與 GPU 無關**——就算完全不用 GPUNetIO，用 FlexRAN 做 O-RAN fronthaul 一樣會用到這些。

> **路徑慣例**：文中所有 `檔案:行號` 都是**相對於 repo 根目錄**（本文件位於 `notes/`，往上一層）。設定基準 `cuPHY-CP/cuphycontroller/config/cuphycontroller_F08_WNC_DGX.yaml`。所有行號都經過實際讀取確認。

**姊妹文件**：[`GPUNETIO_INTRO.md`](GPUNETIO_INTRO.md)、[`TX_PATH_CPLANE_UPLANE.md`](TX_PATH_CPLANE_UPLANE.md)

---

## 0. 你已經懂的，是「最後一哩」

```c
nb_rx = rte_eth_rx_burst(port_id, queue_id, mbufs, MAX_BURST);   // 從 queue 拿封包
nb_tx = rte_eth_tx_burst(port_id, queue_id, mbufs, nb);          // 把封包丟進 queue
```

這兩個函式各自跳過了一個問題：

| 函式 | 跳過的問題 | 一般應用的預設答案 | O-RAN 需要的答案 |
|---|---|---|---|
| `rx_burst` | 封包**怎麼進到這個 queue** | 全進 queue 0，或 RSS 雜湊分散 | 按 eAxC ID 精確分流 → **`rte_flow`** |
| `tx_burst` | 封包**什麼時候真的離開網卡** | 盡快送出 | 在指定的奈秒時刻送出 → **SEND_ON_TIMESTAMP** |

---

## 1. `rte_flow`：控制封包進哪個 queue

### 1.1 沒有 rte_flow 的時候

**單 queue**：網卡收到的所有封包都進 queue 0。

**多 queue**：DPDK 預設用 **RSS（Receive Side Scaling）**——網卡對每個封包的 IP/port 做雜湊，用結果決定進哪個 queue。目的是把流量平均分散到多個 CPU core，同時保證同一條連線的封包進同一個 queue（維持順序）。

RSS 的特性是「**均勻分散**」。你無法指定「這種封包給我進 queue 3」。

### 1.2 rte_flow 是什麼

對網卡下**精確規則**：「**符合這些條件**的封包 → **做這件事**」。規則在網卡硬體裡執行，**不消耗 CPU**。

一條規則由三部分組成：

#### Pattern（比對什麼）

一串由外而內的協定層，以 `END` 收尾：

```c
rte_flow_item patterns[] = {
    { .type = RTE_FLOW_ITEM_TYPE_ETH,   .spec = &eth_spec,   .mask = &eth_mask   },
    { .type = RTE_FLOW_ITEM_TYPE_VLAN,  .spec = &vlan_spec,  .mask = &vlan_mask  },
    { .type = RTE_FLOW_ITEM_TYPE_ECPRI, .spec = &ecpri_spec, .mask = &ecpri_mask },
    { .type = RTE_FLOW_ITEM_TYPE_END }
};
```

每層有兩個結構：

- **spec**：要比對的值
- **mask**：哪些 bit 要比。全 `0xFF` = 該欄位必須完全相符；全 `0` = 不比這個欄位

> mask 是最容易出錯的地方。忘記設 mask（預設全 0）會變成「這個欄位不比」，規則比你以為的寬鬆很多。

#### Action（做什麼）

```c
rte_flow_action_queue queue = { .index = 3 };
rte_flow_action actions[] = {
    { .type = RTE_FLOW_ACTION_TYPE_QUEUE, .conf = &queue },
    { .type = RTE_FLOW_ACTION_TYPE_END }
};
```

常見 action：

| Action | 作用 |
|---|---|
| `QUEUE` | 導向指定的 RX queue |
| `DROP` | 硬體直接丟棄 |
| `RSS` | 用 RSS 分散到一組 queue |
| `MARK` | 打標記，之後在 mbuf 的 `hash.fdir.hi` 讀得到 |
| `COUNT` | 計數（可用來確認規則有沒有比中，除錯很有用） |
| `JUMP` | 跳到另一個 group 繼續比對 |
| `SAMPLE` | 複製一份到別的 queue（做鏡像／抓包） |

#### Attribute（規則的屬性）

```c
rte_flow_attr attr = { .group = 0, .ingress = 1 };
```

- **`ingress` / `egress`**：這條規則管收包還是送包。**fronthaul 收包用 `ingress = 1`**
- **`group`**：規則分組，可用 `JUMP` 在 group 間跳轉，做多級比對
- **`priority`**：同 group 內的優先序，數字小的先比

### 1.3 裝上去

```c
rte_flow_validate(port_id, &attr, patterns, actions, &err);   // 先問網卡支不支援
struct rte_flow *flow = rte_flow_create(port_id, &attr, patterns, actions, &err);
```

**一定要先 `validate`**。不同網卡支援的 pattern/action 組合差很多，`validate` 會告訴你這條規則能不能裝，而不是裝到一半才失敗。

`rte_flow_create` 回傳的 handle 之後可用 `rte_flow_destroy()` 移除。

### 1.4 Aerial 實際下的規則

位置：`cuPHY-CP/aerial-fh-driver/lib/peer.cpp:528-693`（`Peer::create_rx_rule`）

每個上行 eAxC 裝一條，內容：

| 層 | spec | mask |
|---|---|---|
| ETH | dst MAC = DU 自己（yaml `src_mac_addr`）、src MAC = RU（yaml `dst_mac_addr`） | 全 `0xFF` |
| VLAN | TCI = `pcp<<13 \| vid`，本設定 = `7<<13 \| 564` = **`0xE234`** | 全 `0xFFFF`（**PCP 也在比對範圍內**） |
| eCPRI | `common.type` = `RTE_ECPRI_MSG_TYPE_IQ_DATA`(0x00)、`type0.pc_id` = eAxC ID | 全 `0xFF` / `0xFFFF` |

⚠️ **收包方向 MAC 是對調的**：yaml 的 `src_mac_addr`（DU 自己）在規則裡是比對 **dst**，`dst_mac_addr`（RU）是比對 **src**。

Action（`peer.cpp:583-586`）：

```cpp
queue.index = rxq_->get_doca_rx_items()->dpdk_queue_idx;
rte_flow_action actions[]{
    {.type = RTE_FLOW_ACTION_TYPE_QUEUE, .conf = &queue},
    {.type = RTE_FLOW_ACTION_TYPE_END}};
```

**`dpdk_queue_idx` 就是 DOCA queue 透過 `rte_pmd_mlx5_external_rx_queue_id_map()` 註冊進 DPDK index 空間的那個編號**（`queue.cpp:319`）。這是 DPDK 與 GPUNetIO 的縫合點——flow rule 是 DPDK 的，但指向的 queue 是 DOCA 建的。

Attribute（`peer.cpp:577`）：

```cpp
rte_flow_attr attr{.group = 0, .ingress = 1};
```

### 1.5 為什麼 Aerial 非用 rte_flow 不可

1. **按 eAxC 分流**。RSS 只能雜湊分散，無法保證「eAxC 8 的封包進特定 queue」。
2. **把封包導進 GPU 的 queue**。那個 queue 不是 `rte_eth_rx_queue_setup()` 建的，只能靠 flow rule 指過去。

### 1.6 `rte_flow_isolate`：白名單模式（最重要的副作用）

`cuPHY-CP/aerial-fh-driver/lib/nic.cpp:854-865`（`Nic::restrict_ingress_traffic`）：

```cpp
auto ret = rte_flow_isolate(port_id_, 1, &flowerr);   // :860
```

**開啟後，沒有 flow rule 明確允許的封包，網卡直接在硬體丟棄。** 不進任何 queue，任何軟體統計都看不到。

配合 1.4 的 full-mask 比對，這代表以下情況封包會**靜默消失，沒有任何 log**：

- MAC 位址錯一個 bit
- VLAN ID 對，但 **PCP 不對**（因為 TCI 是 full-mask，PCP 佔高 3 bits）
- eAxC ID 不在規則清單裡
- eCPRI message type 不是 IQ data（例如對方送了 C-plane 過來）

**這跟「封包總會進 queue 0，我 rx_burst 就看得到」的直覺完全相反。**

### 1.7 一個 Aerial 的靜默降級

`cuPHY-CP/aerial-fh-driver/lib/peer.cpp:608-614`：

```cpp
auto flow = rte_flow_create(port_id, &attr, patterns, actions, &err);
if(!flow && std::string(err.message).find("eCPRI") != std::string::npos)
{
    NVLOGW_FMT(TAG, "eCPRI parser not supported on NIC {}, retrying without eCPRI", name.c_str());
    patterns[2] = {.type = RTE_FLOW_ITEM_TYPE_END};   // ← 砍掉 eCPRI 那層
    flow = rte_flow_create(port_id, &attr, patterns, actions, &err);
}
```

如果網卡不支援 eCPRI parser，規則會**降級成只比對 ETH + VLAN**，只留一行 warning。降級後**所有 eAxC 的封包都會進同一個 queue**，per-eAxC 的過濾完全消失。

除錯時值得先 grep 這行 warning。

### 1.8 除錯 rte_flow 的方法

| 方法 | 怎麼做 |
|---|---|
| 加 `COUNT` action | 在規則裡加 `RTE_FLOW_ACTION_TYPE_COUNT`，用 `rte_flow_query()` 讀命中次數。**確認規則有沒有被比中的最直接方法** |
| `validate` 的錯誤訊息 | `rte_flow_validate()` 失敗時 `err.message` 會說明是哪個 item/action 不支援 |
| 網卡硬體計數器 | `ethtool -S <iface>` 看 `rx_packets`，或 `rte_eth_xstats_get()` 讀 mlx5 的 device 層級計數（`rx_vport_unicast_packets` 等）。⚠️ 注意 `rte_eth_stats_get()` 與 per-queue 計數是 PMD **軟體**計數，看不到 GPUNetIO 的 external RxQ；**device 層級的硬體計數器看得到**。硬體收到但應用沒收到 = 規則沒比中或被 isolate 丟掉 |
| `tcpdump` | 注意：DPDK 綁定的介面通常看不到，需要 `dpdk-pdump` 或先用核心驅動測 |
| Aerial 的 log | `flow.cpp:247` / `peer.cpp` 的 `Setting up flow rules for ... [Destination MAC: ...][VLAN Tci: ...][eCPRI eAxC ID: ...]`，可逐欄核對實際裝上去的值 |

---

## 2. `RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP`：控制何時送出

### 2.1 問題

`rte_eth_tx_burst()` 的語意是「**盡快送出**」。

但 O-RAN fronthaul 要求封包在**精確的時間窗**抵達 RU。設定檔裡的 `T1a_max_up_ns: 339000`、`T1a_max_cp_ul_ns: 392000`、`Ta4_min_ns: 84000` 這些就是窗的邊界。

如果靠 CPU「算好時間再呼叫 `tx_burst`」，要對抗的是：OS 排程抖動、中斷、cache miss。即使 SCHED_FIFO + `isolcpus`，微秒級精度也很難保證。

### 2.2 解法：把時間交給硬體

在 mbuf 上標一個**時間戳**，網卡自己等到那個時刻才送出。CPU 可以提早很多把封包丟進去。

### 2.3 三個步驟

#### ① 開啟 offload

`cuPHY-CP/aerial-fh-driver/lib/nic.cpp:374-384`：

```cpp
if(is_cx6())
{
    if(!fhi_->get_info().accu_tx_sched_disable)
        eth_conf.txmode.offloads |= RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP;
}
else
{
    eth_conf.txmode.offloads |= RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP;   // CX7 無條件開
}
```

然後在 `rte_eth_dev_configure()` 時生效（`nic.cpp:390`）。

#### ② 註冊 mbuf 的動態欄位

`rte_mbuf` 結構是固定大小的，但 DPDK 允許在裡面「租」空間放自訂資料：

- **dynfield**（動態欄位）：租一塊空間，回傳**偏移量**
- **dynflag**（動態旗標）：租一個 bit，回傳 **bit 位置**

為什麼是執行期分配？因為多個函式庫可能都要租，誰先註冊誰先拿，位置不能寫死。

`cuPHY-CP/aerial-fh-driver/lib/fronthaul.cpp:354-378`：

```cpp
static const rte_mbuf_dynfield dynfield_desc = {
    .name  = RTE_MBUF_DYNFIELD_TIMESTAMP_NAME,
    .size  = sizeof(uint64_t),
    .align = __alignof__(uint64_t)
};
static const rte_mbuf_dynflag dynflag_desc = { RTE_MBUF_DYNFLAG_TX_TIMESTAMP_NAME };

timestamp_offset_ = rte_mbuf_dynfield_register(&dynfield_desc);      // 拿偏移量
int32_t dynflag_bitnum = rte_mbuf_dynflag_register(&dynflag_desc);   // 拿 bit 位置
timestamp_mask_ = 1ULL << dynflag_shift;
```

這兩個名字（`RTE_MBUF_DYNFIELD_TIMESTAMP_NAME` / `RTE_MBUF_DYNFLAG_TX_TIMESTAMP_NAME`）是 DPDK 定義的**約定名稱**，PMD 也用同一個名字去查，所以驅動和應用才對得上。

#### ③ 送出前寫進 mbuf

`cuPHY-CP/aerial-fh-driver/lib/peer.cpp:2953-2957`：

```cpp
if(info.tx_window.tx_window_start > last_packet_ts)
{
    mbufs[0]->ol_flags = fhi->get_timestamp_mask_();                     // 舉旗：這包要排程
    *RTE_MBUF_DYNFIELD(mbufs[0], fhi->get_timestamp_offset(), uint64_t*)
            = info.tx_window.tx_window_start;                            // 寫時間戳
    last_packet_ts = info.tx_window.tx_window_start;
}
```

然後照常 `rte_eth_tx_burst()`。網卡看到旗標就知道要等。

**兩個實作細節**：

1. **只有 burst 的第一個 mbuf 打時間戳**，後面的跟著走
2. **有單調性檢查**（`if(... > last_packet_ts)`），避免時間戳倒退——倒退的時間戳網卡會判為「已過期」而立刻送出或丟棄

### 2.4 時間基準

時間戳用的是**網卡的 PHC（PTP Hardware Clock）**時間域，不是 `CLOCK_REALTIME`。Aerial 會加上 TAI offset 轉換（`fh.cpp:1338`）。

這也是為什麼 fronthaul 一定要 PTP（`ptp4l` + `phc2sys`）正常運作——網卡時鐘沒對準，時間戳就沒有意義。

### 2.5 CX-6 與 CX-7 的差異

| | CX-6 | CX-7 |
|---|---|---|
| wait-on-time 實作 | 靠 mlx5 的 **tx packet pacing** 引擎（軟體輔助） | **硬體原生** |
| 需要 `tx_pp` devarg | **要**（`nic.cpp:239-243`） | 不要 |
| 受 `accu_tx_sched_disable` 控制 | 是 | 否，無條件開 |

偵測方式（`nic.cpp:222-232`）：`doca_eth_txq_cap_get_wait_on_time_offload_supported()` 回傳 `DOCA_ETH_WAIT_ON_TIME_TYPE_DPDK` 就判定為 CX-6。

**本機是 CX-7**，所以 `accu_tx_sched_res_ns: 500` 這個設定實際上不生效（它只會變成 CX-6 的 `tx_pp` devarg）。

### 2.6 除錯

送出時序的健康度看 mlx5 的 xstats：

| 計數器 | 意思 |
|---|---|
| `tx_pp_timestamp_past_errors` | 時間戳已過期（送太晚了） |
| `tx_pp_timestamp_future_errors` | 時間戳太遠的未來 |
| `tx_pp_jitter` / `tx_pp_wander` | 時脈抖動 |
| `tx_pp_sync_lost` | 失去同步 |

Aerial 有把這些匯出成 Prometheus metric（`metrics.cpp:68-91`、`nic.cpp:684, 725`）。

---

## 3. mlx5 PMD：驅動層

### 3.1 PMD 是什麼

**PMD = Poll Mode Driver**，DPDK 的網卡驅動。

「Poll Mode」是相對於傳統中斷模式——DPDK 不用中斷，靠 CPU 主動輪詢 descriptor ring，省掉中斷開銷與 context switch。你呼叫 `rte_eth_rx_burst()` 時，底下就是 PMD 在讀網卡的 ring。

`mlx5` 是 NVIDIA/Mellanox **ConnectX-4 以後**（CX-4/4Lx/5/6/6Dx/7/8 與 BlueField）的 PMD。本機的整合式 CX-7 走的就是它。（更早的 ConnectX-3 走 `mlx4`。）

### 3.2 devargs：探測階段傳給驅動的參數

有些驅動行為必須在**裝置探測時**就決定，透過 devargs 字串傳入。

Aerial 組的字串（`cuPHY-CP/aerial-fh-driver/lib/nic.cpp:239-245`）：

```cpp
if(cx6) //tx_pp application only for CX-6 device
{
    if(!accu_tx_sched_disable)
        devargs_builder << "tx_pp=" << accu_tx_sched_res_ns << ",";
}
devargs_builder << "txq_inline_max=0,dv_flow_en=2"; //HWS
```

| 參數 | 意思 | 為什麼 Aerial 要 |
|---|---|---|
| `dv_flow_en=2` | **HW Steering (HWS)**——用網卡最新的硬體流表引擎執行 rte_flow | 規則多、要低延遲。原始碼註解就寫 `//HWS` |
| `txq_inline_max=0` | 關閉「把 payload 內嵌進 WQE」 | payload 在 GPU 記憶體或 external mbuf，inline 會強迫 CPU 讀取那些資料，破壞 zero-copy 並增加延遲 |
| `tx_pp=<ns>` | Tx packet pacing 引擎的時脈解析度 | **只給 CX-6**。見 2.5 |

### 3.3 mlx5 專屬 API

`rte_pmd_mlx5_*` 開頭的函式**不是 DPDK 通用 API**，是 mlx5 PMD 專屬的。Aerial 用到兩個關鍵的：

```cpp
// nic.cpp:251  註解：should be done before port probe
rte_pmd_mlx5_driver_enable_steering();

// queue.cpp:319  把 DOCA 建的 queue 註冊進 DPDK 的 index 空間
rte_pmd_mlx5_external_rx_queue_id_map(port_id, doca_rx_h.dpdk_queue_idx, doca_rx_h.hw_queue_idx);
```

第二個是整套 GPUNetIO + DPDK 架構能成立的關鍵。**這也是 Aerial 綁死在 NVIDIA 網卡上的原因**——換成 Intel 網卡，這條路整個不存在。

### 3.4 其他 mlx5 相關設定

`Nic` 建構時還做了兩件 mlx5 專屬的事（`nic.cpp:59-64`，只在 `driver_name_ == kMlxPciDriverName` 時執行）：

- **`set_pcie_max_read_request_size()`**（`nic.cpp:819-853`）：用 `popen` 跑 `setpci` 把 PCIe MRRS 設成 **4096 bytes**，減少 DMA read 交易數。需要 root。
- **`disable_ethernet_flow_control()`**（`nic.cpp:1045`）：關掉 PAUSE frame，避免流量控制打亂 fronthaul 時序。

---

## 4. 這些東西在 Aerial 的哪裡

| 主題 | 檔案 | 關鍵行 |
|---|---|---|
| DPDK EAL 初始化 | `cuPHY-CP/aerial-fh-driver/lib/fronthaul.cpp` | `:38-70` |
| dynfield/dynflag 註冊 | `cuPHY-CP/aerial-fh-driver/lib/fronthaul.cpp` | `:354-378` |
| devargs 組裝、CX6 偵測 | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp` | `:222-251` |
| port 設定、TX offloads | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp` | `:366-398` |
| `rte_flow_isolate` | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp` | `:854-865` |
| MRRS / flow control | `cuPHY-CP/aerial-fh-driver/lib/nic.cpp` | `:819-853`, `:1045` |
| **上行 flow rule** | `cuPHY-CP/aerial-fh-driver/lib/peer.cpp` | **`:528-693`** |
| eCPRI 降級 fallback | `cuPHY-CP/aerial-fh-driver/lib/peer.cpp` | `:608-614` |
| C-plane 時間戳寫入 | `cuPHY-CP/aerial-fh-driver/lib/peer.cpp` | `:2953-2957` |
| header template | `cuPHY-CP/aerial-fh-driver/lib/flow.cpp` | `:215-297` |
| queue 建立（DOCA/DPDK 分岔） | `cuPHY-CP/aerial-fh-driver/lib/queue.cpp` | `:273-331` |
| `rte_eth_tx_burst` | `cuPHY-CP/aerial-fh-driver/lib/queue.cpp` | `:122` |

> 注意：`flow.cpp` 裡也有一整套 `create_rx_rule()`，但在本設定（`rx_mode = RxApiMode::PEER`）下**不會執行**——`Flow::create_rx_rules()` 只對 `FLOW`/`HYBRID` 模式作用（`flow.cpp:495-513`）。實際生效的是 `peer.cpp` 那套。追程式碼時很容易讀錯檔。

---

## 5. 收不到封包時的檢查順序

從硬體往軟體走，這個順序能最快縮小範圍：

1. **實體層**：`ethtool <iface>` 看 link 是否 up、速率是否正確
2. **網卡硬體計數器**：`ethtool -S <iface>` 看 `rx_packets` 有沒有在增加
   - 沒增加 → 對方沒送，或線路問題。跟軟體無關
   - 有增加但應用收不到 → 繼續往下
3. **flow rule 有沒有裝上**：看啟動 log 的 `Setting up flow rules for ...`，逐欄核對 MAC / VLAN TCI / eAxC ID
4. **有沒有發生 eCPRI 降級**：grep `eCPRI parser not supported`
5. **規則有沒有被比中**：加 `COUNT` action 用 `rte_flow_query()` 讀，或看網卡的 flow 相關計數器
6. **檢查最容易錯的三個欄位**：
   - **VLAN PCP**（TCI 是 full-mask，PCP 錯就整條不匹配）
   - **MAC 方向**（收包時 yaml 的 `src_mac_addr` 是比對 dst）
   - **eAxC ID** 是否在 `eAxC_id_*` 清單內
7. **`rte_flow_isolate` 的影響**：規則沒比中就是硬體丟棄，**不會有任何 log**。這一步沒有捷徑，只能回頭核對規則

> 在 Aerial 這種 GPUNetIO 架構下，還要多一層注意：**DPDK 的 RX 統計看不到 GPU queue 的封包**。詳見 [`GPUNETIO_INTRO.md`](GPUNETIO_INTRO.md) §6。

---

## 6. 延伸閱讀

- [DPDK Programmer's Guide — Generic flow API (rte_flow)](https://doc.dpdk.org/guides/prog_guide/rte_flow.html)
- [DPDK — mlx5 poll mode driver](https://doc.dpdk.org/guides/nics/mlx5.html)（devargs 完整清單在這）
- [DPDK — Mbuf Library](https://doc.dpdk.org/guides/prog_guide/mbuf_lib.html)（dynfield / dynflag 章節）
- `rte_flow` 的實驗工具：DPDK 內建的 `testpmd` 有互動式 `flow create` 指令，適合先在安全環境試規則
