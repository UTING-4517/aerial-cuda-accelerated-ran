# Aerial cuBB 學習筆記

自行整理的技術筆記，**非 NVIDIA 官方文件**。用來記錄追過的程式碼路徑、平台限制與踩過的坑。

## 文件

| 文件 | 主題 | 適合什麼時候看 |
|---|---|---|
| [GPUNETIO_INTRO.md](GPUNETIO_INTRO.md) | **DOCA GPUNetIO 入門**（給有 DPDK / FlexRAN 背景的人）。DPDK 知識哪些還適用、要新建立的 5 個 GPU 概念、上行封包完整旅程、FlexRAN 直覺會誤導的地方、工具對照、術語速查 | 建立心智模型；不確定某個 GPU 名詞是什麼 |
| [TX_PATH_CPLANE_UPLANE.md](TX_PATH_CPLANE_UPLANE.md) | **發送路徑完整拆解**。NIC 初始化 → C-plane 走 DPDK → U-plane DL 走 GPU。含 kernel 鏈、時序控制、無效參數清單、除錯用 log 字串 | 要改 code 或查某個設定實際做什麼 |

閱讀順序：先 `GPUNETIO_INTRO.md` 建立概念，再依需要查 `TX_PATH_CPLANE_UPLANE.md`。

## 慣例

寫新筆記時沿用這幾條，讓內容可被獨立驗證：

1. **路徑相對於 repo 根目錄** — 文中的 `檔案:行號` 一律相對 repo 根，不是相對 `notes/`。
2. **每個技術宣稱都附 `檔案:行號`** — 且必須實際讀過該行，不從檔名或函式名推測。
3. **分開「驗證過的」與「推測的」** — 無法從 repo 原始碼確認的內容集中在文末「未確認事項」，不混進正文。引用外部文件（如 DOCA 官方）時附連結與原文。
4. **標明設定基準** — 開頭寫清楚以哪個 yaml 為準。同一段程式碼在不同設定下走的分支可能完全不同（例如 `mMIMO_enable` 一開，C-plane 整條路徑就換了）。
5. **記下死碼與過期註解** — 追程式碼時遇到沒有呼叫端的函式、與程式碼矛盾的註解，寫下來。這類東西最容易誤導下一次閱讀。

## 目前的設定基準

現行使用 `cuPHY-CP/cuphycontroller/config/cuphycontroller_F08_WNC_DGX.yaml`（DGX Spark / GB10 + 整合式 ConnectX-7 @ `0000:01:00.0`）。

主要旗標：

```yaml
gpu_init_comms_dl: 1        # U-plane TX 走 GPU、C-plane TX 走 CPU
gpu_init_comms_via_cpu: 1   # DGX Spark 不支援 GPUDirect RDMA，官方指定做法
cpu_init_comms: 0
ul_order_kernel_mode: 0     # Ping-Pong 模式
cell_group_num: 1
mMIMO_enable: 0
enable_srs: 1               # → RX queue 變 2 個（一般 + SRS）
ru_type: 1                  # SINGLE_SECT_MODE
```

> 換設定檔時記得回頭確認筆記裡的分支敘述是否仍成立。
