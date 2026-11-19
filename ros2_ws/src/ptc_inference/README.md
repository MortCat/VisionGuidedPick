# RF-DETR runtime

## Product migration status — 2026-11-02

The active [product specification](../../../docs/architecture/product-workflow.md) requires target-only instance segmentation from an activated Teach Target model. Defect classes, EvaluateView and report completion are archived branch requirements, not prerequisites for recognizing/picking a target. The adapter/settings still contain these historical assumptions; this documentation update does not remove them or deliver target-only runtime integration. Keep the existing bounded inference boundary, original-pixel masks/bboxes and capture-time association when implementing Observe.

Python 僅用於模型推論；同一 model instance 同時處理連續受測物定位與有界 EvaluateView，GPU 工作串行，task evidence 不丟棄。連續取像以 latest-frame backpressure，模型結果以 frame/epoch/calibration/revision 關聯並可重用。

使用 C++ `settings_check --export-json` 載入同一 ready v2，不重新實作 YAML loader 或 semantic revision。實際 RF-DETR checkpoint 與 class-index names 由使用者訓練/提供；object 與 defect 類別必須同時可用、不得把 bunny 分數當瑕疵分數。

在執行此 Node 的 ROS Python 環境安裝 `requirements-inference.txt`。使用 RF-DETR 的 safe `from_checkpoint(..., trust_checkpoint=False)` API；缺少套件、舊/不相容 checkpoint 或 class mapping 不符會發布 not-ready，不自動退回 mock、下載訓練權重或開啟不安全 pickle。actual GPU/model smoke 與精度由使用者手動確認。

bbox 使用原始 RGB pixels，mask 可選 PNG；有效多缺陷輸出完整保留，超過 configured capacity 直接失敗，不截斷後宣稱完整。候選分數 threshold 必須不高於 object/defect 正式門檻。

官方 API：[checkpoint](https://rfdetr.roboflow.com/latest/reference/rfdetr/)、[inference](https://rfdetr.roboflow.com/latest/learn/run/detection/)。模型 backend 局限於本地 inference，不呼叫 Roboflow cloud、不上傳影像。
