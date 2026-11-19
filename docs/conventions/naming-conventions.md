# VisionGuidedPick 命名與模組結構規範

## 1. 目的

本規範讓開發者只看檔名或型別名稱，就能辨識程式入口、ROS 2 邊界、資料用途與模組責任。新增程式碼必須遵守本規範；既有初始骨架若尚未符合，應在該模組下一次修改時一併整理。

## 2. 基本命名

| 項目 | 格式 | 範例 |
| --- | --- | --- |
| C++ namespace | `snake_case` | `ptc_app` |
| C++ type | `PascalCase` | `ArmState` |
| Enum member | `kPascalCase` | `kPlanning` |
| 函式與 method | `snake_case` | `publish_state()` |
| 變數與 member | `snake_case`、member 加尾端 `_` | `target_pose`、`publisher_` |
| 檔案與目錄 | `snake_case` | `arm_gateway_node.cpp` |
| ROS package | `snake_case`、本專案 package 使用 `ptc_` | `ptc_interfaces` |
| ROS Topic、Service、Action | 小寫 `snake_case` 相對名稱 | `arm/move_to_pose` |
| ROS msg、srv、action 檔名 | `PascalCase` | `MoveToPose.action` |

## 3. 可執行模組

每個 ROS 2 executable 都將入口與 Node class 分離：
以下為規劃中的共同 gateway 與真後端組裝範例，檔案尚未建立；mock 組裝入口位於 `ptc_mock`。

```text
ptc_motion/include/ptc_motion/arm/
`-- arm_gateway_node.hpp

ptc_motion/src/arm/
`-- arm_gateway_node.cpp

ptc_motion_moveit/src/arm/
`-- arm_gateway_main.cpp
```

對應關係：

| 名稱 | 責任 |
| --- | --- |
| `arm_gateway_main.cpp` | 唯一程式入口；只做 ROS 初始化、建立 Node、spin、shutdown 與回傳結束碼 |
| `arm_gateway_node.hpp/.cpp` | `ArmGatewayNode` 的宣告與實作，負責 ROS wiring 與生命週期 |
| `arm_gateway_node` | CMake target、安裝後 executable 與 `ros2 run` 使用的名稱 |

禁止將設備邏輯、資料轉換或工作流程直接寫進 `main()`。
後端專用入口只組裝同一共同 Node 與 backend Adapter；MoveIt include/link 留在可選 package。mock 對應 `mock_arm_gateway_main.cpp`／`mock_arm_gateway_node`，不得複製共同 GatewayNode 或新增第二套狀態機。

## 4. 資料型別後綴

| 後綴 | 語意 | 範例 |
| --- | --- | --- |
| `State` | 某一時間點的運行狀態 | `ArmState`、`GripperState` |
| `Status` | 連線、健康度與錯誤摘要 | `DeviceStatus` |
| `Command` | 不需長時間追蹤的控制命令 | `MotionTargetCommand` |
| `Request` | 一次操作的輸入 | `CaptureRequest` |
| `Result` | 操作或流程的最終結果 | `InspectionResult` |
| `Config` | 從部署設定載入、運行期間通常不變的設定 | `ArmConfig` |
| `Params` | 函式或演算法的調整參數 | `MotionParams` |
| `Frame` | 同一時間點取得的感測資料 | `RgbdFrame`、`PointCloudFrame` |
| `Model` | 靜態描述、校正或內參 | `CameraModel` |
| `Detection` | 單一偵測輸出 | `ObjectDetection` |
| `Observation` | 包含時間、座標系與來源的觀測 | `ObjectObservation` |

`State` 表示設備或流程本身的動態狀態；`Status` 表示它是否可用及是否發生錯誤，兩者不得混用。

ROS 標準 `sensor_msgs/msg/CameraInfo` 只代表相機校正資訊。專案內部不得再定義 `CameraInfo` 來表示 RGB、Depth 或點雲；使用 `CameraModel`、`RgbdFrame` 與 `PointCloudFrame`。

## 5. 行為元件後綴

| 後綴 | 責任 | 範例 |
| --- | --- | --- |
| `Gateway` | 對外部設備提供穩定的內部操作邊界 | `ArmGateway` |
| `Adapter` | 接收外部介面、同步或整理後交給內部系統 | `CameraAdapter` |
| `Converter` | 進行明確且盡量無狀態的型別轉換 | `ArmStateConverter` |
| `Coordinator` | 協調多個能力完成一段工作流程 | `TaskCoordinator` |
| `Aggregator` | 聚合多筆輸入成單一結果 | `InspectionAggregator` |
| `Client` | 封裝 Action 或 Service client | `MoveToPoseClient` |
| `Server` | 實作 Action 或 Service server | `MoveToPoseServer` |
| `Node` | ROS 2 通訊、參數與生命週期邊界 | `ArmGatewayNode` |

`Gateway` 與 `Node` 是不同責任。`ArmGateway` 表達應用層可使用的手臂能力；`ArmGatewayNode` 負責把該能力接到 ROS 2。初期可以位於同一 package，但不得用 `Node` 取代核心能力介面。

避免使用下列無法表達責任的名稱：

```text
ArmInfo
RobotManager
MessageHandler
CommonUtils
```

應改為能描述實際語意的名稱，例如：

```text
ArmState
ArmGateway
JointStateConverter
TimestampValidator
```

## 6. Mock 命名

Mock 所扮演的角色必須出現在名稱中，並保留最終責任後綴：

```text
mock_arm_server_main.cpp
mock_arm_server_node.hpp
mock_arm_server_node.cpp
```

類別與 executable：

```text
MockArmServerNode
mock_arm_server_node
```

Mock 必須遵守與 Isaac Sim adapter 相同的 ROS contract，不得建立只有 mock 才能使用的捷徑介面。

## 7. ROS 名稱

- 程式碼內使用相對名稱，例如 `arm/move_to_pose`、`wrist_camera/depth/image_raw`。
- 工作站名稱由 launch 套用，例如 `/cell_1/arm/move_to_pose`。
- 不在 Topic 加入 `pick_turn_check` 或 `ptc` 前綴。
- `/clock`、`/tf`、`/tf_static` 維持 ROS 全域標準名稱。
- 名稱表達設備或能力，不表達實作來源；不得使用 `isaac_arm` 作為應用層固定介面名稱。
- 現有 schema v2 由 Settings YAML 管理 logical endpoint 相對名稱，再由邊界/launch remap；改名不改變型別、owner 或時間語意。五面／瑕疵契約是備案；新產品接口依 `docs/architecture/product-workflow.md` 實作，不直接加入 loader 尚未支援的新鍵。

## 8. 測試與 Launch

| 類型 | 格式 | 範例 |
| --- | --- | --- |
| 單元測試 | `<subject>_test.cpp` | `arm_state_converter_test.cpp` |
| 整合測試 | `<flow>_integration_test.cpp` | `move_to_pose_integration_test.cpp` |
| Launch | `<purpose>.launch.py` | `arm_smoke.launch.py` |
| 應用統一設定 | `settings.yaml` | `config/settings.yaml` |
| 第三方/部署設定 | `<module>.yaml` | MoveIt 所需的 `kinematics.yaml` |

測試名稱必須描述被驗證的主體或流程，不使用 `test1`、`basic_test` 等無法辨識目的的名稱。

## 9. Settings 與狀態機實作型別

`SystemSettings` 是全系統唯讀設定聚合的特例名稱；各領域使用 `TrackingConfig`、`TaskConfig`、`MotionConfig` 等明確 `Config` 後綴。
YAML/檔案邊界稱 `SettingsLoader`，只載入設定，不管理設備或可變作業狀態。
狀態機可使用 `TaskPhase` enum、`TaskState` 快照、`TaskEvent` 輸入、`TaskDecision` 輸出與 `TransitionRecord` 紀錄等後綴。
文件中的狀態機責任名稱不強制等於 enum 名稱；避免同名 enum/struct 或多個可寫的 motion 狀態。
