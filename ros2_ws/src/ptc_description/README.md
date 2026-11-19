# Robot assets

此 package 不提供虛構的可部署 Franka/Bunny 資產。由使用者提供展開後的 URDF（包含實際 TCP、RGB/depth optical links、collision）、SRDF、mesh 與資產 identity。

共同應用只讀 URDF/已知 hand-eye YAML 並核對固定安裝。MoveIt 的 SRDF、joint limits、kinematics/controllers 設定位於可選 `ptc_moveit_config`。USD 不自動轉成完整 URDF/MoveIt config。

部署 URDF 的 mesh 引用先解析成相對／絕對本機檔案（不要留下 `package://`）；相鄰 robot description identity manifest 的 `dependencies` 列表逐一列出 `path`/`sha256`。loader 將 dependency digest 納入 semantic revision，description loader 再核對實際 visual/collision mesh 引用；缺檔或內容不符不就緒。各 ROS/MoveIt profile 使用同一份展開 URDF。

可選 `ptc_description_tools/usd_extrinsics` 離線讀取 composed USD，不讀受測物 pose，不在 core/runtime 連結 OpenUSD。命令：

```text
ros2 run ptc_description_tools usd_extrinsics stage.usd /Robot/TCP /Robot/Camera /Robot/Hand tcp camera_color_optical calibration-v1 hand_eye-v1.yaml
```

`fixedMountPrim` 必須是使用者確認的同一剛性安裝；動態 arm joints 可位於 mount 之上。工具拒絕缺少/未解析 references、animated/reset 分支與非剛性世界變換；USD camera 的 -Z/+Y 轉成 ROS optical 的 +Z/-Y，translation 依 metersPerUnit 轉成米。輸出保存所有 consumed source layers 的 digest 與相鄰 hand_eye identity manifest，不覆寫既有版本。

輸出設定到 `calibration.hand_eye_path`，frame/id 對應同一份 URDF。啟動 loader 檢查 TCP/camera 間只能通過 fixed joints，且 URDF known extrinsic 與輸出相符；取像時間 TF 由 vision 再使用。這是已知模擬安裝驗證，不是實體手眼外參估測。

OpenUSD API 參考：[XformCache](https://openusd.org/release/api/class_usd_geom_xform_cache.html)、[stage units](https://openusd.org/release/api/group___usd_geom_linear_units__group.html)、[consumed layers](https://openusd.org/release/api/class_usd_stage.html)。OpenUSD SDK 未安装的正常 runtime 不建置此離線工具。
