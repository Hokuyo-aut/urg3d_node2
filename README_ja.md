# 概要
ROS2で動作する北陽電機製 3D測域センサ(LiDAR)のドライバです。

- 特徴
  - スキャンデータ出力
  - 補助データ出力（トピックの配信には`imu` `mag` `temperature`パッケージ使用）
  - ハードウェア診断情報の出力（診断情報の配信には`diagnostic_updater`パッケージ使用）
  - コンポーネント実装
  - [ライフサイクル制御](http://design.ros2.org/articles/node_lifecycle.html)対応

インタフェースや一部の処理は、ROS1でよく利用されている[hokuyo3dパッケージ](https://github.com/at-wat/hokuyo3d)を参考にしています。

LiDARとの通信には[urg3d_library](https://github.com/UrgNetwork/urg3d_library) のC言語APIを使用します。

## ライフサイクル制御
ライフサイクルの各状態での動作は以下のようになります。
- Unconfigured  
  起動状態
- Inactive  
  LiDAR接続状態（スキャンデータと診断情報は配信されません）
- Active  
  LiDARデータ配信状態（スキャンデータと診断情報が配信されます）
- Finalized  
  終了状態

# 対応機種
北陽電機製のVSSP 2.1に対応した測域センサ  
動作確認済み機種：`YVT-35LX-F0`, `YVT-35LX-FK`

動作確認済み環境：`foxy`, `galactic`, `humble`

# ライセンス
本パッケージのライセンスは`Apache License 2.0`です。  
urg_libray C言語APIのライセンスは`Simplified BSD License`です。

# Publish
- /hokuyo_cloud2 (sensor_msgs::msg::PointCloud2)  
  LiDARスキャンデータ
- /imu (sensor_msgs::msg::Imu)  
  6-D Imu データ（パラメータ`publish_auxiliary`=true時出力）
- /mag (sensor_msgs::msg::Imu)   
  3-D MagneticField データ（パラメータ`publish_auxiliary`=true時出力）
- /temperature (sensor_msgs::msg::Temperature)  
  センサ温度データ（パラメータ`publish_multiecho`=true時出力）  
- /diagnostics (diagnostics_msgs::msg::DiagnosticArray)  
  診断情報

# パラメータ
- ip_address (string, default: "192.168.0.10")  
  LiDARにイーサネット接続する場合のIPアドレス（"XX.XX.XX.XX"の形式で指定します）  
  ※指定する文字列が空（empty）の場合はシリアル接続になります。
- ip_port (int, default: 10940)  
  LiDARにイーサネット接続する場合のポート番号
- frame_id (string, default: "hokuyo3d")  
  スキャンデータのframe_id  
  スキャンデータのメッセージヘッダ"frame_id"には、パラメータ`frame_id`が設定されます。  
- range_min (string, default: 0.1)  
  range_min 以内の距離データは削除されます。このパラメータは、センサの近くにあるゴーストポイントを除去するのに役立ちます。
- interlace_h (int, default: 1)
  センサスキャンの水平インターレースを設定します。"1"を設定するとインターレースを使用しません。
- interlace_v (int, default: 1)
  センサスキャンの垂直インターレースを設定します。"1"を設定するとインターレースを使用しません。
- output_cycle (string, default: "frame")
  点群出力のタイミングを指定します。 "frame":全インターレースをまとめて出力、 "field":水平インターレース1つごとに出力、 "line"、垂直走査1回ごとに出力
- calibrate_time (bool, default: false)  
  調整モードのフラグ  
  このフラグがtrueの場合、スキャン開始時にLiDARの時刻とシステムの時刻のズレを計測しレイテンシとしてスキャンデータのtimestampに加算します。
- synchronize_time (bool, default: false)  
  同期モードのフラグ  
  このフラグがtrueの場合、スキャンデータのtimestampの基準となるシステム時刻についてLiDARの時刻とのズレを用いて動的補正します。
- publish_intensity (bool, default: true)  
  強度出力モードフラグ  
  このフラグがtrueの場合、スキャンデータの強度データ（intensities）が出力され、falseの場合、強度データは空（empty）で出力されます。  
  LiDARが強度出力に対応していない場合は、false設定と同様に動作します。
- publish_auxiliary (bool, default: false)  
  補助データ出力フラグ  
  このフラグがtrueの場合、マルチエコースキャンデータ（/imu, /mag, /temparture）が出力されます。
- error_limit (int, default: 4 [回])  
  再接続を実施するエラー回数  
  データ取得の際に発生したエラー回数がerror_limitより大きくなった場合にLiDARとの接続を再接続します。  
- error_reset_period (double, default: 10.0 [sec])  
  エラーのリセット周期  
  データ取得の際に発生したエラー回数を周期的にリセットします。  
  ※散発的なエラー発生における再接続を防止するためです。
- diagnostics_tolerance (double, default: 0.05)  
  スキャンデータの配信頻度に関する診断情報（Diagnostics）の目標配信頻度に対しての許容割合  
  ※例えばdiagnostics_toleranceが0.05であれば、目標配信頻度の105%～95%の範囲が正常範囲となります。
- diagnostics_window_time (double, default: 5.0 [sec])  
  スキャンデータの配信頻度に関する診断情報（Diagnostics）の出力配信頻度を計測する期間
- time_offset (double, default: 0.0 [sec])  
  スキャンデータのtimestampに加算するユーザレイテンシ

# ビルド方法

1. ソースコードの取得

```
$ cd <ROS2_workspace>/src
$ git clone --recursive https://github.com/Hokuyo-aut/urg3d_node2
```

2. 関連パッケージのインストール

```
$ rosdep update
$ rosdep install -i --from-paths urg3d_node2
```

3. ビルド

```
$ cd <ROS2_workspace>
$ colcon build --symlink-install
```

4. テスト（オプション）

`YVT-35LX-FK`を対象としたテストが実行されますので、`YVT-35LX-FK`を接続して電源ON状態で実行してください。

```
$ cd <ROS2_workspace>
$ colcon test
```

# 使用例

## 起動

1. LiDARを接続  
   イーサネットで接続します。  
1. 接続先（パラメータ）を設定  
   config/params.yaml を編集して接続先の設定を行います。  
1. ノードの起動  
   以下のコマンドを実行すると、自動でActive状態に遷移しスキャンデータの配信が開始されます。

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

   自動でActive状態へ遷移させたくない場合は以下のコマンドを実行してください。（起動後、Unconfigured状態になります）

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py auto_start:=false
   ```

   ※`urg3d_node2.launch.py`ではurg3d_node2を（コンポーネントとしてではなく）ライフサイクルノードとしてスタンドアロン起動しています。これはコンポーネントかつライフサイクルノードとして起動した場合に、launchでのライフサイクル制御インタフェースがないためです（現時点でのROS2では未実装）。

## パラメータの変更

パラメータの変更を行う場合は以下の手順を実行してください。

1. ノードの終了  
   ノードがすでに起動している場合は\<Ctrl-C\>でノードを終了させます。
1. パラメータを再設定  
   使用しているパラメータファイル（config/params.yamlなど）を編集します。
1. ノードの再起動  
   以下のコマンドでノードを起動します。

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

## 異常時のリセット

スキャンデータが配信されないなどの異常が発生した場合は以下の手順を実行してください。

1. ノードの終了  
   発生しているノードを\<Ctrl-C\>で終了させます。
1. 接続確認  
   LiDARの電源や接続が正常かどうか確認します。
1. ノードの再起動  
   以下のコマンドでノードを起動します。

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

# 制限事項

- Galacticでは2回目以降のInactive->Active状態遷移が失敗します  
diagnostic_updaterを生成するとdeclare_parameterが必ず実行されます。ROS2では、定義済みパラメータに対してdeclare_parameterを実行するとエラーになるため、状態遷移時のdiagnostic_updater生成に失敗して状態遷移ができません。  
下記のプルリクエストが反映されれば本制限は解消されます。  
  https://github.com/ros/diagnostics/pull/227

