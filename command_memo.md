# micromouse-std-esp32

## Commands to control thread


||Command|Describe|Parameter|
|:-|:-|:-|:-|
|**Search run**|||
||SStart| Starts running from a standstill.|
||SForward| Forward 1 block|
||SReturn| Return|
||SRight| Turn right|
||SLeft| Turn left|
||SStop| Stop|
|**Try run**|||
||TBD||
|**Other**|||
||WSEnable|Set Enable/Disable each sensor|
||GyroCalibration|Calibrate the gyro|offset: f32|

## Response to main thread

|Response|Describe|Corresponding commands|
|:-:|:--|:--|
|CalibrationDone|Gyro calibration finished|GyroCalibration|
|Judge|Judge the next drive command|SStart, SForward, SReturn, SRight, SLeft|
|Stopped|The micromouse stopped|SStop|

### Search run

```mermaid
sequenceDiagram
    main-)+ctrl: "Start"
	Note right of ctrl: 壁の情報を取得
    ctrl-)main: "Judge"
	Note left of main: 進行方向決定
	main-)ctrl: Drive command 1
	Note right of ctrl: "Start"コマンドの完了を待つ
	Note right of ctrl: Drive command 1を開始
	Note right of ctrl: 壁の情報を取得
	ctrl-)main: "Judge"
	Note left of main: 進行方向決定
	main-)ctrl: Drive command 2
	Note right of ctrl: Drive command 1の完了を待つ 
	Note right of ctrl: Drive command 2を開始
	Note right of ctrl: 壁の情報を取得
	Note over main, ctrl: ...
```

