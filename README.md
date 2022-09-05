# DIY-FFB-Wheel

自作フォースフィードバックステアリングホイール

## 必要なもの

- 「[Arduino Leonard(入出力 5V)](https://www.switch-science.com/catalog/968/)」または「[Pro micro(入出力 5V)](https://www.switch-science.com/catalog/3914/)」などの「Atmega32UX」系列の Arduino 対応ボード
- [Waveshare の CAN for RaspberryPi Pico ボード(入出力 3.3V)](https://www.waveshare.com/pico-can-a.htm)
- [レベル変換ボード(3.3V 系と 5V 系をつなぐ)](https://www.switch-science.com/catalog/1523/)
- [DDT インホイールモーター](https://www.switch-science.com/catalog/8248/)

## ファームウェアビルド

### 必要なライブラリ

- QuickPID(MIT-Licence)
- ArduinoJoystickWithFFB(LGPL-ICENCE)

上記２つのライブラリをライブラリマネージャから探してあらかじめインストールしておいてください。
ArduinoJoystickWithFFB は検索でも見つからないので、以下の操作でインストールしてください。

```コマンドプロンプト
winget install Git # <-まだgitコマンドが使えない場合のみ実行してください
cd C:\Users\<ユーザー名>\Documents\Arduino\libraries
git clone https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary
```

## ボードサポートのインストール

Arduino-IDE または Arduino-IDE-V2 または arduino-cli のいずれも利用可能です。
core サポートのインストールが必要です。「Arduino AVR Boards Support」を検索してインストールが必要です。

arduino-cli の場合は以下の操作でコアサポートインストールを行います。

コアサポートインストール

```コマンドプロンプト
arduino-cli core install arduuino:avr
```

## ビルドとフラッシュ

arduino-cli の場合

```コマンドプロンプト
cd firmware
make build flash
```

## 配線

| 名称   | Arduino Leonard | レベル変換 H 側 | レベル変換 L 側 | CAN アダプタ |
| ------ | --------------- | --------------- | --------------- | ------------ |
| RX     | 0 番            | HV1             | LV1             | GP1/RX       |
| TX     | 1 番            | HV2             | LV2             | GP0/TX       |
| Config | 2 番            | HV3             | LV3             | GP2          |
| Reset  | 3 番            | HV4             | LV4             | GP3          |
| 5V     | 5V              | HV              | -               | VSYS         |
| 3.3V   | 3.3V            | -               | LV              | -            |
| GND    | GND             | GND             | GND             | GND          |

## 3D モデル

- ステアリングバックプレート: steering.stl
- 固定台： holder.stl

STL ファイル内の寸法が 1000~1000000 分の 1 になってしまっているのは STL 出力の問題かもしれません。
スライサーソフトは「Cura」にて確認しましたがこちらは寸法の違いを自動的に修正してくれました。
