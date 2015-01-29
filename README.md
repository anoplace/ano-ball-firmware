TLBStudio Remote Controller
===========================

## ビルド環境の構築

* git、Python2.7が必要なので、予めインストールしてパスを通しておく
* 各環境に応じたSDKをダウンロードする [http://tocos-wireless.com/jp/products/ToCoNet/TWESDK.html](http://tocos-wireless.com/jp/products/ToCoNet/TWESDK.html)
* ダウンロードしたZIPファイルを展開する
* 展開したディレクトリ内の`Wks_ToCoNet`ディレクトリにこのリポジトリを`git clone`する
* git cloneしたリポジトリ内で`make`コマンドをターミナルで実行する
* 以下のパスにビルドされたバイナリが生成される
```
./RemoteController/Build/TLBStudioRemoteController_RemoteController_JN5164_X_X_X.bin  # リモコン用ファームウェア
./USBDongle/Build/TLBStudioRemoteController_USBDongle_JN5164_X_X_X.bin  # 受信機用ファームウェア
```


## ファームウェアの書き込み

* ターミナルを開いて`[SDKのパス]/Wks_ToCoNet/[リポジトリ名]`に移動する
* 全てのFTDIデバイスを取り外し、書き込むデバイスのみUSBで接続する
* ターミナルから`make flash reset`を実行する
* `reset success!`と表示されたら書き込み完了


## ソースコードのフォーマット

* clang-formatをインストールする
* ターミナルを開いて`[SDKのパス]/Wks_ToCoNet/[リポジトリ名]`に移動する
* ターミナルから`make format`を実行する
