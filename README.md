## クロスコンパイラ環境の構築方法 ##
1. Ubuntu環境を整える．
2. ターミナルで  
sudo apt-get update  
sudo apt-get upgrade  
sudo apt-get install lib32z1 lib32ncurses5 lib32bz2-1.0  
sudo apt-get install libc6-i386 lib32stdc++6 zlib1g:i386  
3. https://bitbucket.org/hogefugabar/minerva/downloads/compiler.tar.gz  
を落としてきて，解凍し，ソースコードのあるディレクトリに放り込む．  
4. ~~ ~/.bashrcに~~  
~~export PKG_CONFIG_PATH=さっき解凍したコンパイラのディレクトリのパス/arm-linux-gnueabihf/lib/pkgconfig:$PKG_CONFIG_PATH~~  
~~という1行を追加して，~~  
~~source ~/.bashrc~~  
~~を実行．~~    
やらなくていいです．2014/06/25  
5. ソースコードのあるディレクトリでmakeコマンド実行．

## UbuntuでBitbucketをsshで利用する方法 ##
1. ターミナルで，  
ssh-keygen  
で認証に使う鍵を生成する．~/.ssh/にid_rsa(秘密鍵)とid_rsa.pub(公開鍵)が生成される．  
2. 公開鍵の方をコピーして，Bitbucketのアカウントの管理-SSHキーに鍵を追加で貼り付ける．  
3. これでうまく動くはずだが，うまく動かない場合は  
ssh-add ~/.ssh/  
を実行するとうまく行くかも．

## WindowsでBitbucketをsshで利用する方法 ##
1. TortoiseGitをインストール
    * 本体： https://code.google.com/p/tortoisegit/
    * ライブラリ： http://msysgit.github.io/
2. puttyをインストール
    * 以下からZipまたはインストーラでダウンロード
    * http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html
3. PuTTYgenを起動してsshキーを生成
    * 公開鍵が上に表示されるからこれをBitBucketにコピペ
    * 秘密鍵はローカルの好きな場所に保存(クラウドは非推奨)
4. リポジトリをおきたいフォルダで右クリック→git cloneを選択
5. 設定する
    * URL： git@bitbucket.org:hogefugabar/minerva.git
    * Load Putty Keyにチェック
    * Putty Keyとして先ほどの秘密鍵を選択
## プログラムの転送方法 ##
1. USBメモリのルートディレクトリにarliss2014rivaiというディレクトリをつくる．  
2. 転送したいものを1で作ったディレクトリにいれる．
3. USBメモリをローバーに挿して，setup.pyを実行する．実行方法は，  
./setup.py  
もしくは  
python setup.py  
4. USBメモリを抜く．

## ログファイルのUSBメモリへの転送方法 ##
1. USBメモリのルートディレクトリにlogというディレクトリをつくる．  
2. USBメモリをローバーに挿して，log.pyを実行する．実行方法は，  
./log.py  
もしくは  
python log.py  
3. USBメモリを抜く．  

## 画像ファイルのUSBメモリへの転送方法 ##
1. USBメモリのルートディレクトリにpicというディレクトリをつくる．  
2. USBメモリをローバーに挿して，picture.pyを実行する．実行方法は，  
./picture.py  
もしくは  
python picture.py  
3. USBメモリを抜く．

## 自力で転送する方法 ##
1. sudo mount -t vfat /dev/sda1 /mnt/usbmem/
2. sudo cp /mnt/usbmem/移動したいファイル名 ./
3. sudo umount /mnt/usbmem/