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
ごめんわかんない．

## プログラムの転送方法 ##
1. USBメモリのルートディレクトリにarliss2014rivaiというディレクトリをつくる．  
2. 転送したいものを1で作ったディレクトリにいれる．
3. USBメモリをローバーに挿して，setup.pyを実行する．実行方法は，  
./setup.py  
もしくは  
python setup.py  
4. USBメモリを抜く．