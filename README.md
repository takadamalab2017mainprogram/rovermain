## クロスコンパイラ環境の構築方法 ##
1. Ubuntu環境を整える．
2. ターミナルで  
sudo apt-get update  
sudo apt-get upgrade  
sudo apt-get install lib32z1 lib32ncurses5 lib32bz2-1.0  
sudo apt-get install libc6-i386 lib32stdc++6 zlib1g:i386  
3. https://bitbucket.org/hogefugabar/minerva/downloads/compiler.tar.gz  
を落としてきて，解凍し，ソースコードのあるディレクトリに放り込む．  
4. ~/.bashrcに  
export PKG_CONFIG_PATH=さっき解凍したコンパイラのディレクトリのパス/arm-linux-gnueabihf/lib/pkgconfig:$PKG_CONFIG_PATH  
という1行を追加して，  
source ~/.bashrc  
を実行．
5. ソースコードのあるディレクトリでmakeコマンド実行．

## プログラムの転送方法 ##
1. USBメモリのルートディレクトリにarliss2014rivaiというディレクトリをつくる．  
2. 転送したいものを1で作ったディレクトリにいれる．
3. USBメモリをローバーに挿して，setup.pyを実行する．実行方法は，  
./setup.py  
もしくは  
python setup.py  
4. USBメモリを抜く．