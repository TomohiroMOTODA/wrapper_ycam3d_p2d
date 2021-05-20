PointCloud_To_Depth

Written by Motoda 2019-10-25

main.cpp
CmakeLists.txt
の構成

関数仕様：
	ディレクトリ”./PLY_Files”内のPLYファイルを深度画像に変換し，
	ディレクトリ”./images”内に格納する．

関数仕様：
	コマンド -a : フォルダ内のファイルをすべて変換（オプションをつけない時にはout.plyのみ変換する．）
	
注意事項：
	ディレクトリが空の状態ではバグる．
	パラメータ調整を行うと，完全にコンパイルのし直し．
	
必要ライブラリ：
	OpenCV (> 2.0 )
	PCL　(> 1.8)
	Boost (> 1.0)
