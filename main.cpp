#include <iostream>
// PCL 01 モデルを表示
// Cのmin,maxマクロを無効にする
#define NOMINMAX
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
// 点群の型を定義しておく
typedef pcl::PointXYZ PointType;
#include <opencv2/opencv.hpp>

// 時間関連ライブラリ（処理時間を計測するための方法）
#include <time.h>
// 文字（String）関連ライブラリ
#include <string>

// ディレクトリ内のファイル名を取得するためにBoostを利用
// C++の新しいバージョンならstringの範囲内で実行できるらしいが，今回は，C++バージョンが古いのでこのように実装した．
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// 撮影データのサイズ（YCAMに対応した設定）
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 1024

// キャリブレーション時
//#define MAX_DISTANCE 1120 //mm 作成するデプス画像での底面（真黒）となる部分
//#define MIN_DISTANCE 880 //mm 作成するデプス画像での上面（真白）となる部分
// 実験時
//#define MIN_DISTANCE 900 //mm 作成するデプス画像での上面（真白）となる部分
//#define MAX_DISTANCE 955 //mm ※※箱上面から底面までは5.5cmなので「MIN_DISTANCE+55」に設定！(作成するデプス画像での底面（真黒）となる部分 )
#define ROTATE180 1 //YCAMで撮影した写真の上辺がNextage側になるなら180°回転させる必要があるので「１」を、そうでなければ「０」を。

// 将棋の駒で深度画像を取得するときの調整．
//#define MIN_DISTANCE 200 // 200
//#define MAX_DISTANCE 400 // 320
//#define MIN_DISTANCE 325
//#define MAX_DISTANCE 400

#define MIN_DISTANCE 420
#define MAX_DISTANCE 650

// 面倒くさいので実行オプションにせずに，直に書きました．
#define PATH_TO_PLY "./plyfiles/" 
#define PATH_TO_IMAGES_DIR "./images/" 

//#define SAVE_PCD
//#define FILTERED_PCD

namespace fs = boost::filesystem;

bool getFileNames(std::string folderPath, std::vector<std::string> &file_names);
bool convertPLY(std::string filename, std::string writename);

int main(int argc, char *argv[])
{

	std::string folders(PATH_TO_PLY);
	std::vector<std::string> filenames;
	std::vector<std::string>::iterator itr;
	bool flag_Readfolders = false;
	filenames.clear(); // initialize

    for (int i = 0; i < argc; ++i)
	{
		if (*argv[i] == '-')
		{
			switch(*(argv[i] + 1))
			{
				case 'a':
					flag_Readfolders = getFileNames(folders, filenames);
					for (itr = filenames.begin(); itr != filenames.end(); itr++) 
					{
						convertPLY(*itr, *itr);
					}
					break;
				default:
					break;
			}
		}
	}

	// 一枚だけ読み込みのパターン
	if (flag_Readfolders == false)
	{
		std::string Filename("out.ply"); // initial name
		convertPLY(Filename, Filename);
	}

	return 0;
}

bool convertPLY(std::string filename, std::string writename)
{
	clock_t start = clock();//Time start

	// 点群
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);

	// PLYファイルを読み込む
	pcl::PLYReader reader;
	pcl::PLYWriter writer;

	//これはexeファイルのあるディレクトリから見た相対パス
	reader.read(PATH_TO_PLY + filename, *cloud);

	#ifdef SAVE_PCD
    pcl::io::savePCDFileASCII("./plyfiles/out.pcd", *cloud); // PCDを出力
	#endif

	int count = 0;
	int count2 = 0;
	int x, y, pixel;
	cv::Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3); //3チャネルの画像用メモリを確保

	for (y = 0; y < WINDOW_HEIGHT; y++) {
		for (x = 0; x < WINDOW_WIDTH; x++) {
			// z値がある場合
			if (cloud->points.at(count).z > -9999999) {
				//if (count2 < 50) printf("%d - %f %f %f\n", count2++, cloud->points.at(count).x, cloud->points.at(count).y, cloud->points.at(count).z); // 値の読み取り
				// 「cloud->points.at(count).z」は恐らくメートル表記なので1000倍してmm単位で比較
				// 修正: 1000倍が不要なようなので，カットした．必要なら，if文の条件の値は1000倍にして比較する．
				if ((cloud->points.at(count).z) >= MAX_DISTANCE) {
					img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
				}
				else if ((cloud->points.at(count).z) <= MIN_DISTANCE) {
					img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
				}
				else {
					pixel = 255 * (1.0f - (float((cloud->points.at(count).z) - MIN_DISTANCE) / float(MAX_DISTANCE - MIN_DISTANCE)));
					img.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel, pixel, pixel);
				}
			}
			// nullの場合
			else {
        //printf("null %d\n", null_count++);
				img.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);
			}
			count++;
		}
	}

	// メディアンフィルタ適用（ごま塩ノイズ除去）->深度画像のゴミの削除
	cv::Mat img2(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3); //3チャネルの画像用メモリを確保
	//cv::fastNlMeansDenoisingColored(img, img2, 10, 10, 7, 21);
	cv::medianBlur(img, img2, 3); //カーネルサイズは必ず奇数
	//cv::imwrite(PATH_TO_IMAGES_DIR + (*itr) + "_MedianFilterDepth.png", img2);

	//cv::imwrite("./images/" + writename + "_default.jpg", img); //save the default image
	cv::imwrite("./images/" + writename + ".jpg", img2);

	// Crop and resize images
		//cv::Rect rect = cv::Rect(128, 0, WINDOW_WIDTH-256, WINDOW_HEIGHT); // # cv::Rect(始点x, 始点y, 幅, 高さ) を基にして、傾きのない四角形を描く
		//cv::Mat img_cropped(img2, rect);
		//cv::imwrite("./images/" + writename + "_cropped.jpg", img_cropped);
		//img_cropped.release();

	#ifdef FILTERED_PCD
		count = 0;
		int count_filter = 0;
		pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
		for (y = 0; y < WINDOW_HEIGHT; y++)
		{
			for (x = 0; x < WINDOW_WIDTH; x++)
			{
				if (cloud->points.at(count).z > -9999999 && (x > 0 && x < 1024) && (y > 650 && y < 1280)) {
					cloud_filtered->push_back(cloud->points.at(count));
					count_filter++;
				}
				count++;
			}
		}

		pcl::io::savePCDFileASCII ("./plyfiles/" + writename + "_filtered.pcd" , *cloud_filtered);
		printf("Point Size: %d\n", count_filter);
	#endif


	img.release();
	img2.release();
	//delete cloud; // error
	//delete cloud_filtered; // error

	// Show the calc time
	clock_t end = clock();     // end time
	std::cout << "Success Convert!, Duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

	return true;
}

// ディレクトリ内のファイル名を取得します．実は使っていない．複数にするときに使う？
bool getFileNames(std::string folderPath, std::vector<std::string> &file_names)
{
    const fs::path path(folderPath);
    bool vacant_check = false;

    BOOST_FOREACH(const fs::path& p, std::make_pair(fs::recursive_directory_iterator(path),
                                                    fs::recursive_directory_iterator())) {
        if (!fs::is_directory(p))
        {
          fs::path filename = p.filename();
          std::cout << "Read - "<< filename.generic_string() << std::endl;
          file_names.push_back(filename.generic_string());
          vacant_check = true;
        }
    }

    return vacant_check;
}
