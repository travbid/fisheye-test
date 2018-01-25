#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const float square_size = 50.0f;

void drawCrosses(Mat &img, vector<Point2f> points);
void initImgPoints1(vector<Point3f> &w, vector<Point2f> &i);
void initImgPoints2(vector<Point3f> &w, vector<Point2f> &i);
void initImgPoints3(vector<Point3f> &w, vector<Point2f> &i);
void initImgPoints4(vector<Point3f> &w, vector<Point2f> &i);
void initImgPoints5(vector<Point3f> &w, vector<Point2f> &i);

int main() {

	vector<Point3f> w1;
	vector<Point2f> i1;
	initImgPoints1(w1, i1);

	// vector<Point3f> w2;
	// vector<Point2f> i2;
	// initImgPoints2(w2, i2);

	// vector<Point3f> w3;
	// vector<Point2f> i3;
	// initImgPoints3(w3, i3);

	// vector<Point3f> w4;
	// vector<Point2f> i4;
	// initImgPoints4(w4, i4);

	// vector<Point3f> w5;
	// vector<Point2f> i5;
	// initImgPoints5(w5, i5);

	vector<vector<Point3f>> world_points = {w1};
	vector<vector<Point2f>> img_points = {i1};

	namedWindow("fisheye", WINDOW_NORMAL);
	resizeWindow("fisheye", 1008, 764);

	// Mat img = imread("close.jpg");
	Mat img1(1528, 2016, CV_32FC3);
	drawCrosses(img1, i1);
	imshow("fisheye", img1);
	waitKey(0);

	// Mat img2(1528, 2016, CV_32FC3);
	// drawCrosses(img2, i2);
	// imshow("fisheye", img2);
	// waitKey(0);

	// Mat img3(1528, 2016, CV_32FC3);
	// drawCrosses(img3, i3);
	// imshow("fisheye", img3);
	// waitKey(0);

	// Mat img4(1528, 2016, CV_32FC3);
	// drawCrosses(img4, i4);
	// imshow("fisheye", img4);
	// waitKey(0);

	// Mat img5(1528, 2016, CV_32FC3);
	// drawCrosses(img5, i5);
	// imshow("fisheye", img5);
	// waitKey(0);

	Size image_size = cv::Size(2016, 1528);
	Matx33d K = cv::Matx33f(600.0, 0.0, 1007.5, 0.0, 600.0, 763.5, 0.0, 0.0, 1.0);
	Matx41d D = cv::Matx41f(0.0, 0.0, 0.0, 0.0);
	vector<Vec3d> rvecs;
	vector<Vec3d> tvecs;
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 500, DBL_EPSILON);
	const int flags =
	   /*fisheye::CALIB_USE_INTRINSIC_GUESS |*/ fisheye::CALIB_RECOMPUTE_EXTRINSIC; // | fisheye::CALIB_CHECK_COND;

	try {
		double rms = fisheye::calibrate(world_points, img_points, image_size, K, D, rvecs, tvecs, flags, criteria);
		printf("re-projection error: %f\n", rms);
	} catch (const cv::Exception &e) {
		printf("cv::fisheye::calibrate Exception:\n");
		printf("code: %d\n", e.code);
		printf("err: %s\n", e.err.c_str());
		printf("msg: %s\n", e.msg.c_str());
		printf("func: %s\n", e.func.c_str());
		printf("file: %s\n", e.file.c_str());
		printf("line: %d\n", e.line);
		return 0;
	} catch (...) {
		printf("cv::fisheye::calibrate unknown exception\n");
		return 0;
	}

	std::cout << "K:\n" << K << std::endl;
	std::cout << "D:\n" << D << std::endl;
	std::cout << "rvec[0]:\n" << rvecs[0] << std::endl;
	std::cout << "tvec[0]:\n" << tvecs[0] << std::endl;

	Mat undistorted;
	Matx33d newK = cv::Matx33f(200.0, 0.0, 1008.0, 0.0, 200.0, 764.0, 0.0, 0.0, 1.0);
	fisheye::undistortImage(img1, undistorted, K, D, newK, image_size);
	imshow("fisheye", undistorted);
	waitKey(0);
}

void drawCrosses(Mat &img, vector<Point2f> points) {
	for (size_t i = 0; i < points.size(); i++) {
		//
		Point2f h(8, 0);
		Point2f v(0, 8);
		line(img, points[i] - h, points[i] + h, Scalar(0, 0, 255), 2);
		line(img, points[i] - v, points[i] + v, Scalar(0, 0, 255), 2);
	}
}

// Points found from a picture taken of a calibration board by a camera with a fisheye lens
void initImgPoints1(vector<Point3f> &w, vector<Point2f> &i) {
	vector<Point3f> wvec = {
	   square_size * Point3f(-5, -13, 0),  square_size * Point3f(-4, -12, 0),  square_size * Point3f(-5, -12, 0),
	   square_size * Point3f(-7, -13, 0),  square_size * Point3f(-6, -13, 0),  square_size * Point3f(-6, -12, 0),
	   square_size * Point3f(-7, -12, 0),  square_size * Point3f(-5, -11, 0),  square_size * Point3f(-6, -11, 0),
	   square_size * Point3f(-1, -11, 0),  square_size * Point3f(0, -11, 0),   square_size * Point3f(0, -10, 0),
	   square_size * Point3f(-1, -10, 0),  square_size * Point3f(-3, -11, 0),  square_size * Point3f(-2, -11, 0),
	   square_size * Point3f(-2, -10, 0),  square_size * Point3f(-3, -10, 0),  square_size * Point3f(-8, -12, 0),
	   square_size * Point3f(-7, -11, 0),  square_size * Point3f(-8, -11, 0),  square_size * Point3f(-4, -11, 0),
	   square_size * Point3f(-4, -10, 0),  square_size * Point3f(-5, -10, 0),  square_size * Point3f(-10, -13, 0),
	   square_size * Point3f(-10, -12, 0), square_size * Point3f(-11, -12, 0), square_size * Point3f(-9, -12, 0),
	   square_size * Point3f(-9, -11, 0),  square_size * Point3f(-10, -11, 0), square_size * Point3f(-6, -10, 0),
	   square_size * Point3f(-7, -10, 0),  square_size * Point3f(7, -11, 0),   square_size * Point3f(8, -10, 0),
	   square_size * Point3f(7, -10, 0),   square_size * Point3f(-1, -9, 0),   square_size * Point3f(-2, -9, 0),
	   square_size * Point3f(1, -10, 0),   square_size * Point3f(1, -9, 0),    square_size * Point3f(0, -9, 0),
	   square_size * Point3f(2, -10, 0),   square_size * Point3f(3, -10, 0),   square_size * Point3f(3, -9, 0),
	   square_size * Point3f(2, -9, 0),    square_size * Point3f(-3, -9, 0),   square_size * Point3f(-4, -9, 0),
	   square_size * Point3f(-8, -10, 0),  square_size * Point3f(-9, -10, 0),  square_size * Point3f(4, -10, 0),
	   square_size * Point3f(5, -10, 0),   square_size * Point3f(5, -9, 0),    square_size * Point3f(4, -9, 0),
	   square_size * Point3f(6, -10, 0),   square_size * Point3f(7, -9, 0),    square_size * Point3f(6, -9, 0),
	   square_size * Point3f(-5, -9, 0),   square_size * Point3f(-6, -9, 0),   square_size * Point3f(-11, -11, 0),
	   square_size * Point3f(-10, -10, 0), square_size * Point3f(-11, -10, 0), square_size * Point3f(-7, -9, 0),
	   square_size * Point3f(-8, -9, 0),   square_size * Point3f(2, -8, 0),    square_size * Point3f(1, -8, 0),
	   square_size * Point3f(4, -8, 0),    square_size * Point3f(3, -8, 0),    square_size * Point3f(0, -8, 0),
	   square_size * Point3f(-1, -8, 0),   square_size * Point3f(6, -8, 0),    square_size * Point3f(5, -8, 0),
	   square_size * Point3f(-9, -9, 0),   square_size * Point3f(-10, -9, 0),  square_size * Point3f(-4, -8, 0),
	   square_size * Point3f(-5, -8, 0),   square_size * Point3f(-11, -9, 0),  square_size * Point3f(-6, -8, 0),
	   square_size * Point3f(-7, -8, 0),   square_size * Point3f(7, -8, 0),    square_size * Point3f(7, -7, 0),
	   square_size * Point3f(6, -7, 0),    square_size * Point3f(5, -7, 0),    square_size * Point3f(4, -7, 0),
	   square_size * Point3f(-8, -8, 0),   square_size * Point3f(-9, -8, 0),   square_size * Point3f(3, -7, 0),
	   square_size * Point3f(2, -7, 0),    square_size * Point3f(-10, -8, 0),  square_size * Point3f(-11, -8, 0),
	   square_size * Point3f(1, -7, 0),    square_size * Point3f(0, -7, 0),    square_size * Point3f(8, -7, 0),
	   square_size * Point3f(7, -6, 0),    square_size * Point3f(-3, -8, 0),   square_size * Point3f(-3, -7, 0),
	   square_size * Point3f(-4, -7, 0),   square_size * Point3f(6, -6, 0),    square_size * Point3f(5, -6, 0),
	   square_size * Point3f(-5, -7, 0),   square_size * Point3f(-6, -7, 0),   square_size * Point3f(-7, -7, 0),
	   square_size * Point3f(-8, -7, 0),   square_size * Point3f(-9, -7, 0),   square_size * Point3f(-10, -7, 0),
	   square_size * Point3f(-12, -8, 0),  square_size * Point3f(-11, -7, 0),  square_size * Point3f(4, -6, 0),
	   square_size * Point3f(3, -6, 0),    square_size * Point3f(2, -6, 0),    square_size * Point3f(1, -6, 0),
	   square_size * Point3f(6, -5, 0),    square_size * Point3f(-1, -7, 0),   square_size * Point3f(0, -6, 0),
	   square_size * Point3f(-1, -6, 0),   square_size * Point3f(5, -5, 0),    square_size * Point3f(4, -5, 0),
	   square_size * Point3f(-10, -6, 0),  square_size * Point3f(-11, -6, 0),  square_size * Point3f(-8, -6, 0),
	   square_size * Point3f(-9, -6, 0),   square_size * Point3f(-6, -6, 0),   square_size * Point3f(-7, -6, 0),
	   square_size * Point3f(-4, -6, 0),   square_size * Point3f(-5, -6, 0),   square_size * Point3f(3, -5, 0),
	   square_size * Point3f(2, -5, 0),    square_size * Point3f(6, -4, 0),    square_size * Point3f(5, -4, 0),
	   square_size * Point3f(1, -5, 0),    square_size * Point3f(0, -5, 0),    square_size * Point3f(-9, -5, 0),
	   square_size * Point3f(-10, -5, 0),  square_size * Point3f(4, -4, 0),    square_size * Point3f(3, -4, 0),
	   square_size * Point3f(-7, -5, 0),   square_size * Point3f(-8, -5, 0),   square_size * Point3f(7, -4, 0),
	   square_size * Point3f(6, -3, 0),    square_size * Point3f(-5, -5, 0),   square_size * Point3f(-6, -5, 0),
	   square_size * Point3f(-3, -6, 0),   square_size * Point3f(-3, -5, 0),   square_size * Point3f(-4, -5, 0),
	   square_size * Point3f(-11, -5, 0),  square_size * Point3f(-10, -4, 0),  square_size * Point3f(2, -4, 0),
	   square_size * Point3f(1, -4, 0),    square_size * Point3f(5, -3, 0),    square_size * Point3f(4, -3, 0),
	   square_size * Point3f(-8, -4, 0),   square_size * Point3f(-9, -4, 0),   square_size * Point3f(-6, -4, 0),
	   square_size * Point3f(-7, -4, 0),   square_size * Point3f(-1, -5, 0),   square_size * Point3f(0, -4, 0),
	   square_size * Point3f(-1, -4, 0),   square_size * Point3f(3, -3, 0),    square_size * Point3f(2, -3, 0),
	   square_size * Point3f(-9, -3, 0),   square_size * Point3f(-10, -3, 0),  square_size * Point3f(-4, -4, 0),
	   square_size * Point3f(-5, -4, 0),   square_size * Point3f(4, -2, 0),    square_size * Point3f(3, -2, 0),
	   square_size * Point3f(-7, -3, 0),   square_size * Point3f(-8, -3, 0),   square_size * Point3f(1, -3, 0),
	   square_size * Point3f(0, -3, 0),    square_size * Point3f(-5, -3, 0),   square_size * Point3f(-6, -3, 0),
	   square_size * Point3f(2, -2, 0),    square_size * Point3f(1, -2, 0),    square_size * Point3f(-8, -2, 0),
	   square_size * Point3f(-9, -2, 0),   square_size * Point3f(-3, -4, 0),   square_size * Point3f(-3, -3, 0),
	   square_size * Point3f(-4, -3, 0),   square_size * Point3f(3, -1, 0),    square_size * Point3f(2, -1, 0),
	   square_size * Point3f(-6, -2, 0),   square_size * Point3f(-7, -2, 0),   square_size * Point3f(-1, -3, 0),
	   square_size * Point3f(0, -2, 0),    square_size * Point3f(-1, -2, 0),   square_size * Point3f(-4, -2, 0),
	   square_size * Point3f(-5, -2, 0),   square_size * Point3f(-7, -1, 0),   square_size * Point3f(-8, -1, 0),
	   square_size * Point3f(1, -1, 0),    square_size * Point3f(0, -1, 0),    square_size * Point3f(-9, -1, 0),
	   square_size * Point3f(2, 0, 0),     square_size * Point3f(1, 0, 0),     square_size * Point3f(-5, -1, 0),
	   square_size * Point3f(-6, -1, 0),   square_size * Point3f(3, 0, 0),     square_size * Point3f(-6, 0, 0),
	   square_size * Point3f(-7, 0, 0),    square_size * Point3f(-1, -1, 0),   square_size * Point3f(0, 0, 0),
	   square_size * Point3f(-1, 0, 0),    square_size * Point3f(-4, -1, 0),   square_size * Point3f(-4, 0, 0),
	   square_size * Point3f(-5, 0, 0),    square_size * Point3f(0, 1, 0),     square_size * Point3f(-5, 1, 0),
	   square_size * Point3f(-4, 1, 0)};

	vector<Point2f> ivec = {
	   Point2f(693.624451, 367.008148),   Point2f(764.948608, 410.001648),   Point2f(680.829712, 435.906616),
	   Point2f(566.229004, 422.309143),   Point2f(623.382263, 395.337646),   Point2f(610.419800, 462.446411),
	   Point2f(551.776917, 488.819519),   Point2f(671.644958, 517.376404),   Point2f(599.580261, 541.497925),
	   Point2f(1081.820312, 456.528900),  Point2f(1185.194092, 459.105103),  Point2f(1203.258545, 555.748230),
	   Point2f(1095.229736, 555.710144),  Point2f(860.019226, 474.179626),   Point2f(970.164062, 460.906128),
	   Point2f(978.006897, 562.069519),   Point2f(861.921448, 575.092834),   Point2f(504.090149, 514.003784),
	   Point2f(541.363892, 564.755737),   Point2f(493.966644, 586.166931),   Point2f(758.488342, 494.308990),
	   Point2f(756.431458, 592.179504),   Point2f(667.030701, 611.633057),   Point2f(443.371643, 498.707153),
	   Point2f(432.609772, 557.791870),   Point2f(405.606812, 576.933472),   Point2f(464.525360, 537.218201),
	   Point2f(456.265289, 605.213074),   Point2f(425.246063, 622.145020),   Point2f(594.366760, 630.557983),
	   Point2f(536.030762, 648.263672),   Point2f(1569.247437, 543.342773),  Point2f(1603.293701, 611.463074),
	   Point2f(1581.594238, 605.054626),  Point2f(1107.443237, 669.694946),  Point2f(987.370056, 677.921326),
	   Point2f(1295.573608, 560.179077),  Point2f(1310.023071, 663.068420),  Point2f(1216.940186, 664.874146),
	   Point2f(1372.040283, 567.403748),  Point2f(1432.752563, 575.507141),  Point2f(1445.460205, 663.584534),
	   Point2f(1385.299805, 662.875122),  Point2f(868.128052, 689.102600),   Point2f(760.197205, 701.855530),
	   Point2f(489.755798, 664.158325),   Point2f(452.327972, 678.250854),   Point2f(1482.960693, 583.873291),
	   Point2f(1521.276001, 591.863037),  Point2f(1532.465088, 667.396790),  Point2f(1493.656006, 665.290588),
	   Point2f(1555.323486, 599.962280),  Point2f(1589.515381, 670.979004),  Point2f(1563.273193, 669.102661),
	   Point2f(669.492004, 714.634399),   Point2f(596.000977, 726.604492),   Point2f(399.551056, 637.359863),
	   Point2f(422.371613, 690.432373),   Point2f(397.025696, 701.347717),   Point2f(537.879456, 737.159607),
	   Point2f(491.487152, 746.226196),   Point2f(1391.550903, 761.704590),  Point2f(1316.785522, 769.634521),
	   Point2f(1498.431763, 749.121460),  Point2f(1450.763550, 754.899170),  Point2f(1224.874268, 779.072083),
	   Point2f(1116.346191, 789.094238),  Point2f(1567.318970, 740.383240),  Point2f(1536.348389, 744.074097),
	   Point2f(454.585114, 754.110657),   Point2f(424.530579, 760.567017),   Point2f(770.633179, 815.309448),
	   Point2f(679.605896, 820.711365),   Point2f(399.692139, 766.388611),   Point2f(605.934570, 824.523804),
	   Point2f(547.230835, 827.148071),   Point2f(1592.534912, 737.059326),  Point2f(1591.335693, 802.440308),
	   Point2f(1565.716431, 810.761414),  Point2f(1534.554565, 820.363525),  Point2f(1496.291382, 831.116211),
	   Point2f(500.303345, 829.207703),   Point2f(462.548096, 830.469360),   Point2f(1448.739990, 843.635864),
	   Point2f(1389.606934, 857.499817),  Point2f(432.063965, 831.449097),   Point2f(406.886139, 831.985596),
	   Point2f(1315.996704, 872.714722),  Point2f(1226.211426, 888.306946),  Point2f(1612.769531, 795.531494),
	   Point2f(1585.567871, 865.797974),  Point2f(878.377563, 808.178101),   Point2f(892.157288, 920.882385),
	   Point2f(786.930786, 923.018188),   Point2f(1559.048706, 878.366821),  Point2f(1527.044312, 892.692383),
	   Point2f(697.009155, 921.771118),   Point2f(623.016296, 918.374084),   Point2f(563.431946, 914.068176),
	   Point2f(515.402954, 909.299438),   Point2f(476.515686, 904.764038),   Point2f(444.814240, 900.184021),
	   Point2f(385.747314, 832.634216),   Point2f(418.445923, 896.147705),   Point2f(1488.178467, 908.818542),
	   Point2f(1439.969971, 926.719604),  Point2f(1381.138062, 945.793213),  Point2f(1308.623779, 966.263367),
	   Point2f(1548.451904, 941.724670),  Point2f(1121.353760, 902.658630),  Point2f(1222.336182, 985.516785),
	   Point2f(1122.601685, 1002.308777), Point2f(1515.273438, 959.763977),  Point2f(1475.364502, 979.426880),
	   Point2f(461.974854, 965.781982),   Point2f(433.533081, 957.387207),   Point2f(535.721802, 984.279663),
	   Point2f(495.229156, 974.403015),   Point2f(645.571655, 1003.880676),  Point2f(585.064209, 993.979370),
	   Point2f(806.740845, 1018.085266),  Point2f(719.064087, 1012.357788),  Point2f(1426.661377, 1000.822327),
	   Point2f(1367.694824, 1023.809570), Point2f(1534.424316, 1000.141541), Point2f(1500.604614, 1020.025085),
	   Point2f(1297.661255, 1046.210205), Point2f(1214.916260, 1067.362061), Point2f(517.488892, 1038.620117),
	   Point2f(482.437012, 1026.082031),  Point2f(1459.443481, 1042.406250), Point2f(1410.252563, 1066.266357),
	   Point2f(610.241577, 1065.339233),  Point2f(559.420410, 1051.606689),  Point2f(1563.391846, 981.447937),
	   Point2f(1518.880005, 1052.108276), Point2f(743.271851, 1089.513672),  Point2f(670.799316, 1078.114380),
	   Point2f(907.062561, 1019.037598),  Point2f(921.701233, 1100.214233),  Point2f(827.410095, 1097.544556),
	   Point2f(453.243073, 1015.048218),  Point2f(505.500488, 1081.581177),  Point2f(1352.389893, 1089.754883),
	   Point2f(1284.166138, 1113.349121), Point2f(1483.089478, 1074.465454), Point2f(1441.341797, 1097.868408),
	   Point2f(585.142334, 1111.242920),  Point2f(542.003296, 1095.860962),  Point2f(696.899048, 1141.093872),
	   Point2f(636.617310, 1126.480103),  Point2f(1121.672607, 1084.713989), Point2f(1205.963379, 1134.358154),
	   Point2f(1119.151001, 1151.244995), Point2f(1392.687500, 1121.574951), Point2f(1335.710083, 1145.771851),
	   Point2f(567.172729, 1146.343750),  Point2f(529.237000, 1130.453979),  Point2f(847.347717, 1162.222656),
	   Point2f(767.423035, 1153.504517),  Point2f(1423.097412, 1145.247559), Point2f(1374.843140, 1168.966309),
	   Point2f(662.867859, 1178.672241),  Point2f(611.696899, 1162.629395),  Point2f(1270.387451, 1168.402832),
	   Point2f(1196.678467, 1188.595215), Point2f(790.202271, 1205.893433),  Point2f(722.464844, 1193.343140),
	   Point2f(1319.367554, 1192.087402), Point2f(1256.805420, 1213.829224), Point2f(637.440063, 1206.850464),
	   Point2f(593.108093, 1190.340210),  Point2f(935.361694, 1165.341064),  Point2f(949.626404, 1217.468262),
	   Point2f(865.659607, 1214.204346),  Point2f(1357.207153, 1208.970459), Point2f(1303.598877, 1231.098877),
	   Point2f(746.295105, 1236.873291),  Point2f(688.437378, 1222.491699),  Point2f(1116.287476, 1204.295898),
	   Point2f(1187.718018, 1232.362427), Point2f(1113.179443, 1246.742188), Point2f(881.889404, 1256.213745),
	   Point2f(811.097595, 1248.245605),  Point2f(712.158508, 1259.410767),  Point2f(662.150818, 1244.201294),
	   Point2f(1244.296997, 1250.901611), Point2f(1179.301270, 1268.110962), Point2f(618.136047, 1228.697266),
	   Point2f(1289.028931, 1263.854370), Point2f(1232.560791, 1282.092163), Point2f(829.695435, 1283.143066),
	   Point2f(768.238525, 1272.609131),  Point2f(1340.580078, 1243.334229), Point2f(788.199402, 1302.594482),
	   Point2f(734.408203, 1290.585693),  Point2f(1110.094482, 1281.174927), Point2f(1171.511719, 1297.395020),
	   Point2f(1107.297485, 1309.272095), Point2f(896.272949, 1289.967529),  Point2f(908.914917, 1317.893188),
	   Point2f(846.664246, 1311.934326),  Point2f(1164.576782, 1322.033936), Point2f(861.476685, 1335.848511),
	   Point2f(920.622009, 1342.062622)};

	w = wvec;
	i = ivec;
}

void initImgPoints2(vector<Point3f> &w, vector<Point2f> &i) {
	vector<Point3f> wvec = {
	   square_size * Point3f(-5, -7, 0), square_size * Point3f(-4, -7, 0), square_size * Point3f(-4, -6, 0),
	   square_size * Point3f(-5, -6, 0), square_size * Point3f(-3, -6, 0), square_size * Point3f(-3, -5, 0),
	   square_size * Point3f(-4, -5, 0), square_size * Point3f(-6, -6, 0), square_size * Point3f(-5, -5, 0),
	   square_size * Point3f(-6, -5, 0), square_size * Point3f(-2, -5, 0), square_size * Point3f(-2, -4, 0),
	   square_size * Point3f(-3, -4, 0), square_size * Point3f(-4, -4, 0), square_size * Point3f(-5, -4, 0),
	   square_size * Point3f(3, -6, 0),  square_size * Point3f(3, -5, 0),  square_size * Point3f(2, -5, 0),
	   square_size * Point3f(-7, -5, 0), square_size * Point3f(-6, -4, 0), square_size * Point3f(-7, -4, 0),
	   square_size * Point3f(-1, -4, 0), square_size * Point3f(-1, -3, 0), square_size * Point3f(-2, -3, 0),
	   square_size * Point3f(-3, -3, 0), square_size * Point3f(-4, -3, 0), square_size * Point3f(-9, -5, 0),
	   square_size * Point3f(-8, -5, 0), square_size * Point3f(-8, -4, 0), square_size * Point3f(-9, -4, 0),
	   square_size * Point3f(-5, -3, 0), square_size * Point3f(-6, -3, 0), square_size * Point3f(4, -5, 0),
	   square_size * Point3f(4, -4, 0),  square_size * Point3f(3, -4, 0),  square_size * Point3f(-7, -3, 0),
	   square_size * Point3f(-8, -3, 0), square_size * Point3f(5, -5, 0),  square_size * Point3f(6, -4, 0),
	   square_size * Point3f(5, -4, 0),  square_size * Point3f(2, -4, 0),  square_size * Point3f(3, -3, 0),
	   square_size * Point3f(2, -3, 0),  square_size * Point3f(-4, -2, 0), square_size * Point3f(-5, -2, 0),
	   square_size * Point3f(0, -3, 0),  square_size * Point3f(0, -2, 0),  square_size * Point3f(-1, -2, 0),
	   square_size * Point3f(5, -3, 0),  square_size * Point3f(4, -3, 0),  square_size * Point3f(-6, -2, 0),
	   square_size * Point3f(-7, -2, 0), square_size * Point3f(1, -3, 0),  square_size * Point3f(2, -2, 0),
	   square_size * Point3f(1, -2, 0),  square_size * Point3f(4, -2, 0),  square_size * Point3f(3, -2, 0),
	   square_size * Point3f(-8, -2, 0), square_size * Point3f(-7, -1, 0), square_size * Point3f(-8, -1, 0),
	   square_size * Point3f(-5, -1, 0), square_size * Point3f(-6, -1, 0), square_size * Point3f(-3, -2, 0),
	   square_size * Point3f(-3, -1, 0), square_size * Point3f(-4, -1, 0), square_size * Point3f(1, -1, 0),
	   square_size * Point3f(0, -1, 0),  square_size * Point3f(3, -1, 0),  square_size * Point3f(2, -1, 0),
	   square_size * Point3f(5, -2, 0),  square_size * Point3f(5, -1, 0),  square_size * Point3f(4, -1, 0),
	   square_size * Point3f(-9, -1, 0), square_size * Point3f(-8, 0, 0),  square_size * Point3f(-9, 0, 0),
	   square_size * Point3f(-6, 0, 0),  square_size * Point3f(-7, 0, 0),  square_size * Point3f(6, -1, 0),
	   square_size * Point3f(5, 0, 0),   square_size * Point3f(4, 0, 0),   square_size * Point3f(3, 0, 0),
	   square_size * Point3f(-4, 0, 0),  square_size * Point3f(-5, 0, 0),  square_size * Point3f(2, 0, 0),
	   square_size * Point3f(1, 0, 0),   square_size * Point3f(-1, -1, 0), square_size * Point3f(0, 0, 0),
	   square_size * Point3f(-1, 0, 0),  square_size * Point3f(-7, 1, 0),  square_size * Point3f(-8, 1, 0),
	   square_size * Point3f(3, 1, 0),   square_size * Point3f(2, 1, 0),   square_size * Point3f(1, 1, 0),
	   square_size * Point3f(0, 1, 0),   square_size * Point3f(-3, 0, 0),  square_size * Point3f(-3, 1, 0),
	   square_size * Point3f(-4, 1, 0),  square_size * Point3f(2, 2, 0),   square_size * Point3f(1, 2, 0),
	   square_size * Point3f(-1, 1, 0),  square_size * Point3f(0, 2, 0),   square_size * Point3f(-1, 2, 0),
	   square_size * Point3f(1, 3, 0),   square_size * Point3f(0, 3, 0),   square_size * Point3f(-1, 3, 0),
	   square_size * Point3f(0, 4, 0),   square_size * Point3f(-1, 4, 0),  square_size * Point3f(1, 4, 0),
	   square_size * Point3f(0, 5, 0)};

	vector<Point2f> ivec = {
	   Point2f(629.423950, 142.109497),   Point2f(740.113464, 103.697090),   Point2f(707.441040, 175.901138),
	   Point2f(590.900757, 220.909821),   Point2f(852.907593, 145.633087),   Point2f(828.838501, 246.770584),
	   Point2f(672.489136, 280.479279),   Point2f(502.168518, 269.653137),   Point2f(554.342041, 325.923523),
	   Point2f(469.508453, 371.716675),   Point2f(1011.126953, 240.601227),  Point2f(1004.518921, 386.702942),
	   Point2f(806.741577, 391.968933),   Point2f(644.233459, 421.176910),   Point2f(528.037170, 458.194153),
	   Point2f(1556.958618, 367.832336),  Point2f(1575.859619, 466.204102),  Point2f(1519.370117, 419.238922),
	   Point2f(409.480042, 412.513123),   Point2f(448.470459, 492.809784),   Point2f(393.302277, 522.510193),
	   Point2f(1197.373779, 411.720428),  Point2f(1193.693970, 594.189453),  Point2f(996.568115, 577.965271),
	   Point2f(795.087830, 578.477966),   Point2f(633.425110, 592.234253),   Point2f(333.568726, 476.987579),
	   Point2f(365.820557, 447.335052),   Point2f(353.810944, 547.254028),   Point2f(324.673798, 567.523926),
	   Point2f(520.717529, 609.833313),   Point2f(444.273956, 626.436035),   Point2f(1612.290039, 497.937561),
	   Point2f(1615.403320, 593.241577),  Point2f(1579.042603, 567.631104),  Point2f(391.712616, 640.376953),
	   Point2f(354.003571, 651.821716),   Point2f(1643.231812, 531.835754),  Point2f(1662.618530, 630.972351),
	   Point2f(1642.150024, 614.143066),  Point2f(1530.199219, 538.620117),  Point2f(1571.927368, 679.360352),
	   Point2f(1522.222168, 662.803955),  Point2f(645.832153, 767.242188),   Point2f(535.461487, 763.009460),
	   Point2f(1346.101685, 618.512390),  Point2f(1322.574585, 785.697083),  Point2f(1175.197876, 782.638672),
	   Point2f(1633.948364, 702.540100),  Point2f(1607.327881, 692.050842),  Point2f(459.113037, 759.749817),
	   Point2f(405.625061, 757.317444),   Point2f(1451.366577, 642.729187),  Point2f(1499.943726, 788.601990),
	   Point2f(1427.061890, 787.431641),  Point2f(1588.641357, 789.223267),  Point2f(1550.924927, 789.028137),
	   Point2f(366.698639, 755.449707),   Point2f(432.061951, 864.920410),   Point2f(390.197327, 852.066284),
	   Point2f(567.177551, 897.949890),   Point2f(488.788574, 880.176025),   Point2f(800.089905, 772.512878),
	   Point2f(816.847717, 932.875427),   Point2f(674.777161, 916.601868),   Point2f(1389.714355, 915.140259),
	   Point2f(1286.139160, 928.724915),  Point2f(1519.839355, 890.010742),  Point2f(1464.586426, 902.293396),
	   Point2f(1616.835449, 789.652771),  Point2f(1592.925293, 871.479614),  Point2f(1560.863159, 880.096680),
	   Point2f(358.248566, 841.446716),   Point2f(420.609894, 937.999084),   Point2f(384.681183, 920.752747),
	   Point2f(526.470947, 980.810181),   Point2f(466.529449, 958.167847),   Point2f(1611.000000, 870.000000),
	   Point2f(1562.255005, 946.142456),  Point2f(1527.591797, 960.493896),  Point2f(1483.398682, 977.159241),
	   Point2f(708.870911, 1029.704712),  Point2f(605.800598, 1005.508423),  Point2f(1424.368164, 996.965088),
	   Point2f(1347.483765, 1017.703979), Point2f(1148.944946, 939.076172),  Point2f(1247.715210, 1037.541382),
	   Point2f(1123.101807, 1052.371948), Point2f(504.002350, 1035.429688),  Point2f(455.262146, 1011.729126),
	   Point2f(1443.572510, 1050.603271), Point2f(1381.919556, 1074.380005), Point2f(1306.984009, 1095.961304),
	   Point2f(1212.841187, 1116.745728), Point2f(836.550964, 1048.657471),  Point2f(854.449097, 1128.891479),
	   Point2f(740.823364, 1111.235474),  Point2f(1344.568481, 1132.899536), Point2f(1270.776245, 1155.278320),
	   Point2f(1101.489990, 1131.419189), Point2f(1183.617554, 1174.028320), Point2f(1083.982910, 1187.140259),
	   Point2f(1239.971313, 1199.658936), Point2f(1159.409058, 1216.519043), Point2f(1070.276733, 1227.460449),
	   Point2f(1139.614624, 1248.360962), Point2f(1059.224365, 1257.886963), Point2f(1213.756714, 1234.180664),
	   Point2f(1122.682495, 1274.020996)};

	w = wvec;
	i = ivec;
}

void initImgPoints3(vector<Point3f> &w, vector<Point2f> &i) {
	vector<Point3f> wvec = {
	   square_size * Point3f(0, -14, 0),  square_size * Point3f(1, -14, 0),  square_size * Point3f(1, -13, 0),
	   square_size * Point3f(0, -13, 0),  square_size * Point3f(2, -14, 0),  square_size * Point3f(3, -14, 0),
	   square_size * Point3f(3, -13, 0),  square_size * Point3f(2, -13, 0),  square_size * Point3f(-1, -13, 0),
	   square_size * Point3f(0, -12, 0),  square_size * Point3f(-1, -12, 0), square_size * Point3f(4, -14, 0),
	   square_size * Point3f(5, -14, 0),  square_size * Point3f(5, -13, 0),  square_size * Point3f(4, -13, 0),
	   square_size * Point3f(1, -12, 0),  square_size * Point3f(1, -11, 0),  square_size * Point3f(0, -11, 0),
	   square_size * Point3f(4, -12, 0),  square_size * Point3f(3, -12, 0),  square_size * Point3f(-2, -12, 0),
	   square_size * Point3f(-1, -11, 0), square_size * Point3f(-2, -11, 0), square_size * Point3f(6, -13, 0),
	   square_size * Point3f(6, -12, 0),  square_size * Point3f(5, -12, 0),  square_size * Point3f(0, -10, 0),
	   square_size * Point3f(-1, -10, 0), square_size * Point3f(5, -11, 0),  square_size * Point3f(4, -11, 0),
	   square_size * Point3f(2, -11, 0),  square_size * Point3f(2, -10, 0),  square_size * Point3f(1, -10, 0),
	   square_size * Point3f(-3, -11, 0), square_size * Point3f(-2, -10, 0), square_size * Point3f(-3, -10, 0),
	   square_size * Point3f(3, -11, 0),  square_size * Point3f(4, -10, 0),  square_size * Point3f(3, -10, 0),
	   square_size * Point3f(7, -12, 0),  square_size * Point3f(7, -11, 0),  square_size * Point3f(6, -11, 0),
	   square_size * Point3f(6, -10, 0),  square_size * Point3f(5, -10, 0),  square_size * Point3f(1, -9, 0),
	   square_size * Point3f(0, -9, 0),   square_size * Point3f(-1, -9, 0),  square_size * Point3f(-2, -9, 0),
	   square_size * Point3f(3, -9, 0),   square_size * Point3f(2, -9, 0),   square_size * Point3f(5, -9, 0),
	   square_size * Point3f(4, -9, 0),   square_size * Point3f(-4, -10, 0), square_size * Point3f(-3, -9, 0),
	   square_size * Point3f(-4, -9, 0),  square_size * Point3f(7, -10, 0),  square_size * Point3f(7, -9, 0),
	   square_size * Point3f(6, -9, 0),   square_size * Point3f(2, -8, 0),   square_size * Point3f(1, -8, 0),
	   square_size * Point3f(11, -10, 0), square_size * Point3f(0, -8, 0),   square_size * Point3f(-1, -8, 0),
	   square_size * Point3f(8, -10, 0),  square_size * Point3f(9, -10, 0),  square_size * Point3f(9, -9, 0),
	   square_size * Point3f(8, -9, 0),   square_size * Point3f(4, -8, 0),   square_size * Point3f(3, -8, 0),
	   square_size * Point3f(10, -10, 0), square_size * Point3f(10, -9, 0),  square_size * Point3f(6, -8, 0),
	   square_size * Point3f(5, -8, 0),   square_size * Point3f(-2, -8, 0),  square_size * Point3f(-3, -8, 0),
	   square_size * Point3f(8, -8, 0),   square_size * Point3f(7, -8, 0),   square_size * Point3f(10, -8, 0),
	   square_size * Point3f(9, -8, 0),   square_size * Point3f(-5, -9, 0),  square_size * Point3f(-4, -8, 0),
	   square_size * Point3f(-5, -8, 0),  square_size * Point3f(3, -7, 0),   square_size * Point3f(2, -7, 0),
	   square_size * Point3f(5, -7, 0),   square_size * Point3f(4, -7, 0),   square_size * Point3f(1, -7, 0),
	   square_size * Point3f(0, -7, 0),   square_size * Point3f(7, -7, 0),   square_size * Point3f(6, -7, 0),
	   square_size * Point3f(9, -7, 0),   square_size * Point3f(8, -7, 0),   square_size * Point3f(-1, -7, 0),
	   square_size * Point3f(-2, -7, 0),  square_size * Point3f(-3, -7, 0),  square_size * Point3f(-4, -7, 0),
	   square_size * Point3f(10, -7, 0),  square_size * Point3f(9, -6, 0),   square_size * Point3f(11, -6, 0),
	   square_size * Point3f(8, -6, 0),   square_size * Point3f(7, -6, 0),   square_size * Point3f(6, -6, 0),
	   square_size * Point3f(5, -6, 0),   square_size * Point3f(4, -6, 0),   square_size * Point3f(3, -6, 0),
	   square_size * Point3f(2, -6, 0),   square_size * Point3f(1, -6, 0),   square_size * Point3f(-6, -8, 0),
	   square_size * Point3f(-5, -7, 0),  square_size * Point3f(-6, -7, 0),  square_size * Point3f(10, -5, 0),
	   square_size * Point3f(0, -6, 0),   square_size * Point3f(-1, -6, 0),  square_size * Point3f(9, -5, 0),
	   square_size * Point3f(8, -5, 0),   square_size * Point3f(7, -5, 0),   square_size * Point3f(6, -5, 0),
	   square_size * Point3f(13, -4, 0),  square_size * Point3f(-2, -6, 0),  square_size * Point3f(-3, -6, 0),
	   square_size * Point3f(9, -4, 0),   square_size * Point3f(3, -5, 0),   square_size * Point3f(2, -5, 0),
	   square_size * Point3f(8, -4, 0),   square_size * Point3f(7, -4, 0),   square_size * Point3f(-4, -6, 0),
	   square_size * Point3f(-5, -6, 0),  square_size * Point3f(12, -3, 0),  square_size * Point3f(1, -5, 0),
	   square_size * Point3f(0, -5, 0),   square_size * Point3f(5, -5, 0),   square_size * Point3f(6, -4, 0),
	   square_size * Point3f(8, -3, 0),   square_size * Point3f(-1, -5, 0),  square_size * Point3f(-2, -5, 0),
	   square_size * Point3f(12, -2, 0),  square_size * Point3f(7, -3, 0),   square_size * Point3f(6, -3, 0),
	   square_size * Point3f(9, -2, 0),   square_size * Point3f(2, -4, 0),   square_size * Point3f(1, -4, 0),
	   square_size * Point3f(-3, -5, 0),  square_size * Point3f(-4, -5, 0),  square_size * Point3f(8, -2, 0),
	   square_size * Point3f(7, -2, 0),   square_size * Point3f(0, -4, 0),   square_size * Point3f(-1, -4, 0),
	   square_size * Point3f(3, -4, 0),   square_size * Point3f(3, -3, 0),   square_size * Point3f(2, -3, 0),
	   square_size * Point3f(8, -1, 0),   square_size * Point3f(-2, -4, 0),  square_size * Point3f(-3, -4, 0),
	   square_size * Point3f(1, -3, 0),   square_size * Point3f(0, -3, 0),   square_size * Point3f(-1, -3, 0),
	   square_size * Point3f(-2, -3, 0),  square_size * Point3f(2, -2, 0),   square_size * Point3f(1, -2, 0),
	   square_size * Point3f(3, -2, 0),   square_size * Point3f(3, -1, 0),   square_size * Point3f(2, -1, 0),
	   square_size * Point3f(-4, -4, 0),  square_size * Point3f(-3, -3, 0),  square_size * Point3f(-4, -3, 0),
	   square_size * Point3f(0, -2, 0),   square_size * Point3f(-1, -2, 0),  square_size * Point3f(1, -1, 0),
	   square_size * Point3f(0, -1, 0),   square_size * Point3f(2, 0, 0),    square_size * Point3f(1, 0, 0),
	   square_size * Point3f(-2, -2, 0),  square_size * Point3f(-1, -1, 0),  square_size * Point3f(-2, -1, 0),
	   square_size * Point3f(3, 0, 0),    square_size * Point3f(0, 0, 0),    square_size * Point3f(-1, 0, 0),
	   square_size * Point3f(0, 1, 0),    square_size * Point3f(-1, 1, 0)};

	vector<Point2f> ivec = {
	   Point2f(788.491394, 24.073915),   Point2f(893.666870, 43.010380),   Point2f(867.485657, 108.511879),
	   Point2f(755.716248, 91.594383),   Point2f(989.953064, 73.579994),   Point2f(1072.870972, 111.359497),
	   Point2f(1056.462036, 173.956253), Point2f(968.764160, 137.641876),  Point2f(641.403442, 90.753960),
	   Point2f(725.235779, 176.210800),  Point2f(607.038513, 178.833603),  Point2f(1142.982422, 151.800400),
	   Point2f(1200.117188, 191.878220), Point2f(1188.040405, 250.112503), Point2f(1128.813721, 212.333572),
	   Point2f(841.323792, 189.777557),  Point2f(817.963074, 285.742126),  Point2f(699.936646, 277.474426),
	   Point2f(1112.862671, 281.728394), Point2f(1037.949219, 247.488541), Point2f(496.282104, 197.269882),
	   Point2f(581.090027, 283.261841),  Point2f(471.263550, 303.117615),  Point2f(1235.287109, 284.982910),
	   Point2f(1221.629517, 345.092896), Point2f(1173.296265, 315.071869), Point2f(683.313416, 389.837708),
	   Point2f(567.042969, 398.890198),  Point2f(1156.541260, 385.470612), Point2f(1095.035767, 358.717529),
	   Point2f(925.979309, 305.310181),  Point2f(906.911682, 403.955872),  Point2f(799.653687, 392.204559),
	   Point2f(377.085236, 333.346802),  Point2f(460.513428, 418.475769),  Point2f(370.119110, 445.572205),
	   Point2f(1018.757935, 330.949646), Point2f(1076.229004, 440.481354), Point2f(999.515320, 421.154449),
	   Point2f(1260.469971, 372.836456), Point2f(1245.468628, 431.797668), Point2f(1205.923828, 410.192230),
	   Point2f(1188.194458, 477.052460), Point2f(1138.113892, 459.467987), Point2f(787.741699, 501.512695),
	   Point2f(676.547729, 505.076416),  Point2f(566.366943, 516.288574),  Point2f(465.199371, 534.528870),
	   Point2f(981.438416, 512.734680),  Point2f(891.091797, 504.897583),  Point2f(1118.870483, 533.778076),
	   Point2f(1057.014160, 522.955933), Point2f(296.533142, 476.595490),  Point2f(378.540405, 557.317383),
	   Point2f(307.960052, 582.307983),  Point2f(1228.440186, 493.247803), Point2f(1210.358765, 554.103943),
	   Point2f(1169.157227, 544.392273), Point2f(879.330688, 601.145630),  Point2f(782.354675, 605.084473),
	   Point2f(1331.296997, 541.212646), Point2f(679.028076, 613.291443),  Point2f(576.642517, 626.264038),
	   Point2f(1261.990967, 507.508301), Point2f(1288.633911, 519.859680), Point2f(1272.333008, 570.856323),
	   Point2f(1244.175659, 562.892212), Point2f(1038.527710, 602.671631), Point2f(965.245300, 600.769226),
	   Point2f(1312.305786, 531.412354), Point2f(1295.697632, 577.920654), Point2f(1149.623901, 609.479309),
	   Point2f(1099.467896, 605.748901), Point2f(482.145081, 642.628601),  Point2f(400.082794, 661.259216),
	   Point2f(1225.634888, 616.993042), Point2f(1191.133301, 613.273438), Point2f(1279.935791, 624.005737),
	   Point2f(1254.488281, 620.500488), Point2f(251.820251, 607.073792),  Point2f(331.664856, 680.368835),
	   Point2f(276.347382, 699.000671),  Point2f(951.704712, 680.760437),  Point2f(871.302368, 687.527649),
	   Point2f(1080.592407, 672.894226), Point2f(1021.454590, 675.996948), Point2f(782.107483, 696.760925),
	   Point2f(687.709900, 708.387390),  Point2f(1171.688965, 669.384216), Point2f(1130.226562, 670.634888),
	   Point2f(1236.274414, 668.066345), Point2f(1206.682495, 668.498474), Point2f(594.238892, 721.942566),
	   Point2f(506.848785, 736.840149),  Point2f(429.499207, 751.892029),  Point2f(363.449615, 766.649475),
	   Point2f(1261.391724, 667.728149), Point2f(1217.774902, 712.767395), Point2f(1266.000000, 707.000000),
	   Point2f(1187.911377, 716.711060), Point2f(1152.663208, 721.225647), Point2f(1111.451538, 726.886658),
	   Point2f(1063.052368, 733.448059), Point2f(1006.259216, 741.433105), Point2f(940.523010, 750.960693),
	   Point2f(866.235168, 761.950073),  Point2f(784.927002, 774.427917),  Point2f(232.190567, 716.368042),
	   Point2f(308.498871, 780.266418),  Point2f(263.651459, 792.569397),  Point2f(1228.140015, 748.228088),
	   Point2f(699.778564, 787.888977),  Point2f(615.030762, 801.844849),  Point2f(1200.219116, 754.520081),
	   Point2f(1169.538452, 760.980652), Point2f(1134.532715, 768.594421), Point2f(1093.911987, 777.221069),
	   Point2f(1272.075684, 766.486694), Point2f(534.841309, 815.418518),  Point2f(462.369537, 828.226440),
	   Point2f(1182.389038, 792.810059), Point2f(931.470215, 810.763062),  Point2f(863.254456, 824.333984),
	   Point2f(1152.182495, 801.514404), Point2f(1117.355225, 811.207581), Point2f(398.736115, 839.712952),
	   Point2f(344.704163, 849.900146),  Point2f(1238.837891, 802.401917), Point2f(789.525269, 838.584229),
	   Point2f(712.771179, 852.840759),  Point2f(1046.965942, 787.277527), Point2f(1077.917603, 822.101929),
	   Point2f(1135.836670, 837.736084), Point2f(636.208679, 866.637268),  Point2f(562.876831, 879.377808),
	   Point2f(1223.433838, 831.585144), Point2f(1101.624756, 849.245056), Point2f(1063.333130, 861.247498),
	   Point2f(1150.508911, 859.207520), Point2f(861.714783, 876.058167),  Point2f(794.783569, 891.032471),
	   Point2f(495.146820, 890.707275),  Point2f(434.686737, 900.329712),  Point2f(1120.795288, 870.627502),
	   Point2f(1087.229492, 882.581604), Point2f(725.549072, 905.489685),  Point2f(656.199707, 919.083740),
	   Point2f(924.075500, 861.621948),  Point2f(918.631042, 903.596741),  Point2f(860.962952, 919.078186),
	   Point2f(1106.312866, 899.736816), Point2f(589.195923, 931.208191),  Point2f(526.280884, 941.494019),
	   Point2f(800.351624, 934.021484),  Point2f(737.360840, 948.298828),  Point2f(674.452698, 961.501221),
	   Point2f(612.932861, 972.988708),  Point2f(861.001587, 954.560547),  Point2f(805.408630, 969.278381),
	   Point2f(913.910950, 939.604736),  Point2f(910.242249, 969.641602),  Point2f(861.212585, 984.421082),
	   Point2f(468.744110, 950.041382),  Point2f(554.641724, 982.818481),  Point2f(500.513550, 990.622437),
	   Point2f(748.123047, 983.233765),  Point2f(690.597168, 995.918335),  Point2f(810.165588, 998.635742),
	   Point2f(757.672058, 1012.073364), Point2f(861.701050, 1009.867554), Point2f(814.540710, 1023.257568),
	   Point2f(634.196228, 1007.002380), Point2f(704.869446, 1024.254639), Point2f(652.897339, 1034.925415),
	   Point2f(907.341309, 994.996948),  Point2f(766.234985, 1036.199463), Point2f(717.599182, 1047.768066),
	   Point2f(774.099182, 1056.969116), Point2f(729.374512, 1068.590698)};

	w = wvec;
	i = ivec;
}

void initImgPoints4(vector<Point3f> &w, vector<Point2f> &i) {
	vector<Point3f> wvec = {
	   square_size * Point3f(3, -6, 0),  square_size * Point3f(3, -5, 0),  square_size * Point3f(2, -5, 0),
	   square_size * Point3f(5, -6, 0),  square_size * Point3f(-3, -5, 0), square_size * Point3f(-2, -5, 0),
	   square_size * Point3f(-2, -4, 0), square_size * Point3f(-3, -4, 0), square_size * Point3f(-7, -7, 0),
	   square_size * Point3f(-6, -6, 0), square_size * Point3f(-7, -6, 0), square_size * Point3f(5, -5, 0),
	   square_size * Point3f(4, -5, 0),  square_size * Point3f(-5, -6, 0), square_size * Point3f(-5, -5, 0),
	   square_size * Point3f(-6, -5, 0), square_size * Point3f(4, -4, 0),  square_size * Point3f(3, -4, 0),
	   square_size * Point3f(-8, -6, 0), square_size * Point3f(-7, -5, 0), square_size * Point3f(-8, -5, 0),
	   square_size * Point3f(-1, -4, 0), square_size * Point3f(-1, -3, 0), square_size * Point3f(-2, -3, 0),
	   square_size * Point3f(-6, -4, 0), square_size * Point3f(-7, -4, 0), square_size * Point3f(2, -4, 0),
	   square_size * Point3f(3, -3, 0),  square_size * Point3f(2, -3, 0),  square_size * Point3f(-4, -4, 0),
	   square_size * Point3f(-3, -3, 0), square_size * Point3f(-4, -3, 0), square_size * Point3f(-9, -5, 0),
	   square_size * Point3f(5, -4, 0),  square_size * Point3f(5, -3, 0),  square_size * Point3f(4, -3, 0),
	   square_size * Point3f(-8, -4, 0), square_size * Point3f(-9, -4, 0), square_size * Point3f(-5, -4, 0),
	   square_size * Point3f(-5, -3, 0), square_size * Point3f(-6, -3, 0), square_size * Point3f(-7, -3, 0),
	   square_size * Point3f(-8, -3, 0), square_size * Point3f(0, -3, 0),  square_size * Point3f(0, -2, 0),
	   square_size * Point3f(-1, -2, 0), square_size * Point3f(1, -3, 0),  square_size * Point3f(2, -2, 0),
	   square_size * Point3f(1, -2, 0),  square_size * Point3f(-4, -2, 0), square_size * Point3f(-5, -2, 0),
	   square_size * Point3f(4, -2, 0),  square_size * Point3f(3, -2, 0),  square_size * Point3f(-6, -2, 0),
	   square_size * Point3f(-7, -2, 0), square_size * Point3f(-8, -2, 0), square_size * Point3f(-7, -1, 0),
	   square_size * Point3f(-8, -1, 0), square_size * Point3f(-5, -1, 0), square_size * Point3f(-6, -1, 0),
	   square_size * Point3f(-3, -2, 0), square_size * Point3f(-3, -1, 0), square_size * Point3f(-4, -1, 0),
	   square_size * Point3f(5, -2, 0),  square_size * Point3f(5, -1, 0),  square_size * Point3f(4, -1, 0),
	   square_size * Point3f(3, -1, 0),  square_size * Point3f(2, -1, 0),  square_size * Point3f(1, -1, 0),
	   square_size * Point3f(0, -1, 0),  square_size * Point3f(-9, -1, 0), square_size * Point3f(-8, 0, 0),
	   square_size * Point3f(-6, 0, 0),  square_size * Point3f(-7, 0, 0),  square_size * Point3f(6, -1, 0),
	   square_size * Point3f(6, 0, 0),   square_size * Point3f(5, 0, 0),   square_size * Point3f(-4, 0, 0),
	   square_size * Point3f(-5, 0, 0),  square_size * Point3f(4, 0, 0),   square_size * Point3f(3, 0, 0),
	   square_size * Point3f(-7, 1, 0),  square_size * Point3f(-8, 1, 0),  square_size * Point3f(2, 0, 0),
	   square_size * Point3f(1, 0, 0),   square_size * Point3f(-1, -1, 0), square_size * Point3f(0, 0, 0),
	   square_size * Point3f(-1, 0, 0),  square_size * Point3f(-5, 1, 0),  square_size * Point3f(-6, 1, 0),
	   square_size * Point3f(5, 1, 0),   square_size * Point3f(4, 1, 0),   square_size * Point3f(-6, 2, 0),
	   square_size * Point3f(-7, 2, 0),  square_size * Point3f(-3, 0, 0),  square_size * Point3f(-3, 1, 0),
	   square_size * Point3f(-4, 1, 0),  square_size * Point3f(3, 1, 0),   square_size * Point3f(2, 1, 0),
	   square_size * Point3f(-4, 2, 0),  square_size * Point3f(-5, 2, 0),  square_size * Point3f(1, 1, 0),
	   square_size * Point3f(0, 1, 0),   square_size * Point3f(-5, 3, 0),  square_size * Point3f(-6, 3, 0),
	   square_size * Point3f(2, 2, 0),   square_size * Point3f(1, 2, 0),   square_size * Point3f(-3, 2, 0),
	   square_size * Point3f(-3, 3, 0),  square_size * Point3f(-4, 3, 0),  square_size * Point3f(-1, 1, 0),
	   square_size * Point3f(0, 2, 0),   square_size * Point3f(-1, 2, 0),  square_size * Point3f(-4, 4, 0),
	   square_size * Point3f(-5, 4, 0),  square_size * Point3f(-6, 4, 0),  square_size * Point3f(-5, 5, 0)};

	vector<Point2f> ivec = {
	   Point2f(1584.825439, 313.183533),  Point2f(1630.843506, 408.474182),  Point2f(1551.484985, 378.213593),
	   Point2f(1699.528076, 377.333862),  Point2f(893.030701, 390.265686),   Point2f(1022.435852, 356.536865),
	   Point2f(1028.680054, 476.866974),  Point2f(888.007874, 504.555756),   Point2f(635.233459, 388.086578),
	   Point2f(665.627563, 419.595978),   Point2f(619.179932, 452.418518),   Point2f(1734.760986, 465.615875),
	   Point2f(1690.600952, 439.093170),  Point2f(725.332764, 382.518463),   Point2f(710.399414, 465.247864),
	   Point2f(651.230530, 497.607605),   Point2f(1722.540161, 548.096008),  Point2f(1668.325928, 525.360718),
	   Point2f(583.077759, 480.856537),   Point2f(606.287659, 525.599670),   Point2f(571.483704, 549.201965),
	   Point2f(1192.374878, 460.584412),  Point2f(1208.976440, 622.885315),  Point2f(1034.598755, 630.622803),
	   Point2f(640.865234, 587.274841),   Point2f(597.439209, 606.994019),   Point2f(1593.258423, 500.533478),
	   Point2f(1689.421387, 658.450806),  Point2f(1617.970093, 646.064209),  Point2f(779.136353, 535.346924),
	   Point2f(886.368286, 645.054932),   Point2f(774.834595, 660.304871),   Point2f(544.556458, 568.778137),
	   Point2f(1762.464966, 568.786865),  Point2f(1776.518311, 679.399597),  Point2f(1739.866699, 669.676575),
	   Point2f(564.409058, 623.449829),   Point2f(538.414490, 636.801697),   Point2f(699.372559, 563.330200),
	   Point2f(694.865173, 673.825012),   Point2f(637.073181, 685.086304),   Point2f(594.467163, 694.268799),
	   Point2f(562.253540, 701.625000),   Point2f(1378.754272, 624.468811),  Point2f(1380.795654, 805.226807),
	   Point2f(1211.993164, 804.621826),  Point2f(1517.083374, 633.514587),  Point2f(1617.792236, 800.628540),
	   Point2f(1517.724854, 803.367798),  Point2f(779.067322, 793.132568),   Point2f(698.871033, 789.357666),
	   Point2f(1738.691895, 795.259338),  Point2f(1688.553345, 798.006165),  Point2f(640.853577, 786.252808),
	   Point2f(597.942749, 783.685120),   Point2f(565.400757, 781.592285),   Point2f(607.935547, 870.550842),
	   Point2f(574.111450, 859.566528),   Point2f(711.001221, 899.727539),   Point2f(652.036743, 883.818115),
	   Point2f(890.344299, 797.530701),   Point2f(899.457886, 939.497253),   Point2f(791.034729, 918.753113),
	   Point2f(1775.378906, 793.102539),  Point2f(1758.861816, 902.642273),  Point2f(1719.865845, 915.787354),
	   Point2f(1666.356567, 930.388855),  Point2f(1593.103760, 946.193848),  Point2f(1492.280396, 960.777954),
	   Point2f(1359.411377, 970.454102),  Point2f(547.838074, 850.812927),   Point2f(587.702454, 933.491638),
	   Point2f(668.817566, 972.907043),   Point2f(623.078735, 951.267090),   Point2f(1788.688843, 891.520325),
	   Point2f(1764.922363, 984.446960),  Point2f(1730.610596, 1003.064270), Point2f(807.955688, 1026.546387),
	   Point2f(728.779480, 998.050415),   Point2f(1686.557007, 1023.558472), Point2f(1628.971436, 1046.098633),
	   Point2f(641.689453, 1023.677856),  Point2f(604.301270, 1000.201538),  Point2f(1551.893066, 1068.512451),
	   Point2f(1451.451660, 1088.296997), Point2f(1201.157349, 970.204590),  Point2f(1326.040405, 1099.972778),
	   Point2f(1182.742676, 1098.515625), Point2f(749.382629, 1080.667358),  Point2f(688.905823, 1050.380981),
	   Point2f(1693.687134, 1090.383667), Point2f(1645.805420, 1114.825928), Point2f(710.403564, 1116.130859),
	   Point2f(662.163818, 1086.490723),  Point2f(910.764160, 1056.229980),  Point2f(922.508118, 1145.848633),
	   Point2f(826.140869, 1113.252197),  Point2f(1583.277222, 1140.500122), Point2f(1504.722290, 1164.921021),
	   Point2f(843.825867, 1181.238281),  Point2f(770.203369, 1147.885864),  Point2f(1406.889282, 1184.270996),
	   Point2f(1291.128906, 1194.382324), Point2f(789.884094, 1202.218750),  Point2f(731.343079, 1170.295288),
	   Point2f(1457.932861, 1237.898071), Point2f(1365.329224, 1254.571655), Point2f(932.668457, 1212.947754),
	   Point2f(944.548706, 1265.051514),  Point2f(859.836792, 1233.920532),  Point2f(1163.878906, 1190.845703),
	   Point2f(1259.599121, 1261.603271), Point2f(1146.864990, 1256.738281), Point2f(873.593140, 1275.287964),
	   Point2f(807.820740, 1245.461304),  Point2f(751.153076, 1215.184448),  Point2f(824.194702, 1281.686768)};

	w = wvec;
	i = ivec;
}

void initImgPoints5(vector<Point3f> &w, vector<Point2f> &i) {
	vector<Point3f> wvec = {
	   square_size * Point3f(4, -4, 0),  square_size * Point3f(3, -4, 0), square_size * Point3f(6, -4, 0),
	   square_size * Point3f(5, -4, 0),  square_size * Point3f(2, -4, 0), square_size * Point3f(3, -3, 0),
	   square_size * Point3f(2, -3, 0),  square_size * Point3f(0, -4, 0), square_size * Point3f(1, -4, 0),
	   square_size * Point3f(1, -3, 0),  square_size * Point3f(0, -3, 0), square_size * Point3f(5, -3, 0),
	   square_size * Point3f(4, -3, 0),  square_size * Point3f(6, -3, 0), square_size * Point3f(6, -2, 0),
	   square_size * Point3f(5, -2, 0),  square_size * Point3f(4, -2, 0), square_size * Point3f(3, -2, 0),
	   square_size * Point3f(2, -2, 0),  square_size * Point3f(1, -2, 0), square_size * Point3f(7, -2, 0),
	   square_size * Point3f(6, -1, 0),  square_size * Point3f(5, -1, 0), square_size * Point3f(4, -1, 0),
	   square_size * Point3f(3, -1, 0),  square_size * Point3f(2, -1, 0), square_size * Point3f(0, -2, 0),
	   square_size * Point3f(1, -1, 0),  square_size * Point3f(0, -1, 0), square_size * Point3f(4, 0, 0),
	   square_size * Point3f(3, 0, 0),   square_size * Point3f(2, 0, 0),  square_size * Point3f(1, 0, 0),
	   square_size * Point3f(-1, -1, 0), square_size * Point3f(0, 0, 0),  square_size * Point3f(-1, 0, 0),
	   square_size * Point3f(1, 1, 0),   square_size * Point3f(0, 1, 0),  square_size * Point3f(2, 1, 0),
	   square_size * Point3f(2, 2, 0),   square_size * Point3f(1, 2, 0),  square_size * Point3f(-1, 1, 0),
	   square_size * Point3f(0, 2, 0),   square_size * Point3f(-1, 2, 0), square_size * Point3f(1, 3, 0),
	   square_size * Point3f(0, 3, 0)};

	vector<Point2f> ivec = {
	   Point2f(1488.808350, 646.444153),  Point2f(1438.883301, 644.551697),  Point2f(1554.339966, 649.878723),
	   Point2f(1525.592163, 648.372986),  Point2f(1371.140869, 643.660278),  Point2f(1481.532227, 751.371582),
	   Point2f(1418.052490, 766.144836),  Point2f(1146.087524, 649.968140),  Point2f(1276.179932, 644.601318),
	   Point2f(1325.704956, 785.466919),  Point2f(1191.077148, 810.415588),  Point2f(1559.711548, 732.158081),
	   Point2f(1526.418701, 740.533447),  Point2f(1585.856201, 725.481750),  Point2f(1608.806274, 805.321594),
	   Point2f(1586.173218, 821.692627),  Point2f(1555.841553, 841.730896),  Point2f(1513.698120, 867.704956),
	   Point2f(1453.345947, 900.594482),  Point2f(1363.171387, 942.773132),  Point2f(1627.073853, 792.402283),
	   Point2f(1625.601074, 886.573242),  Point2f(1603.442383, 912.088318),  Point2f(1573.595825, 943.819092),
	   Point2f(1531.780029, 983.261902),  Point2f(1471.043213, 1033.244141), Point2f(1228.718140, 992.283569),
	   Point2f(1381.612549, 1092.703735), Point2f(1250.102417, 1159.075928), Point2f(1579.634399, 1040.498047),
	   Point2f(1535.890259, 1090.014771), Point2f(1473.067261, 1149.723511), Point2f(1383.105225, 1218.116333),
	   Point2f(1072.890259, 1216.278320), Point2f(1256.827759, 1288.295654), Point2f(1094.417847, 1344.945679),
	   Point2f(1373.914185, 1314.602173), Point2f(1255.537354, 1380.359497), Point2f(1463.580811, 1246.317627),
	   Point2f(1448.000488, 1322.627441), Point2f(1361.931274, 1385.484497), Point2f(1108.835205, 1431.784790),
	   Point2f(1250.690918, 1444.782959), Point2f(1118.827026, 1490.008911), Point2f(1346.839478, 1440.204346),
	   Point2f(1244.527710, 1491.369385)};

	w = wvec;
	i = ivec;
}