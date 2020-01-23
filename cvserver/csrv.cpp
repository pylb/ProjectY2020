/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <mutex>
#include <condition_variable>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <networktables/NetworkTableInstance.h>

#include "cameraserver/CameraServer.h"
#include "csrv.h"

// Scale factor. Should be 1 for the real robot
#define SCALE 1

// Set to 0 for not printing anything on the screen
#define DEBUG 0

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ]
           }
       ]
   }
 */

using namespace cv;
using namespace std;
//using namespace cv::xfeatures2d;

struct CameraImpl : Camera {
	wpi::json	config;
//	CS_Source	outHandle;	// output stream
	CS_Source	targetHandle;	// target output stream
	CS_Source	ballHandle;	// ball output stream

	// for publishing target data
	std::shared_ptr<nt::NetworkTable> ntbl;

	// callbacks for when new frame and/or targets are read
	void (*frameCallback)(const Camera& c, uint64_t timestamp, Mat frame);
	void (*targetsCallback)(const Camera& c, uint64_t timestamp, Target *t);

	// for saving the image in a separate thread
	std::mutex imgmtx;
	std::condition_variable imgcond;
	Mat* img;
	char imgname[256];
};

static const char* configFile = "/boot/frc.json";
unsigned int team;
bool server = false;

vector<CameraImpl*> cameras;
nt::NetworkTableInstance ntinst;

bool saveImages;		// true if any of the cameras is going to save images
char imagePath[256];

// Target static members and methods
// Model of the target. All numbers are in meters.
// The center of the coordinate system is the floor level of the 
// center of the target.
vector<Point3f> Target::model = {
	Point3f(-0.498285/SCALE, 2.49555/SCALE, 0),	// top left corner
	Point3f(-0.2492375/SCALE, 2.06375/SCALE, 0),	// bottom left corner
	Point3f(0.2492375/SCALE, 2.06375/SCALE, 0),	// bottom right corner
	Point3f(0.498285/SCALE, 2.49555/SCALE, 0),
};

// Raspberry Pi Camera (640x480)
// old Mat Target::cameraMatrix = (Mat_<double>(3,3) <<  4.6676194913596777e+02, 0., 3.1086529034645207e+02, 0., 4.6676194913596777e+02, 2.3946292934201807e+02, 0., 0., 1.);
// Mat Target::cameraMatrix = (Mat_<double>(3,3) << 4.9889252599572541e+02, 0., 3.1157422532497679e+02, 0., 4.9889252599572541e+02, 2.3724852037903509e+02, 0., 0., 1.);

// Microsoft HD Camera (640x480)
// old Mat Target::cameraMatrix = (Mat_<double>(3,3) <<  6.7939614509180524e+02, 0., 3.0627626279973128e+02, 0., 6.7939614509180524e+02, 2.2394196729643429e+02, 0., 0., 1.);
 Mat Target::cameraMatrix = (Mat_<double>(3,3) <<  6.7076054565726974e+02, 0., 3.3601187921143253e+02, 0., 6.7076054565726974e+02, 2.3234578140238639e+02, 0., 0., 1.);

// Intel RealSense (640x480)
//Mat Target::cameraMatrix = (Mat_<double>(3,3) <<  5.9499960389797957e+02, 0., 3.2018801228431829e+02, 0., 5.9499960389797957e+02, 2.4618253134582881e+02, 0., 0., 1.);

// Intel RealSense (960x540)
//Mat Target::cameraMatrix = (Mat_<double>(3,3) << 6.9584472882317743e+02, 0., 4.8789603110723795e+02, 0., 6.9584472882317743e+02, 2.6041500439328513e+02, 0., 0., 1.);

Target::Target(Point2f* box) {
	poly[0] = box[0];
	poly[1] = box[1];
	poly[2] = box[2];
	poly[3] = box[3];
	calcPnP();
}

Target::~Target() {
}

double Target::topWidth() {
	return poly[3].x - poly[0].x;
}

double Target::bottomWidth() {
	return poly[2].x - poly[1].x;
}

double Target::centerX() {
	return (poly[0].x + poly[1].y + poly[2].x + poly[3].x) / 4.0;
}

double Target::width() {
	return (topWidth() + bottomWidth()) / 2.0;
}

double Target::height() {
	double left, right;

	left = poly[1].y - poly[0].y;
	right = poly[2].y - poly[3].y;

	return (left + right) / 2.0;
}

void Target::calcPnP() {
	int i;
	vector<Point2f> pts;

	for(i = 0; i < 4; i++)
		pts.push_back(poly[i]);

	Mat d;
	bool ret = solvePnP(model, pts, cameraMatrix, d, rvec, tvec);

	double *t = (double *) tvec.datastart;
	tpos[0] = t[0];
	tpos[1] = t[1];
	tpos[2] = t[2];

	Mat rvec1;
	Rodrigues(rvec, rvec1);

	Vec3d eulerAngles;
	Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
	double* r = rvec1.ptr<double>();
	double projMatrix[12] = {
		r[0], r[1], r[2], 0,
                r[3], r[4], r[5], 0,
                r[6], r[7], r[8], 0
	};

	decomposeProjectionMatrix( Mat(3,4,CV_64FC1, projMatrix), cameraMatrix,
		rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);

	yaw   = eulerAngles[1]; 
	pitch = eulerAngles[0];
	roll  = eulerAngles[2];
}

void Target::draw(Mat &dst, int idx, int w) {
	Scalar color[] = { Scalar(0, 0, 255), Scalar(255, 0, 0), Scalar(255, 255, 255) };

	line(dst, poly[0], poly[1], color[idx], w);
	line(dst, poly[1], poly[2], color[idx], w);
	line(dst, poly[2], poly[3], color[idx], w);
	line(dst, poly[3], poly[0], color[idx], w);

//	circle(dst, poly[0], 5, Scalar(255, 0, 0));
//	circle(dst, poly[1], 5, Scalar(0, 0, 255));
//	circle(dst, poly[2], 5, Scalar(255, 0, 0), 3);
//	circle(dst, poly[3], 5, Scalar(0, 0, 255), 3);
}

double distance(Point2f& p1, Point2f& p2) {
	double dx, dy;

	dx = p1.x - p2.x;
	dy = p1.y - p2.y;

	return sqrt(dx*dx + dy*dy);
}

int cmppoint(const void *p1, const void *p2) {
	int ret;

	Point *pt1 = (Point *) p1;
	Point *pt2 = (Point *) p2;

	ret = pt1->x - pt2->x;
	if (ret == 0)
		ret = pt1->y - pt2->y;

	return ret;
}

double dist2(Point2f& p1, Point2f& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;

	return dx*dx + dy*dy;
}

Target *processContour(OutputArrayOfArrays contour, Mat &dst) {
	double p;
	Point2f box[5], poly[4];
	vector<Point> approx, convex;

	p = arcLength(contour, true);

	// we don't care about contours that are too small
//	if (p < 40)
//		return NULL;

	convexHull(contour, convex);
	approxPolyDP(convex, approx, p * 0.05, true);
#if 0
	printf("poly: %d length %f\n", approx.size(), p);
#endif

	if (approx.size() != 4)
		return NULL;

	drawContours(dst, vector<vector<Point> >(1,approx), 0, Scalar(0,0,255), 1);
//	line(dst, approx[1], approx[2], Scalar(255, 0, 0), 5);

/*
	if (approx.size() != 4) {
		if (approx.size() > 4) {
			Scalar clr(255, 255, 0);

			for(int i = 0; i < approx.size(); i++) {
				int j = i + 1;
				if (j >= approx.size())
					j = 0;

				line(dst, approx[i], approx[j], clr);
			}
		}

//		printf("approx.size %d\n", approx.size());
		return NULL;
	}
*/

	for(int i = 0; i < approx.size(); i++) {
		Point pt = approx[i];

		box[i].x = pt.x;
		box[i].y = pt.y;
	}
	box[4] = box[0];

	qsort(box, 4, sizeof(box[0]), cmppoint);
	poly[0] = box[0];
	poly[1] = box[1];
	poly[2] = box[2];
	poly[3] = box[3];

	return new Target(poly);
}

bool checkTarget(Target *t) {
	double area;

	area = t->height() * t->width();

	if (area < 50) {
#if DEBUG
		printf("target too small: %f\n", area);
#endif
		return false;
	}

	double r = t->topWidth() / t->bottomWidth();

	if (r < 1.5 || r > 2.4) {
#if DEBUG
		printf("ratio out of limits: %f\n", r);
#endif
		return false;
	}

	return true;
}

void processTargets(Mat &src, Mat &dst, Mat &mask, const CameraImpl& c, uint64_t tstamp) {
	Mat hsv, gauss, emask, dmask;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	Target *tgt;

	// Convert to HSV
	cvtColor(src /*gauss*/, hsv, CV_BGR2HSV);

	// Get only pixels that have the colors we expect the stripes to be
	inRange(hsv, Scalar(c.targetHLow, c.targetSLow, c.targetVLow), Scalar(c.targetHHigh, c.targetSHigh, c.targetVHigh), mask);

	// Find contours
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS/*CHAIN_APPROX_SIMPLE*/);
#if 0
	printf("contours: %d\n", contours.size());
#endif

	// Identify targets
	tgt = NULL;
	for(size_t i = 0; i < contours.size(); i++) {
//		if (hierarchy[i][3] >= 0)
//			continue;

		Target *t = processContour(contours[i], dst);
		if (t == NULL) {
//			printf("\tbad\n");
			continue;
		}

		// check if the target follows some common sense restrictions
		if (!checkTarget(t)) {
			delete(t);
			continue;
		}

		if (tgt == NULL) {
			tgt = t;
		} else {
			// TODO: handle multiple targets: pick the bigger?
		}
	}

	if (c.targetsCallback)
		c.targetsCallback(c, tstamp, tgt);

	if (tgt) {
		tgt->draw(mask, 2, 2);

		double cx = ((tgt->centerX() * 2) / c.width) - 1.0;
		double w = tgt->width() / c.width;

		double yawrad = (tgt->yaw*M_PI)/180.0;
		double z = tgt->tpos[2] * cos(yawrad);
		double x = tgt->tpos[2] * sin(yawrad) - tgt->tpos[0];

		c.ntbl->PutNumber("X", x);
		c.ntbl->PutNumber("Z", z);
		c.ntbl->PutNumber("Yaw", tgt->yaw);
//		c.ntbl->PutNumber("Pitch", center->pitch);
//		c.ntbl->PutNumber("Roll", center->roll);

		c.ntbl->PutNumber("Center", cx);
		c.ntbl->PutNumber("Width", w);
#if DEBUG
		printf("yaw %f pitch %f roll %f\n", tgt->yaw, tgt->pitch, tgt->roll);
		printf("x %f y %f z %f\n", tgt->tpos[0], tgt->tpos[1], tgt->tpos[2]);
		printf("center: %f width %f\n", cx, w);
#endif

	} else {
		c.ntbl->PutNumber("Center", 0.0);
		c.ntbl->PutNumber("Width", 0.0);
	}

	delete(tgt);
}

void processBalls(Mat &src, Mat &dst, Mat &mask, const CameraImpl& c, uint64_t tstamp) {
	Mat hsv, gauss, emask, dmask;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	vector<Ball *> balls;

	// Convert to HSV
	cvtColor(src /*gauss*/, hsv, CV_BGR2HSV);

	// Get only pixels that have the colors we expect the stripes to be
	inRange(hsv, Scalar(c.ballHLow, c.ballSLow, c.ballVLow), Scalar(c.ballHHigh, c.ballSHigh, c.ballVHigh), mask);

	// Find contours
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS/*CHAIN_APPROX_SIMPLE*/);
#if DEBUG
	printf("contours: %d\n", contours.size());
#endif

	// Identify balls
	for(size_t i = 0; i < contours.size(); i++) {
		Point2f c;
		float r;
		double ratio;

		if (hierarchy[i][3] >= 0)
			continue;

		minEnclosingCircle(contours[i], c, r);
		ratio = contourArea(contours[i]) / (r*r*M_PI);

//		if (r < 3 || r > 1000 || ratio < 0.1)
//			continue;

//		printf("%d: r %f ratio %f\n", i, r, ratio);

		// make sure that the ball is big enough
		if (r < 20) {
			continue;
		}

		// make sure that the contour is ball-like
		if (ratio < 0.7) {
			continue;
		}

		Ball *b = new Ball(c, r);
		balls.push_back(b);
		circle(dst, c, r, Scalar(0, 0, 255));
	}

	
	printf("total balls: %d\n", balls.size());

#if 0
	if (c.targetsCallback)
		c.targetsCallback(c, tstamp, tgt);

	if (tgt) {
		tgt->draw(dst, 0, 2);

		double cx = ((tgt->centerX() * 2) / c.width) - 1.0;
		double w = tgt->width() / c.width;

		double yawrad = (tgt->yaw*M_PI)/180.0;
		double z = tgt->tpos[2] * cos(yawrad);
		double x = tgt->tpos[2] * sin(yawrad) - tgt->tpos[0];

		c.ntbl->PutNumber("X", x);
		c.ntbl->PutNumber("Z", z);
		c.ntbl->PutNumber("Yaw", tgt->yaw);
//		c.ntbl->PutNumber("Pitch", center->pitch);
//		c.ntbl->PutNumber("Roll", center->roll);

		c.ntbl->PutNumber("Center", cx);
		c.ntbl->PutNumber("Width", w);
#if DEBUG
		printf("yaw %f pitch %f roll %f\n", center->yaw, center->pitch, center->roll);
		printf("x %f y %f z %f\n", center->tpos[0], center->tpos[1], center->tpos[2]);
		printf("center: %f width %f\n", cx, w);
#endif

	} else {
		c.ntbl->PutNumber("Center", 0.0);
		c.ntbl->PutNumber("Width", 0.0);
	}

	delete(tgt);
#endif

}

wpi::raw_ostream& ParseError() {
	return wpi::errs() << "config error in '" << configFile << "': ";
}

bool readCameraConfig(const wpi::json& config) {
	CameraImpl *cp = new CameraImpl();
	CameraImpl& c = *cp;

	c.config = config;
	c.width = 640;
	c.height = 480;
	c.targetTrack = false;
	c.targetHLow = 0;
	c.targetSLow = 0;
	c.targetVLow = 0;
	c.targetHHigh = 180;
	c.targetSHigh = 255;
	c.targetVHigh = 255;
	c.savePeriod = 0;
	c.ntbl = NULL;
	c.frameCallback = NULL;
	c.targetsCallback = NULL;

	// name
	try {
		c.name = config.at("name").get<string>();
	} catch (const wpi::json::exception& e) {
		ParseError() << "could not read camera name: " << e.what() << '\n';
		return false;
	}

	// path
	try {
		c.path = config.at("path").get<string>();
	} catch (const wpi::json::exception& e) {
		ParseError() << "camera '" << c.name << "': could not read path: " << e.what() << '\n';
		return false;
	}

	// image width and height
	try {
		c.width = config.at("width").get<int>();
		c.height = config.at("height").get<int>();
	} catch (const wpi::json::exception& e) {
		// ignore, we have default values
	}

	try {
		for (auto&& prop : config.at("properties")) {
			std::string name;

			name = prop.at("name").get<std::string>();
			if (name == "track_target") {
				c.targetTrack  = prop.at("value").get<bool>();
			} else if (name == "track_ball") {
				c.ballTrack  = prop.at("value").get<bool>();
			} else if (name == "target_h_low") {
				c.targetHLow = prop.at("value").get<int>();
			} else if (name == "target_s_low") {
				c.targetSLow = prop.at("value").get<int>();
			} else if (name == "target_v_low") {
				c.targetVLow = prop.at("value").get<int>();
			} else if (name == "target_h_high") {
				c.targetHHigh = prop.at("value").get<int>();
			} else if (name == "target_s_high") {
				c.targetSHigh = prop.at("value").get<int>();
			} else if (name == "target_v_high") {
				c.targetVHigh = prop.at("value").get<int>();
			} else if (name == "ball_h_low") {
				c.ballHLow = prop.at("value").get<int>();
			} else if (name == "ball_s_low") {
				c.ballSLow = prop.at("value").get<int>();
			} else if (name == "ball_v_low") {
				c.ballVLow = prop.at("value").get<int>();
			} else if (name == "ball_h_high") {
				c.ballHHigh = prop.at("value").get<int>();
			} else if (name == "ball_s_high") {
				c.ballSHigh = prop.at("value").get<int>();
			} else if (name == "ball_v_high") {
				c.ballVHigh = prop.at("value").get<int>();
			} else if (name == "save_period") {
				c.savePeriod = prop.at("value").get<int>();
				if (c.savePeriod != 0)
					saveImages = true;
			}
		}
	} catch (const wpi::json::exception& e) {
		ParseError() << "could not read property name: " << e.what();
		// ignore
	}

	cameras.emplace_back(cp);
	return true;
}

bool readConfig() {
	// open config file
	error_code ec;
	wpi::raw_fd_istream is(configFile, ec);
	if (ec) {
		wpi::errs() << "could not open '" << configFile << "': " << ec.message() << '\n';
		return false;
	}

	// parse file
	wpi::json j;
	try {
		j = wpi::json::parse(is);
	} catch (const wpi::json::parse_error& e) {
		ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
		return false;
	}

	// top level must be an object
	if (!j.is_object()) {
		ParseError() << "must be JSON object\n";
		return false;
	}

	// team number
	try {
		team = j.at("team").get<unsigned int>();
	} catch (const wpi::json::exception& e) {
		ParseError() << "could not read team number: " << e.what() << '\n';
		return false;
	}

	// ntmode (optional)
	if (j.count("ntmode") != 0) {
		try {
			auto str = j.at("ntmode").get<string>();
			wpi::StringRef s(str);
			if (s.equals_lower("client")) {
				server = false;
			} else if (s.equals_lower("server")) {
				server = true;
			} else {
				ParseError() << "could not understand ntmode value '" << str << "'\n";
			}
		} catch (const wpi::json::exception& e) {
			ParseError() << "could not read ntmode: " << e.what() << '\n';
		}
	}

	// cameras
	try {
		for (auto&& camera : j.at("cameras")) {
			if (!readCameraConfig(camera)) return false;
		}
	} catch (const wpi::json::exception& e) {
		ParseError() << "could not read cameras: " << e.what() << '\n';
		return false;
	}

	return true;
}

int initImageDirectory(const char *basePath) {
	for(int i = 0; i < 1000; i++) {
		snprintf(imagePath, sizeof(imagePath), "%s/%03d", basePath, i);
		if (mkdir(imagePath, 0777) == 0) {
			return 0;
		}

		if (errno != EEXIST) {
			imagePath[0] = '\0';
			return -1;
		}
	}

	imagePath[0] = '\0';
	return -1;
}

bool saveImage(CameraImpl *c, const char *cameraName, uint64_t timestamp, Mat& img) {
	char fname[256];

	if (imagePath[0] == '\0')
		return true;

	std::unique_lock<std::mutex> lck (c->imgmtx);
	snprintf(c->imgname, sizeof(c->imgname), "%s/%s-%011lld.jpg", imagePath, cameraName, timestamp);
	c->img = &img;
	c->imgcond.notify_all();

	return true;
}

void cameraThread(CameraImpl *c) {
	CS_Status status = 0;
	char buf[20];
	cv::Mat frames[2];
	string tgtname, ballname;
	cs::CvSource targetStream, ballStream;

	wpi::outs() << "Starting camera '" << c->name << "' on " << c->path << '\n';
	auto camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(c->name, c->path);
	camera.SetConfigJson(c->config);

	if (!c->targetTrack && !c->ballTrack && c->savePeriod == 0)
		return;

	if (c->targetTrack) {
		printf(">>> track target %d\n", c->targetTrack);
		tgtname = c->name + "Target";
		c->ntbl = ntinst.GetTable("SmartDashboard/" + tgtname);
		targetStream = frc::CameraServer::GetInstance()->PutVideo(tgtname, c->width, c->height);
		targetStream.CreateProperty("track_target", cs::VideoProperty::Kind::kBoolean, 0, 1, 1, 0, c->targetTrack);
		targetStream.CreateProperty("target_h_low", cs::VideoProperty::Kind::kInteger, 0, 180, 1, 25, c->targetHLow);
		targetStream.CreateProperty("target_h_high", cs::VideoProperty::Kind::kInteger, 0, 180, 1, 35, c->targetHHigh);
		targetStream.CreateProperty("target_s_low", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128,c->targetSLow);
		targetStream.CreateProperty("target_s_high", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128,c->targetSHigh);
		targetStream.CreateProperty("target_v_low", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128, c->targetVLow);
		targetStream.CreateProperty("target_v_high", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128, c->targetVHigh);
		c->targetHandle = targetStream.GetHandle();
	}

	if (c->ballTrack) {
		printf(">>> track ball %d\n", c->ballTrack);
		ballname = c->name + "Ball";
		c->ntbl = ntinst.GetTable("SmartDashboard/" + ballname);
		ballStream = frc::CameraServer::GetInstance()->PutVideo(tgtname, c->width, c->height);
		ballStream.CreateProperty("track_ball", cs::VideoProperty::Kind::kBoolean, 0, 1, 1, 0, c->ballTrack);
		ballStream.CreateProperty("ball_h_low", cs::VideoProperty::Kind::kInteger, 0, 180, 1, 25, c->ballHLow);
		ballStream.CreateProperty("ball_h_high", cs::VideoProperty::Kind::kInteger, 0, 180, 1, 35, c->ballHHigh);
		ballStream.CreateProperty("ball_s_low", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128,c->ballSLow);
		ballStream.CreateProperty("ball_s_high", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128,c->ballSHigh);
		ballStream.CreateProperty("ball_v_low", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128, c->ballVLow);
		ballStream.CreateProperty("ball_v_high", cs::VideoProperty::Kind::kInteger, 0, 255, 1, 128, c->ballVHigh);
		c->ballHandle = ballStream.GetHandle();
	}

	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo(camera);
	uint64_t prevtime = 0;
	uint64_t savebase, prevsave = 0;
	int framecount = 0;
	const char *cname = c->name.c_str();
	const char *tcname = tgtname.c_str();
	int fnum = 0;
	while (true) {
		Mat& frame = frames[fnum];
		fnum = (fnum+1) % 2;

		// Tell the CvSink to grab a frame from the camera and put it
		// in the source mat.  If there is an error notify the output.
		uint64_t tstamp = cvSink.GrabFrame(frame);
		if (tstamp == 0) {
			// Send the output the error.
			targetStream.NotifyError(cvSink.GetError());
			// skip the rest of the current iteration
			continue;
		}

		// call user's code, if specified
		if (c->frameCallback)
			c->frameCallback(*c, tstamp, frame);

		// save the image, if configured to
		if (c->savePeriod != 0) {
			if (prevsave == 0)
				savebase = tstamp;

			bool bsave = ((tstamp - prevsave)/1000.0) > c->savePeriod;
			if (bsave) {
				saveImage(c, cname, tstamp - savebase, frame);
				prevsave = tstamp;
			}
		}

		if (c->targetTrack) {
			Mat mask;

			processTargets(frame, frame, mask, *c, tstamp);
			targetStream.PutFrame(mask/*frame*/);
		}

		if (c->ballTrack) {
			Mat mask;

			processBalls(frame, frame, mask, *c, tstamp);
//			ballStream.PutFrame(mask);
			ballStream.PutFrame(frame);
		}

		// calculate the fps
		if ((tstamp - prevtime) > 1000000) {
//#if DEBUG
			if (prevtime != 0)
				fprintf(stdout, "Rate: %f fps\n", framecount*1000000.0 / (tstamp - prevtime));
//#endif
			framecount = 0;
			prevtime = tstamp;
		}

		framecount++;
	}
}

void imgThread(CameraImpl *c) {
	char imgname[256];
	Mat *img;

	std::unique_lock<std::mutex> lck (c->imgmtx);

	while (1) {
		if (c->img == NULL) {
			// no image, wait for notification
			c->imgcond.wait(lck);
			continue;
		}

		strncpy(imgname, c->imgname, sizeof(imgname));
		img = c->img;
		c->img = NULL;
		lck.unlock();
		c->imgcond.notify_all();

		imwrite(imgname, *img);
		sync();
		lck.lock();
	}
}


int csrvInit(void (*frameCallback)(const Camera& c, uint64_t timestamp, cv::Mat frame),
		void (*targetsCallback)(const Camera& c, uint64_t timestamp, Target *t)) {

	CS_Status status;

	// read configuration
	if (!readConfig())
		return EXIT_FAILURE;

	// start NetworkTables
	ntinst = nt::NetworkTableInstance::GetDefault();
	if (server) {
		wpi::outs() << "Setting up NetworkTables server\n";
		ntinst.StartServer();
	} else {
		wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
		ntinst.StartClientTeam(team);
	}

	cs::AddListener(
		[&](const cs::RawEvent& event) {
			CameraImpl *cam;

			// find which camera
			cam = NULL;
			for(int i = 0; i < cameras.size(); i++) {
				
				auto c = cameras[i];
				if (c->targetHandle == event.sourceHandle || c->ballHandle == event.sourceHandle) {
					cam = c;
					break;
				}
			}


			if (cam == NULL) {
				printf("Property for unknown camera updated: %s %d\n", event.name.c_str(), event.value);
				return;
			}
//#if DEBUG
//			printf("Property for %s updated %s: %d/%s\n", config->name.c_str(), event.name.c_str(), event.value, event.valueStr.c_str());
//#endif
			if (event.name == "target_h_low") {
				cam->targetHLow = event.value;
			} else if (event.name == "target_s_low") {
				cam->targetSLow = event.value;
			} else if (event.name == "target_v_low") {
				cam->targetVLow = event.value;
			} else if (event.name == "target_h_high") {
				cam->targetHHigh = event.value;
			} else if (event.name == "target_s_high") {
				cam->targetSHigh = event.value;
			} else if (event.name == "target_v_high") {
				cam->targetVHigh = event.value;
			} else if (event.name == "track_target") {
				cam->targetTrack = event.value;
			} else if (event.name == "ball_h_low") {
				cam->ballHLow = event.value;
			} else if (event.name == "ball_s_low") {
				cam->ballSLow = event.value;
			} else if (event.name == "ball_v_low") {
				cam->ballVLow = event.value;
			} else if (event.name == "ball_h_high") {
				cam->ballHHigh = event.value;
			} else if (event.name == "ball_s_high") {
				cam->ballSHigh = event.value;
			} else if (event.name == "ball_v_high") {
				cam->ballVHigh = event.value;
			} else if (event.name == "track_ball") {
				cam->ballTrack = event.value;
			}
		}, cs::RawEvent::kSourcePropertyValueUpdated, true, &status);

	if (saveImages && initImageDirectory("/mnt/rw") != 0)
		fprintf(stderr, "Can't initialize image saving\n");

	// start cameras
	for(int i = 0; i < cameras.size(); i++) {
		auto camera = cameras[i];

		camera->frameCallback = frameCallback;
		camera->targetsCallback = targetsCallback;

		thread cthread(cameraThread, camera);
		cthread.detach();

		thread ithread(imgThread, camera);
		ithread.detach();
	}

	return 0;
}
