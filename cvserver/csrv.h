#ifndef _CSRV_H
#define _CSRV_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

struct Camera {
	std::string	name;
	std::string	path;
	int		width;
	int		height;

	// vision target tracking
	bool		targetTrack;
	int		targetHLow, targetSLow, targetVLow;
	int		targetHHigh, targetSHigh, targetVHigh;

	// ball target tracking
	bool		ballTrack;
	int		ballHLow, ballSLow, ballVLow;
	int		ballHHigh, ballSHigh, ballVHigh;

	// saving images
	int		savePeriod;	// in ms, 0 == disable

};

struct Target {
	static std::vector<cv::Point3f> model;
	static cv::Mat cameraMatrix;

	cv::Point2f poly[4];	// order: top-left, bottom-left, bottom-right, top-right
	cv::Mat	rvec, tvec;
	double	tpos[3];
	double	yaw, pitch, roll;

	Target(cv::Point2f *box);
	~Target();

	void draw(cv::Mat &dst, int idx, int w);
	double topWidth();
	double bottomWidth();
	double centerX();
	double width();
	double height();
	void calcPnP();
};

struct Ball {
	cv::Point2f center;
	double radius;

	Ball(cv::Point2f& c, double r):center(c), radius(r) {};
};

int csrvInit(void (*frameCallback)(const Camera& c, uint64_t timestamp, cv::Mat frame),
	void (*targetsCallback)(const Camera& c, uint64_t timestamp, Target *center));

#endif
