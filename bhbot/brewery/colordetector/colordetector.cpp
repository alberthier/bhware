#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>

#include <poll.h>


#define BGR_IMAGE "BGR Image"
#define BINARIZED_IMAGE "Binarized Image"


//=================================================================================================


class Rect
{
public:
    Rect(int x_, int y_, int width_, int height_);

    int x;
    int y;
    int width;
    int height;
    bool match;
};


Rect::Rect(int x_, int y_, int width_, int height_) :
    x(x_),
    y(y_),
    width(width_),
    height(height_),
    match(false)
{
}


//=================================================================================================


class ColorDetector
{
public:
    ColorDetector(const std::string& configFile, const std::string& imageFile);
    virtual ~ColorDetector();

    void process();

private:
    void readStream(std::istream& stream);
    bool processLine(std::istream& stream);
    void addZone(std::istream& stream, std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones);
    bool wait();
    void initDisplay();
    void updateDisplay();
    void updateZones(std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones);
    void bgrCheck();

private:
    int m_pollTimeout;
    int m_thresholdHue;
    int m_thresholdSaturation;
    int m_thresholdTolerance;
    std::vector<Rect> m_detectionZoneRects;
    std::vector<cv::Mat> m_detectionZones;
    std::vector<Rect> m_calibrationZoneRects;
    std::vector<cv::Mat> m_calibrationZones;
    cv::VideoCapture* m_webcam;
    cv::Mat m_bgrImage;
    cv::Mat m_hsvImage;
    cv::Mat m_binImage;
};


ColorDetector::ColorDetector(const std::string& configFile, const std::string& imageFile) :
    m_pollTimeout(100),
    m_thresholdHue(0),
    m_thresholdSaturation(0),
    m_thresholdTolerance(0),
    m_webcam(NULL)
{
    if (imageFile.empty()) {
        m_webcam = new cv::VideoCapture(0);
        m_webcam->read(m_bgrImage);
    } else {
        m_bgrImage = cv::imread(imageFile);
    }

    initDisplay();

    if (!configFile.empty()) {
        std::fstream config(configFile.c_str(), std::ios_base::in);
        while (config.good()) {
            processLine(config);
        }
        config.close();
    }
}


ColorDetector::~ColorDetector()
{
    delete m_webcam;
}


void ColorDetector::process()
{
    bool again = true;
    while (std::cin.good() && again) {
        if (wait()) {
            again = processLine(std::cin);
        }

        if (m_webcam != NULL) {
            m_webcam->read(m_bgrImage);
        }

        updateDisplay();
    }
}


bool ColorDetector::processLine(std::istream& stream)
{
    bool again = true;

    std::string line;
    getline(stream, line);
    std::stringstream sstr(line);
    std::string command;

    sstr >> command;
    if (command == "quit") {
        return false;
    } else if (command == "fetch") {
        bgrCheck();
    } else if (command == "poll_timeout") {
        sstr >> m_pollTimeout;
    } else if (command == "threshold_hue") {
        sstr >> m_thresholdHue;
    } else if (command == "threshold_saturation") {
        sstr >> m_thresholdSaturation;
    } else if (command == "threshold_tolerance") {
        sstr >> m_thresholdTolerance;
    } else if (command == "add_calibration_zone") {
        addZone(sstr, m_calibrationZoneRects, m_calibrationZones);
    } else if (command == "add_detection_zone") {
        addZone(sstr, m_detectionZoneRects, m_detectionZones);
    } else if (!command.empty() && command[0] != '#') {
        std::cout << "Unknown command '" << command << "'" << std::endl;
    }

    return again;
}


void ColorDetector::addZone(std::istream& stream, std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones)
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    stream >> x >> y >> width >> height;
    zoneRects.push_back(Rect(x, y, width, height));
    updateZones(zoneRects, zones);
}


bool ColorDetector::wait()
{
    struct pollfd fds[1];
    fds[0].fd = 0;
    fds[0].events = POLLIN;

    int rv = poll(fds, 1, m_pollTimeout);

    return rv > 0;
}


void ColorDetector::initDisplay()
{
#ifdef HAVE_OPENCV_HIGHGUI
    cv::namedWindow(BGR_IMAGE, cv::WINDOW_NORMAL);
    cv::resizeWindow(BGR_IMAGE, 640, 480);

    cv::namedWindow(BINARIZED_IMAGE, cv::WINDOW_NORMAL);
    cv::resizeWindow(BINARIZED_IMAGE, 640, 480);
    cv::moveWindow(BINARIZED_IMAGE, 710, 0);
#endif // HAVE_OPENCV_HIGHGUI
}


void ColorDetector::updateDisplay()
{
#ifdef HAVE_OPENCV_HIGHGUI
    for (std::vector<Rect>::const_iterator it = m_calibrationZoneRects.begin(); it != m_calibrationZoneRects.end(); ++it) {
        cv::rectangle(m_bgrImage, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 0, 200));
    }
    for (std::vector<Rect>::const_iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::rectangle(m_bgrImage, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 200, 0));
    }
    cv::imshow(BGR_IMAGE, m_bgrImage);
    if (!m_binImage.empty()) {
        cv::imshow(BINARIZED_IMAGE, m_binImage);
    }
    cv::waitKey(1);
#endif // HAVE_OPENCV_HIGHGUI
}


void ColorDetector::updateZones(std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones)
{
    for (size_t i = zones.size(); i < zoneRects.size(); ++i) {
        const Rect& rect = zoneRects[i];
        cv::Mat zone = m_bgrImage(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
        zones.push_back(zone);
    }
}


void ColorDetector::bgrCheck()
{
    float calibBlue = 0.0;
    float calibRed = 0.0;

    for (std::vector<cv::Mat>::iterator it = m_calibrationZones.begin(); it != m_calibrationZones.end(); ++it) {
        std::vector<cv::Mat> components;
        cv::split(*it, components);

        calibBlue += cv::mean(components[0])[0];
        calibRed += cv::mean(components[2])[0];
    }

    bool isCalibRed = calibBlue < calibRed;

    for (size_t i = 0; i < m_detectionZones.size(); ++i) {
        Rect& rect = m_detectionZoneRects[i];
        cv::Mat& image = m_detectionZones[i];
        std::vector<cv::Mat> components;
        cv::split(image, components);

        float blue = cv::mean(components[0])[0];
        float red = cv::mean(components[2])[0];
        if (isCalibRed) {
            rect.match = red > blue;
        } else {
            rect.match = blue > red;
        }

        std::cout << rect.match << ',';
    }
    std::cout << std::endl;
}


//=================================================================================================


int main(int argc, char** argv)
{
    const char* configFile = "";
    const char* imageFile = "";

    for (int n = 2; n < argc; n += 2) {
        const char* name = argv[n - 1];
        const char* value = argv[n];

        if (std::strcmp(name, "-c") == 0) {
            configFile = value;
        } else if (std::strcmp(name, "-i") == 0) {
            imageFile = value;
        }
    }

    ColorDetector detector(configFile, imageFile);
    detector.process();

    return 0;
}

