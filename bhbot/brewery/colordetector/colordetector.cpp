#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>

#include <poll.h>


#define BGR_IMAGE "BGR Image"
#define HSV_IMAGE "HSV Image"


//=================================================================================================


class ColorDetector
{
public:
    enum Color {
        ColorNone,
        ColorRed,
        ColorYellow
    };
public:
    ColorDetector(int webcamId, const std::string& configFile, const std::string& imageFile, bool quiet);
    virtual ~ColorDetector();

    void process();

private:
    void reset();
    bool processLine(std::istream& stream);
    bool wait();
    void initDisplay();
    void updateDisplay();
    void scan();
    bool testComponent(float value, float reference);
    void sendPacket(const std::string packet);

private:
    bool m_quiet;
    int m_pollTimeoutMs;
    std::vector<cv::Rect> m_detectionZoneRects;
    cv::VideoCapture* m_webcam;
    cv::Mat m_bgrImage;
    float m_redFireBlueRef;
    float m_redFireGreenRef;
    float m_redFireRedRef;
    float m_yellowFireBlueRef;
    float m_yellowFireGreenRef;
    float m_yellowFireRedRef;
    Color m_lastDetectedColor;
};


ColorDetector::ColorDetector(int webcamId, const std::string& configFile, const std::string& imageFile, bool quiet) :
    m_quiet(quiet),
    m_webcam(NULL)
{
    reset();

    if (imageFile.empty()) {
        m_webcam = new cv::VideoCapture(webcamId);
        m_webcam->read(m_bgrImage);
    } else {
        m_bgrImage = cv::imread(imageFile);
    }

    std::ifstream cfg(configFile.c_str());
    if (cfg.good()) {
        std::cerr << "Loading '" << configFile << "'" << std::endl;
    }
    while (cfg.good()) {
        processLine(cfg);
    }
    cfg.close();

    initDisplay();
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

        scan();

        updateDisplay();
    }
}


void ColorDetector::reset()
{
    m_pollTimeoutMs = 100;
    m_detectionZoneRects.clear();
    m_bgrImage = cv::Mat();
    m_redFireBlueRef = -30.0;
    m_redFireGreenRef = -30.0;
    m_redFireRedRef = 220.0;
    m_yellowFireBlueRef = -30.0;
    m_yellowFireGreenRef = 220.0;
    m_yellowFireRedRef = 220.0;
    m_lastDetectedColor = ColorNone;
}


bool ColorDetector::processLine(std::istream& stream)
{
    bool again = true;

    std::string line;
    std::getline(stream, line);
    std::stringstream sstr(line);
    std::string command;

    sstr >> command;
    if (command == "ColorDetectorSetup") {
        sstr >> m_pollTimeoutMs;
        sstr >> m_redFireBlueRef >> m_redFireGreenRef >> m_redFireRedRef;
        sstr >> m_yellowFireBlueRef >> m_yellowFireGreenRef >> m_yellowFireRedRef;
    } else if (command == "ColorDetectorAddDetectionZone") {
        cv::Rect rect;
        sstr >> rect.x >> rect.y >> rect.width >> rect.height;
        m_detectionZoneRects.push_back(rect);
    } else if (command == "ColorDetectorReset") {
        reset();
    } else if (command == "ColorDetectorQuit") {
        again = false;
    } else if (!command.empty() && command[0] != '#') {
        std::cerr << "\"Unknown command '" + command + "'\"" << std::endl;
    }

    return again;
}


bool ColorDetector::wait()
{
    struct pollfd fds[1];
    fds[0].fd = 0;
    fds[0].events = POLLIN;

    int rv = poll(fds, 1, m_pollTimeoutMs);

    return rv > 0;
}


void ColorDetector::initDisplay()
{
#ifndef __arm__
    if (!m_quiet) {
        cv::namedWindow(BGR_IMAGE, cv::WINDOW_NORMAL);
        cv::resizeWindow(BGR_IMAGE, 640, 480);

        cv::namedWindow(HSV_IMAGE, cv::WINDOW_NORMAL);
        cv::resizeWindow(HSV_IMAGE, 640, 480);
        cv::moveWindow(HSV_IMAGE, 710, 0);
    }
#endif // !__arm__
}


void ColorDetector::updateDisplay()
{
#ifndef __arm__
    if (!m_quiet) {
        cv::Mat img = m_bgrImage.clone();
        for (std::vector<cv::Rect>::const_iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
            cv::rectangle(img, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 200, 0));
        }
        cv::imshow(BGR_IMAGE, img);

        cv::Mat hsvImage;
        cv::cvtColor(m_bgrImage, hsvImage, CV_BGR2HSV);
        cv::imshow(HSV_IMAGE, hsvImage);

        cv::waitKey(1);
    }
#endif // !__arm__
}


void ColorDetector::scan()
{
    float blue  = 0.0;
    float green = 0.0;
    float red   = 0.0;

    for (std::vector<cv::Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::Mat image(m_bgrImage, *it);
        std::vector<cv::Mat> components;
        cv::split(image, components);

        blue  += cv::mean(components[0])[0];
        green += cv::mean(components[1])[0];
        red   += cv::mean(components[2])[0];
    }

    blue  /= m_detectionZoneRects.size();
    green /= m_detectionZoneRects.size();
    red   /= m_detectionZoneRects.size();

    if (!m_quiet) {
        std::cerr << "B: " << blue << " G: " << green << " R: " << red << std::endl;
    }

    if (testComponent(blue,  m_redFireBlueRef)  &&
        testComponent(green, m_redFireGreenRef) &&
        testComponent(red,   m_redFireRedRef)) {
        if (m_lastDetectedColor != ColorRed) {
            m_lastDetectedColor = ColorRed;
            sendPacket("packets.ColorDetectorFire(color=TEAM_RED)");
        }
    } else if (testComponent(blue,  m_yellowFireBlueRef)  &&
        testComponent(green, m_yellowFireGreenRef)        &&
        testComponent(red,   m_yellowFireRedRef)) {
        if (m_lastDetectedColor != ColorYellow) {
            m_lastDetectedColor = ColorYellow;
            sendPacket("packets.ColorDetectorFire(color=TEAM_YELLOW)");
        }
    } else {
        m_lastDetectedColor = ColorNone;
    }
}


bool ColorDetector::testComponent(float value, float reference)
{
    if (reference < 0.0) {
        return value < -reference;
    } else {
        return value > reference;
    }
}


void ColorDetector::sendPacket(const std::string packet)
{
    std::cout << packet << std::endl;
    std::cout.flush();
}


//=================================================================================================


int main(int argc, char** argv)
{
    std::string imageFile;
    std::string configFile;
    int webcamId = 0;
    bool quiet = 0;

    configFile = argv[0];
    configFile += ".cfg";

    for (int n = 2; n < argc; n += 2) {
        const char* name = argv[n - 1];
        const char* value = argv[n];

        if (std::strcmp(name, "-i") == 0) {
            imageFile = value;
        } else if (std::strcmp(name, "-c") == 0) {
            configFile = value;
        } else if (std::strcmp(name, "-q") == 0) {
            quiet = std::atoi(value) != 0;
        } else if (std::strcmp(name, "-w") == 0) {
            webcamId = std::atoi(value);
        }
    }

    ColorDetector detector(webcamId, configFile, imageFile, quiet);
    detector.process();

    return 0;
}

