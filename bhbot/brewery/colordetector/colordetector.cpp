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
    Rect(std::istream& stream);

    std::string id;
    int x;
    int y;
    int width;
    int height;
    bool match;
};


Rect::Rect(std::istream& stream) :
    match(false)
{
    stream >> id >> x >> y >> width >> height;
}


//=================================================================================================


class ColorDetector
{
public:
    ColorDetector(const std::string& configFile, int webcamId, const std::string& imageFile);
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
    std::string bgrCheck();
    void logImage();

private:
    int m_pollTimeout;
    int m_thresholdHue;
    int m_thresholdSaturation;
    int m_thresholdTolerance;
    std::string m_logfile;
    std::vector<Rect> m_detectionZoneRects;
    std::vector<cv::Mat> m_detectionZones;
    std::vector<Rect> m_calibrationZoneRects;
    std::vector<cv::Mat> m_calibrationZones;
    cv::VideoCapture* m_webcam;
    cv::Mat m_bgrImage;
    cv::Mat m_hsvImage;
    cv::Mat m_binImage;
};


ColorDetector::ColorDetector(const std::string& configFile, int webcamId, const std::string& imageFile) :
    m_pollTimeout(100),
    m_thresholdHue(0),
    m_thresholdSaturation(0),
    m_thresholdTolerance(0),
    m_webcam(NULL)
{
    if (imageFile.empty()) {
        m_webcam = new cv::VideoCapture(webcamId);
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
    std::string output("None");

    sstr >> command;
    if (command == "quit") {
        again = false;
    } else if (command == "fetch") {
        output = bgrCheck();
        if (!m_logfile.empty()) {
            cv::imwrite(m_logfile.c_str(), m_bgrImage);
            logImage();
        }
    } else if (command == "poll_timeout") {
        sstr >> m_pollTimeout;
    } else if (command == "threshold_hue") {
        sstr >> m_thresholdHue;
    } else if (command == "threshold_saturation") {
        sstr >> m_thresholdSaturation;
    } else if (command == "threshold_tolerance") {
        sstr >> m_thresholdTolerance;
    } else if (command == "set_log_file") {
        sstr >> m_logfile;
    } else if (command == "add_calibration_zone") {
        addZone(sstr, m_calibrationZoneRects, m_calibrationZones);
    } else if (command == "add_detection_zone") {
        addZone(sstr, m_detectionZoneRects, m_detectionZones);
    } else if (!command.empty() && command[0] != '#') {
        output = "\"Unknown command '" + command + "'\"";
    }

    std::cout << output << std::endl;
    std::cout.flush();

    return again;
}


void ColorDetector::addZone(std::istream& stream, std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones)
{
    zoneRects.push_back(Rect(stream));
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
#ifndef __arm__
    cv::namedWindow(BGR_IMAGE, cv::WINDOW_NORMAL);
    cv::resizeWindow(BGR_IMAGE, 640, 480);

    cv::namedWindow(BINARIZED_IMAGE, cv::WINDOW_NORMAL);
    cv::resizeWindow(BINARIZED_IMAGE, 640, 480);
    cv::moveWindow(BINARIZED_IMAGE, 710, 0);
#endif // !__arm__
}


void ColorDetector::updateDisplay()
{
#ifndef __arm__
    cv::Mat img = m_bgrImage.clone();
    for (std::vector<Rect>::const_iterator it = m_calibrationZoneRects.begin(); it != m_calibrationZoneRects.end(); ++it) {
        cv::rectangle(img, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 0, 200));
    }
    for (std::vector<Rect>::const_iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::rectangle(img, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 200, 0));
    }
    cv::imshow(BGR_IMAGE, img);
    if (!m_binImage.empty()) {
        cv::imshow(BINARIZED_IMAGE, m_binImage);
    }
    cv::waitKey(1);
#endif // !__arm__
}


void ColorDetector::updateZones(std::vector<Rect>& zoneRects, std::vector<cv::Mat>& zones)
{
    for (size_t i = zones.size(); i < zoneRects.size(); ++i) {
        const Rect& rect = zoneRects[i];
        cv::Mat zone = m_bgrImage(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
        zones.push_back(zone);
    }
}


std::string ColorDetector::bgrCheck()
{
    std::stringstream output;
    float calibBlue = 0.0;
    float calibRed = 0.0;

    for (std::vector<cv::Mat>::iterator it = m_calibrationZones.begin(); it != m_calibrationZones.end(); ++it) {
        std::vector<cv::Mat> components;
        cv::split(*it, components);

        calibBlue += cv::mean(components[0])[0];
        calibRed += cv::mean(components[2])[0];
    }

    bool isCalibRed = calibBlue < calibRed;

    output << '{';
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

        output << '\'' << rect.id << "':";
        if (rect.match) {
            output << "True";
        } else {
            output << "False";
        }
        if (i != m_detectionZones.size() - 1) {
            output << ',';
        }
    }
    output << '}';

    return output.str();
}


void ColorDetector::logImage()
{
    std::string fname = m_logfile.substr(0, m_logfile.rfind('.')) + ".svg";
    std::string imageName = m_logfile.substr(m_logfile.rfind('/') + 1);
    std::fstream img(fname.c_str(), std::ios_base::out);

    img << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
    img << "<svg xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns=\"http://www.w3.org/2000/svg\" height=\"" << m_bgrImage.rows << "\" width=\"" << m_bgrImage.cols << "\" id=\"svg2\" version=\"1.1\">" << std::endl;
    img << "<image y=\"0.0\" x=\"0.0\" id=\"image\" xlink:href=\"" << imageName << "\" height=\"" << m_bgrImage.rows << "\" width=\"" << m_bgrImage.cols << "\" />" << std::endl;
    for (std::vector<Rect>::const_iterator it = m_calibrationZoneRects.begin(); it != m_calibrationZoneRects.end(); ++it) {
        img << "<rect style=\"fill:none;stroke:#cc0000;stroke-width:1\" width=\"" << it->width << "\" height=\"" << it->height << "\" x=\"" << it->x << "\" y=\"" << it->y << "\" />" << std::endl;
    }
    for (std::vector<Rect>::const_iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        img << "<rect style=\"fill:none;stroke:#73d216;stroke-width:1";
        if (!it->match) {
            img << ";stroke-dasharray:5,5";
        }
        img << "\" width=\"" << it->width << "\" height=\"" << it->height << "\" x=\"" << it->x << "\" y=\"" << it->y << "\" />" << std::endl;
    }
    img << "</svg>" << std::endl;

    img.close();
}


//=================================================================================================


int main(int argc, char** argv)
{
    const char* configFile = "";
    const char* imageFile = "";
    int webcamId = 0;

    for (int n = 2; n < argc; n += 2) {
        const char* name = argv[n - 1];
        const char* value = argv[n];

        if (std::strcmp(name, "-c") == 0) {
            configFile = value;
        } else if (std::strcmp(name, "-i") == 0) {
            imageFile = value;
        } else if (std::strcmp(name, "-w") == 0) {
            webcamId = std::atoi(value);
        }
    }

    ColorDetector detector(configFile, webcamId, imageFile);
    detector.process();

    return 0;
}

