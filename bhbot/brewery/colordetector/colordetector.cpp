#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>

#include <poll.h>


#define BGR_IMAGE "BGR Image"
#define HSV_IMAGE "HSV Image"


//=================================================================================================


class Rect : public cv::Rect
{
public:
    Rect(std::istream& stream);

    std::string id;
    bool match;
};


Rect::Rect(std::istream& stream) :
    cv::Rect(),
    match(false)
{
    stream >> id >> x >> y >> width >> height;
}


//=================================================================================================


class ColorDetector
{
public:
    enum Mode {
        ModeBgr,
        ModeHsv
    };

public:
    ColorDetector(const std::string& configFile, int webcamId, const std::string& imageFile);
    virtual ~ColorDetector();

    void process();

private:
    void readStream(std::istream& stream);
    bool processLine(std::istream& stream);
    bool wait();
    void initDisplay();
    void updateDisplay();
    std::string createFetchResponse();
    std::string bgrFetch();
    std::string hsvFetch();
    void logImage();
    void reset();

private:
    Mode m_mode;
    int m_pollTimeout;
    float m_hsvHueTolerance;
    float m_hsvSaturationTolerance;
    float m_hsvValueTolerance;
    std::string m_logfile;
    std::vector<Rect> m_detectionZoneRects;
    std::vector<Rect> m_calibrationZoneRects;
    cv::VideoCapture* m_webcam;
    cv::Mat m_bgrImage;
};


ColorDetector::ColorDetector(const std::string& configFile, int webcamId, const std::string& imageFile) :
    m_mode(ModeBgr),
    m_webcam(NULL)
{
    reset();

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
    std::getline(stream, line);
    std::stringstream sstr(line);
    std::string command;
    std::string output("None");

    sstr >> command;
    if (command == "set_mode") {
        std::string mode;
        sstr >> mode;
        if (mode == "BGR") {
            m_mode = ModeBgr;
        } else if (mode == "HSV") {
            m_mode = ModeHsv;
        } else {
            output = "\"Unknown mode '" + mode + "'\"";
        }
    } else if (command == "set_poll_timeout") {
        sstr >> m_pollTimeout;
    } else if (command == "set_hsv_hue_tolerance") {
        sstr >> m_hsvHueTolerance;
    } else if (command == "set_hsv_saturation_tolerance") {
        sstr >> m_hsvSaturationTolerance;
    } else if (command == "set_hsv_value_tolerance") {
        sstr >> m_hsvValueTolerance;
    } else if (command == "set_log_file") {
        sstr >> m_logfile;
    } else if (command == "add_calibration_zone") {
        m_calibrationZoneRects.push_back(Rect(sstr));
    } else if (command == "add_detection_zone") {
        m_detectionZoneRects.push_back(Rect(sstr));
    } else if (command == "fetch") {
        switch (m_mode) {
            case ModeBgr:
                output = bgrFetch();
                break;
            case ModeHsv:
                output = hsvFetch();
                break;
        }
        if (!m_logfile.empty()) {
            cv::imwrite(m_logfile.c_str(), m_bgrImage);
            logImage();
        }
    } else if (command == "reset") {
        reset();
    } else if (command == "quit") {
        again = false;
    } else if (!command.empty() && command[0] != '#') {
        output = "\"Unknown command '" + command + "'\"";
    }

    std::cout << output << std::endl;
    std::cout.flush();

    return again;
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

    cv::namedWindow(HSV_IMAGE, cv::WINDOW_NORMAL);
    cv::resizeWindow(HSV_IMAGE, 640, 480);
    cv::moveWindow(HSV_IMAGE, 710, 0);
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

    cv::Mat hsvImage;
    cv::cvtColor(m_bgrImage, hsvImage, CV_BGR2HSV);
    cv::imshow(HSV_IMAGE, hsvImage);

    cv::waitKey(1);
#endif // !__arm__
}


std::string ColorDetector::createFetchResponse()
{
    std::stringstream output;

    output << '{';
    for (size_t i = 0; i < m_detectionZoneRects.size(); ++i) {
        Rect& rect = m_detectionZoneRects[i];

        output << '\'' << rect.id << "':";
        if (rect.match) {
            output << "True";
        } else {
            output << "False";
        }
        if (i != m_detectionZoneRects.size() - 1) {
            output << ',';
        }
    }
    output << '}';

    return output.str();
}


std::string ColorDetector::bgrFetch()
{
    float calibBlue = 0.0;
    float calibRed = 0.0;

    for (std::vector<Rect>::iterator it = m_calibrationZoneRects.begin(); it != m_calibrationZoneRects.end(); ++it) {
        cv::Mat image(m_bgrImage, *it);
        std::vector<cv::Mat> components;
        cv::split(image, components);

        calibBlue += cv::mean(components[0])[0];
        calibRed  += cv::mean(components[2])[0];
    }

    calibBlue /= (float) m_calibrationZoneRects.size();
    calibRed  /= (float) m_calibrationZoneRects.size();

    bool isCalibRed = calibBlue < calibRed;

    for (std::vector<Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::Mat image(m_bgrImage, *it);
        std::vector<cv::Mat> components;
        cv::split(image, components);

        float blue = cv::mean(components[0])[0];
        float red  = cv::mean(components[2])[0];
        if (isCalibRed) {
            it->match = red > blue;
        } else {
            it->match = blue > red;
        }
    }

    return createFetchResponse();
}


std::string ColorDetector::hsvFetch()
{
    float calibHue = 0.0;
    float calibSaturation = 0.0;
    float calibValue = 0.0;

    for (std::vector<Rect>::iterator it = m_calibrationZoneRects.begin(); it != m_calibrationZoneRects.end(); ++it) {
        cv::Mat bgrZone(m_bgrImage, *it);
        cv::Mat hsvZone;
        cv::cvtColor(bgrZone, hsvZone, CV_BGR2HSV);
        std::vector<cv::Mat> components;
        cv::split(hsvZone, components);

        calibHue        += cv::mean(components[0])[0];
        calibSaturation += cv::mean(components[1])[0];
        calibValue      += cv::mean(components[2])[0];
    }

    calibHue        /= (float) m_calibrationZoneRects.size();
    calibSaturation /= (float) m_calibrationZoneRects.size();
    calibValue      /= (float) m_calibrationZoneRects.size();

    float minHue        = std::max(calibHue        - m_hsvHueTolerance,          0.0f);
    float minSaturation = std::max(calibSaturation - m_hsvSaturationTolerance,   0.0f);
    float minValue      = std::max(calibValue      - m_hsvValueTolerance,        0.0f);
    float maxHue        = std::min(calibHue        + m_hsvHueTolerance,        255.0f);
    float maxSaturation = std::min(calibSaturation + m_hsvSaturationTolerance, 255.0f);
    float maxValue      = std::min(calibValue      + m_hsvValueTolerance,      255.0f);

    for (std::vector<Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::Mat bgrZone(m_bgrImage, *it);
        cv::Mat hsvZone;
        cv::cvtColor(bgrZone, hsvZone, CV_BGR2HSV);
        cv::Mat inRange;

        cv::Scalar avg = cv::mean(hsvZone);

        it->match = minHue        <= avg[0] && avg[0] <= maxHue &&
                    minSaturation <= avg[1] && avg[1] <= maxSaturation &&
                    minValue      <= avg[2] && avg[2] <= maxValue;
    }

    return createFetchResponse();
}


void ColorDetector::logImage()
{
    std::string fname = m_logfile.substr(0, m_logfile.rfind('.')) + ".svg";
    std::string imageName = m_logfile.substr(m_logfile.rfind('/') + 1);
    std::fstream img(fname.c_str(), std::ios_base::out);

    img << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
    img << "<svg xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns=\"http://www.w3.org/2000/svg\" height=\"" << m_bgrImage.rows << "\" width=\"" << m_bgrImage.cols << "\" id=\"svg2\" version=\"1.1\">" << std::endl;
    img << "<g transform=\"rotate(180," << m_bgrImage.cols / 2 << ',' << m_bgrImage.rows / 2 << ")\">" << std::endl;
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
    img << "</g>" << std::endl;
    img << "</svg>" << std::endl;

    img.close();
}


void ColorDetector::reset()
{
    m_pollTimeout = 100;
    m_hsvHueTolerance = 255.0;
    m_hsvSaturationTolerance = 255.0;
    m_hsvValueTolerance = 255.0;
    m_logfile.clear();
    m_detectionZoneRects.clear();
    m_calibrationZoneRects.clear();
    m_bgrImage = cv::Mat();
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

