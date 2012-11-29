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
};


Rect::Rect(int x_, int y_, int width_, int height_) :
    x(x_),
    y(y_),
    width(width_),
    height(height_)
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
    bool wait();
    void initDisplay();
    void updateDisplay();
    void updateZones();
    void binarize();

private:
    int m_pollTimeout;
    int m_thresholdHue;
    int m_thresholdSaturation;
    int m_thresholdTolerance;
    std::vector<Rect> m_zones;
    std::vector<cv::Mat> m_detectionZones;
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
        binarize();
    } else if (command == "poll_timeout") {
        sstr >> m_pollTimeout;
    } else if (command == "threshold_hue") {
        sstr >> m_thresholdHue;
    } else if (command == "threshold_saturation") {
        sstr >> m_thresholdSaturation;
    } else if (command == "threshold_tolerance") {
        sstr >> m_thresholdTolerance;
    } else if (command == "add_zone") {
        int x = 0;
        int y = 0;
        int width = 0;
        int height = 0;
        sstr >> x >> y >> width >> height;
        m_zones.push_back(Rect(x, y, width, height));
        updateZones();
    } else if (!command.empty() && command[0] != '#') {
        std::cout << "Unknown command '" << command << "'" << std::endl;
    }

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
    for (std::vector<Rect>::const_iterator it = m_zones.begin(); it != m_zones.end(); ++it) {
        cv::rectangle(m_bgrImage, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 200, 0));
    }
    cv::imshow(BGR_IMAGE, m_bgrImage);
    if (!m_binImage.empty()) {
        cv::imshow(BINARIZED_IMAGE, m_binImage);
    }
    cv::waitKey(1);
#endif // HAVE_OPENCV_HIGHGUI
}


void ColorDetector::updateZones()
{
    for (size_t i = m_detectionZones.size(); i < m_zones.size(); ++i) {
        const Rect& rect = m_zones[i];
        cv::Mat zone = m_bgrImage(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
        m_detectionZones.push_back(zone);
    }
}


void ColorDetector::binarize()
{
    cv::cvtColor(m_bgrImage, m_hsvImage, CV_BGR2HSV);

    cv::inRange(m_hsvImage, cv::Scalar(m_thresholdHue - m_thresholdTolerance - 1,
                                       m_thresholdSaturation - m_thresholdTolerance,
                                       0), 
                            cv::Scalar(m_thresholdHue + m_thresholdTolerance - 1,
                                       m_thresholdSaturation + m_thresholdTolerance,
                                       255),
                            m_binImage);

    // Create kernels for the morphological operation
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(2, 2));

    // Morphological opening (inverse because we have white pixels on black background)
    cv::dilate(m_binImage, m_binImage, kernel);
    cv::erode(m_binImage, m_binImage, kernel);
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

#if 0
static int config_hue = 0;
static int config_saturation = 0;
static int config_tolerance = 10;
static const char* config_image = NULL;


cv::Mat bgrImage;
cv::Mat hsvImage;


void binarize(const cv::Mat& input, cv::Mat& output, int h, int s, int tolerance)
{
    cv::cvtColor(input, hsvImage, CV_BGR2HSV);

    cv::inRange(hsvImage, cv::Scalar(h - tolerance - 1, s - tolerance, 0), cv::Scalar(h + tolerance - 1, s + tolerance, 255), output);

    // Create kernels for the morphological operation
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(2, 2));

    // Morphological opening (inverse because we have white pixels on black background)
    cv::dilate(output, output, kernel);
    cv::erode(output, output, kernel);
}


void processFrame(const cv::Mat& bgrImage)
{
  cv::Mat binMask;

  binarize(bgrImage, binMask, config_hue, config_saturation, config_tolerance);

  cv::imshow("BGR Image", bgrImage);
  cv::imshow("Binarization", binMask);
}


void processImage()
{
  bgrImage = cv::imread(config_image);
  std::cout << "type: " << CV_MAT_TYPE(bgrImage.flags) << std::endl;
  for (int n = 0; n < 4; ++n) {
    std::cout << "CV_8UC" << n << "=" << CV_8UC(n)   << std::endl;
    std::cout << "CV_8SC" << n << "=" << CV_8SC(n)   << std::endl;
    std::cout << "CV_16UC" << n << "=" << CV_16UC(n) << std::endl;
    std::cout << "CV_16SC" << n << "=" << CV_16SC(n) << std::endl;
    std::cout << "CV_32SC" << n << "=" << CV_32SC(n) << std::endl;
    std::cout << "CV_32FC" << n << "=" << CV_32FC(n) << std::endl;
    std::cout << "CV_64FC" << n << "=" << CV_64FC(n) << std::endl;
  }

  int key = 0;
  while (key != KEY_Q) {
    processFrame(bgrImage);

    key = cv::waitKey(100);
  }
}


void processWebcam()
{
  int key = 0;
  cv::VideoCapture capture(0);

  while (key != KEY_Q) {
    capture.read(bgrImage);

    processFrame(bgrImage);

    key = cv::waitKey(100);
  }
}


void getImageColor(int event, int x, int y, int flags, void* userdata)
{
  if (event == CV_EVENT_LBUTTONUP) {
    cv::Vec3b hsvPixel = hsvImage.at<cv::Vec3b>(y, x);
    int h = hsvPixel[0];
    int s = hsvPixel[1];
    int v = hsvPixel[2];
    config_hue = h;
    config_saturation = s;
    std::cout << "h = " << h << "    s = " << s << "    v = " << v << std::endl;
  }
}


int main(int argc, const char** argv)
{
  for (;;) {
    std::string command;
    std::cout << "> ";
    std::cout.flush();
    std::cin >> command;
    std::cout << "'" << command << "'" << std::endl;
  }
  for (int n = 2; n < argc; n += 2) {
    const char* name = argv[n - 1];
    const char* value = argv[n];

    if (std::strcmp(name, "--hue") == 0) {
      config_hue = std::atoi(value);
    } else if (std::strcmp(name, "--saturation") == 0) {
      config_saturation = std::atoi(value);
    } else if (std::strcmp(name, "--tolerance") == 0) {
      config_tolerance = std::atoi(value);
    } else if (std::strcmp(name, "--image") == 0) {
      config_image = value;
    }
  }

  // TODO init webcam

  cv::namedWindow("BGR Image");
  cv::setMouseCallback("BGR Image", getImageColor);

  if (config_image == NULL) {
    // Webcam
    processWebcam();
  } else {
    // Argument
    processImage();
  }

  return 0;
}

#endif
