#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>
#include <map>

#include <poll.h>

#include <sys/time.h>

#define BGR_IMAGE "BGR Image"
#define HSV_IMAGE "HSV Image"


//=================================================================================================

const char* getPixelColorType(int H, int S, int V);
const char* getPixelColorTypeBH(int H, int S, int V);

class ColorDetector
{
public:
    enum Color {
        ColorNone,
        ColorRed,
        ColorYellow
    };

    enum ScanMethod {
        ScanHsv,
        ScanRgb
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
    void scanRgb();
    void scanHsv();
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
    int m_camWidth;
    int m_camHeight;

    ScanMethod m_scanMethod;
    char* m_scanMethodName;
};


ColorDetector::ColorDetector(int webcamId, const std::string& configFile, const std::string& imageFile, bool quiet) :
    m_quiet(quiet),
    m_webcam(NULL)
{
    reset();

    std::ifstream cfg(configFile.c_str());

    if (cfg.good()) {
        std::cerr << "Loading '" << configFile << "'" << std::endl;
    }

    while (cfg.good()) {
        processLine(cfg);
    }
    cfg.close();

    if (imageFile.empty()) {
        m_webcam = new cv::VideoCapture(webcamId);
        m_webcam->set(CV_CAP_PROP_FRAME_WIDTH,m_camWidth);
        m_webcam->set(CV_CAP_PROP_FRAME_HEIGHT,m_camHeight);

        std::cerr << "Using webcam " << webcamId << " resolution " << m_camWidth << "x" << m_camHeight << std::endl;

        m_webcam->read(m_bgrImage);
    } else {
        m_bgrImage = cv::imread(imageFile);
    }

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
    m_redFireBlueRef = -30.0f;
    m_redFireGreenRef = -30.0f;
    m_redFireRedRef = 220.0f;
    m_yellowFireBlueRef = -30.0f;
    m_yellowFireGreenRef = 220.0f;
    m_yellowFireRedRef = 220.0f;
    m_lastDetectedColor = ColorNone;
    m_scanMethod = ScanRgb;
    m_scanMethodName = "RGB";
}


bool ColorDetector::processLine(std::istream& stream)
{
    bool again = true;

    std::string line;
    std::getline(stream, line);
    std::stringstream sstr(line);
    std::string command;

    sstr >> command;

    if (command == "ScanMethod") {
        std::string mode;
        sstr >> mode;

        if (mode == "HSV") {
            m_scanMethod = ScanHsv;
            m_scanMethodName = "HSV";
        } else {
            m_scanMethod = ScanRgb;
            m_scanMethodName = "RGB";
        }

        std::cerr << "ScanMethod " << m_scanMethodName << std::endl;

    }
    else if (command == "CameraSetup") {
        sstr >> m_camWidth >> m_camHeight;
        std::cerr << "CameraSetup " << m_camWidth << " " << m_camHeight << std::endl;
    }
    else if (command == "ColorDetectorSetup") {
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
        cv::resizeWindow(BGR_IMAGE, m_camWidth, m_camHeight);
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
    }
#endif // !__arm__
}

double get_current_time_with_ms (void)
{
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    return  (tv.tv_sec) + (tv.tv_usec) / 1000000 ;

}

const char* colors[] = {"cBLACK", "cWHITE", "cGREY", "cRED", "cORANGE", "cYELLOW", "cGREEN", "cAQUA", "cBLUE", "cPURPLE", "cPINK"};

void ColorDetector::scanHsv()
{
    std::map<const char*, int> tallyColors;

    float blue  = 0.0f;
    float green = 0.0f;
    float red   = 0.0f;

    int pixels = 0;

    float initialConfidence = 1.0f;

    for (std::vector<cv::Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::Mat image(m_bgrImage, *it);

        cv::Mat hsvZone;
        cv::cvtColor(image, hsvZone, CV_BGR2HSV);

        int h = hsvZone.rows;             // Pixel height
        int w = hsvZone.cols;              // Pixel width

        pixels = w * h;

        cv::MatIterator_<cv::Vec3b> it2 = image.begin<cv::Vec3b>(),
        it_end = image.end<cv::Vec3b>();

        for(; it2 != it_end; ++it2)
        {
            cv::Vec3b& pixel = *it2; // reference to pixel in image

            int hVal = pixel[0];
            int sVal = pixel[1];
            int vVal = pixel[2];


	        // Determine what type of color the HSV pixel is.
	        const char* ctype = getPixelColorTypeBH(hVal, sVal, vVal);

	        tallyColors[ctype]+=1;
	    }


        int tallyMaxIndex = 0;
        int tallyMaxCount = -1;


        for (int i=0; i<11; i++) {
            const char* v = colors[i];
            int pixCount = tallyColors[v];

            // TODO : make configurable
            if(pixCount>0) {
                std::cerr << v << " - " << (pixCount*100/pixels) << " %" << std::endl;
            }

            if (pixCount > tallyMaxCount) {
                tallyMaxCount = pixCount;
                tallyMaxIndex = i;
            }
        }

        float percentage = initialConfidence * (tallyMaxCount * 100 / pixels);

        std::cerr << "Color of current note: " << colors[tallyMaxIndex] << " (" << percentage << "% confidence)." << std::endl;

        if (percentage > 80.0)
        {
            if(colors[tallyMaxIndex] == "cYELLOW")
            {
                sendPacket("packets.ColorDetectorFire(color=TEAM_YELLOW)");
            }

            else if(colors[tallyMaxIndex] == "cRED")
            {
                sendPacket("packets.ColorDetectorFire(color=TEAM_RED)");
            }

            else {
                sendPacket("None");
            }
        }
    }
}


const char* getPixelColorType(int H, int S, int V)
{
    char* color;
    if (V < 20)
        color = "cBLACK";
    else if (V > 190 && S < 27)
        color = "cWHITE";
    else if (S < 53 && V < 185)
        color = "cGREY";
    else {  // Is a color
        if (H < 14)
            color = "cRED";
        else if (H < 25)
            color = "cORANGE";
        else if (H < 34)
            color = "cYELLOW";
        else if (H < 73)
            color = "cGREEN";
        else if (H < 102)
            color = "cAQUA";
        else if (H < 127)
            color = "cBLUE";
        else if (H < 149)
            color = "cPURPLE";
        else if (H < 175)
            color = "cPINK";
        else    // full circle
            color = "cRED"; // back to Red
    }
    return color;
}

const char* getPixelColorTypeBH(int H, int S, int V)
{
    // TODO : make configurable
    if (H > 200 )
        return "cRED";

    // TODO : make configurable
    if (H < 20 )
        return "cRED";

    // TODO : make configurable
    if (H > 80 )
        return "cGREEN";

    return "cYELLOW";
}

void ColorDetector::scan()
{
    // TODO : make configurable
    imwrite("image.jpg",m_bgrImage);

    switch(m_scanMethod) {
        default:
        case ScanRgb:
            scanRgb();
            break;
        case ScanHsv:
            scanHsv();
            break;
    }
}


void ColorDetector::scanRgb()
{
    float blue  = 0.0f;
    float green = 0.0f;
    float red   = 0.0f;

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
    if (reference < 0.0f) {
        return value < -reference;
    } else {
        return value > reference;
    }
}


void ColorDetector::sendPacket(const std::string packet)
{
    std::cout << std::setprecision (32) << get_current_time_with_ms() << ", " << packet << std::endl;
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

