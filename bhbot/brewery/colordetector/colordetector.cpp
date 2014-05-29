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
        ScanMask,
        ScanHsv2,
        ScanHsv,
        ScanRgb
    };

    enum Mode {
        ModeReference,
        ModeScan,
        ModePause
    };

public:
    ColorDetector(int webcamId, const std::string& configFile, const std::string& imageFile, bool quiet);
    virtual ~ColorDetector();

    void process();

private:
    void reset();
    bool processLine(std::istream& stream);
    void storeReference(std::string& _color);
    void adaptativeStoreReference(std::string& _color);
    bool wait(double delta);
    void initDisplay();
    void updateDisplay();
    void scan();
    void scanRgb();
    void scanHsv();
    void scanHsv2();
    void scanMask();
    bool testComponent(float value, float reference);
    void sendPacket(const std::string packet);
    void sendPacketColor(std::string & color);
    void sendPacketColor(const char * color);
    void updateMaskWindow(const std::string& color, cv::Mat & img);
    void updateWindow(std::string name, cv::Mat & img);
    void writeImage(std::string name, cv::Mat &img);

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
    std::string m_lastDetectedColorString;
    std::string m_lastSentColorString;
    int m_lastDetectedColorDebounceCounter;
    int m_colorOccurrenceNeeded;
    int m_camWidth;
    int m_camHeight;
    bool m_writeLastCapture;
    bool m_writeReferenceCapture;
    std::string m_currentReferenceColor;
    int m_currentTolerance;
    int m_colorThreshold;
    int m_greenThreshold;
    int m_redThreshold;

    ScanMethod m_scanMethod;
    std::string m_scanMethodName;
    std::map<std::string, std::string> m_maskWindow;
    std::map<std::string, std::string> m_windows;

    Mode m_mode;

    std::map<std::string, cv::Vec3f> m_referenceColors;

    std::string m_logPrefix;
};

class Timer
{
public:
    Timer(bool startIt=false)
    {
        if(startIt)
            start();
    }



    void start()
    {
        gettimeofday(&m_time, NULL);
    }

    double stop()
    {
        struct timeval current;
        gettimeofday(&current, NULL);

        timersub(&current, &m_time, &m_delta);

        m_deltaMs = computeDeltaMs(m_delta);

        return m_deltaMs;

    }

    double tick()
    {
        struct timeval current;
        struct timeval delta;

        gettimeofday(&current, NULL);

        timersub(&current, &m_time, &delta);

        double ret = computeDeltaMs(delta);

        return ret;

    }

    double getDeltaMs()
    {
        return m_deltaMs;
    }

    double computeDeltaMs(struct timeval & diff)
    {
        return diff.tv_sec * 1000.0 + diff.tv_usec / 1000.0;
    }

private:
    struct timeval m_time;
    struct timeval m_delta;
    double m_deltaMs;

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

        m_webcam->read(m_bgrImage);

        double realWidth = m_webcam->get(CV_CAP_PROP_FRAME_WIDTH);
        double realHeight = m_webcam->get(CV_CAP_PROP_FRAME_HEIGHT);

        std::cerr << "Using webcam " << webcamId
                    << " resolution " << m_camWidth << "x" << m_camHeight
                    << " real resolution " << realWidth << "x" << realHeight
                    << std::endl;

        m_camWidth = realWidth;
        m_camHeight = realHeight;

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
    Timer timer;
    bool again = true;

    double delta = 0.0;

    while (std::cin.good() && again) {

        timer.start();

        Timer timerWait(true);
        if (wait(delta)) {
            again = processLine(std::cin);
        }
        timerWait.stop();

        Timer timerCamera(true);

        if (m_webcam != NULL) {
            m_webcam->read(m_bgrImage);
        }

        timerCamera.stop();

        Timer timerProcess(true);

        switch(m_mode)
        {
            case ModeScan:
                scan();
            break;

            case ModeReference:
                adaptativeStoreReference(m_currentReferenceColor);
                m_mode = ModePause;
            break;

            case ModePause:
                m_lastDetectedColorString = "";
            break;
        }



        timerProcess.stop();

        timer.stop();

        updateDisplay();

        if (!m_quiet)
        {
            std::cerr   << std::setprecision(3) << "loop duration " << timer.getDeltaMs() << "ms "
                        << "camera read " << std::setprecision(3) << timerCamera.getDeltaMs() << "ms "
                        << "processing " <<  std::setprecision(3) << timerProcess.getDeltaMs() << "ms "
                        << "wait " <<  std::setprecision(3) << timerWait.getDeltaMs() << "ms "
                        << std::endl;
        }

        delta = timer.getDeltaMs() - timerWait.getDeltaMs();
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
    m_writeLastCapture = false;
    m_currentTolerance = 0;
    m_mode = ModeScan;
    m_colorThreshold = 80;
    m_greenThreshold = 128;
    m_redThreshold = 128;
    m_writeReferenceCapture = false;
    m_lastDetectedColorDebounceCounter = 0;
    m_colorOccurrenceNeeded = 1;
}

void ColorDetector::writeImage(std::string name, cv::Mat &img)
{
    std::stringstream ss;

    ss << m_logPrefix << "_" << name;

    imwrite(ss.str().c_str(),img);
}


bool ColorDetector::processLine(std::istream& stream)
{
    bool again = true;

    std::string line;
    std::getline(stream, line);
    std::stringstream sstr(line);
    std::string command;

    sstr >> command;

    if (command == "WriteLastCapture")
    {
        std::string mode;
        sstr >> mode;

        m_writeLastCapture = mode == "1";

        std::cerr << "WriteLastCapture " << m_writeLastCapture << std::endl;
    }
    else if (command == "WriteReferenceCapture")
    {
        std::string mode;
        sstr >> mode;

        m_writeReferenceCapture = mode == "1";

        std::cerr << "WriteReferenceCapture " << m_writeReferenceCapture << std::endl;
    }
    else if (command == "GreenThreshold")
    {
        sstr >> m_greenThreshold;

        std::cerr << "GreenThreshold " << m_greenThreshold << std::endl;
    }
    else if (command == "RedThreshold")
    {
        sstr >> m_redThreshold;

        std::cerr << "RedThreshold " << m_redThreshold << std::endl;
    }
    else if (command == "SetLoopInterval")
    {
        sstr >> m_pollTimeoutMs;

        std::cerr << "SetLoopInterval " << m_pollTimeoutMs << std::endl;
    }
    else if (command == "SetColorThreshold")
    {
        sstr >> m_colorThreshold;

        std::cerr << "SetColorThreshold " << m_colorThreshold << std::endl;
    }
    else if (command == "ColorOccurrenceNeeded")
    {
        sstr >> m_colorOccurrenceNeeded;

        std::cerr << "ColorOccurrenceNeeded " << m_colorOccurrenceNeeded << std::endl;
    }
    else if (command == "StoreReference")
    {
        sstr >> m_currentReferenceColor;

        m_mode = ModeReference;

        std::cerr << "StoreReference " << m_currentReferenceColor << std::endl;
    }
    else if (command == "EnableScan")
    {
        m_mode = ModeScan;
        std::cerr << "Scan mode enabled" << std::endl;
    }
    else if (command == "DisableScan")
    {
        m_mode = ModePause;
        std::cerr << "Scan mode disabled" << std::endl;
    }
    else if (command == "SetLogPrefix")
    {

        sstr >> m_logPrefix;

        std::cerr << "SetLogPrefix " << m_logPrefix << std::endl;
    }
    else if (command == "SetReferenceTolerance")
    {
        sstr >> m_currentTolerance;

        std::cerr << "SetReferenceTolerance " << m_currentTolerance << std::endl;
    }
    else if (command == "ScanMethod") {
        std::string mode;
        sstr >> mode;

        if (mode == "HSV") {
            m_scanMethod = ScanHsv;
            m_scanMethodName = "HSV";
        }
        else if (mode == "HSV2") {
            m_scanMethod = ScanHsv2;
            m_scanMethodName = "HSV2";
        } else if (mode == "MASK") {
            m_scanMethod = ScanMask;
            m_scanMethodName = "MASK";
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

void ColorDetector::adaptativeStoreReference(std::string &_color)
{

    for(int i=0;i<30;i++)
    {
        m_lastDetectedColorString = "";
        storeReference(_color);
        scanHsv2();

        if(m_lastDetectedColorString == _color)
        {
            std::cerr << "CALIBRATION SUCCESSFUL" << std::endl;
            return;
        } else {
            std::cerr << "CALIBRATION FAILED --> TRY AGAIN" << std::endl;
            m_currentTolerance++;
            std::cerr << "TOLERANCE set to " << m_currentTolerance << std::endl;
        }

    }

    std::cerr << "CALIBRATION FUCKED UP" << std::endl;

}

void ColorDetector::storeReference(std::string &_color)
{
    float calibH = 0.0;
    float calibS = 0.0;
    float calibV = 0.0;

    // TODO: store delta with color
    int delta = 15;

    for (std::vector<cv::Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
        cv::Mat image(m_bgrImage, *it);

        cv::Mat hsvZone;
        cv::cvtColor(image, hsvZone, CV_BGR2HSV);

        std::vector<cv::Mat> components;
        cv::split(image, components);

        //calibH += cv::mean(components[0])[0];

        cv::MatIterator_<cv::Vec3b> it2 = hsvZone.begin<cv::Vec3b>(),
                it_end = hsvZone.end<cv::Vec3b>();

        int pixCount = 0;

        unsigned int pixVal=0;

        for(; it2 != it_end; ++it2)
        {
            cv::Vec3b& pixel = *it2; // reference to pixel in image

            int pixOrig = int(pixel[0]);

            int pixTrans = ( pixOrig + delta) % 180;

            //std::cerr << "pixel " << pixOrig << "->" <<  pixTrans << std::endl;

            pixVal += pixTrans;

            pixCount++;

        }

        std::cerr << "pixVal " << pixVal << std::endl;
        std::cerr << "pixCount " << pixCount << std::endl;
        std::cerr << "pixVal/pixCount " << pixVal/pixCount << std::endl;

        calibH += ( (pixVal/pixCount) - delta ) % 180;
        calibS += cv::mean(components[1])[0];
        calibV += cv::mean(components[2])[0];

    }

    calibH/=m_detectionZoneRects.size();
    calibS/=m_detectionZoneRects.size();
    calibV/=m_detectionZoneRects.size();

    cv::Vec3f vec = cv::Vec3f(calibH, calibS, calibV);

    m_referenceColors[_color] = vec;

    std::cerr << "Stored color " << _color << " as " << calibH << " " << calibS << " " << calibV << std::endl;
    std::cerr << "Stored color " << _color << " as " << vec.val[0] << " " << vec.val[1] << " " << vec.val[2] << std::endl;

    std::stringstream ss;

    ss << "stored_" << _color << "_" << calibH <<"_"<<calibS<<"_"<<calibV<<".jpg";

    if(m_writeReferenceCapture)
    {
        writeImage(ss.str(),m_bgrImage);
    }
}


bool ColorDetector::wait(double delta)
{
    struct pollfd fds[1];
    fds[0].fd = 0;
    fds[0].events = POLLIN;

    int rv = poll(fds, 1, std::max(m_pollTimeoutMs-delta,0.0));

    return rv > 0;
}


void ColorDetector::initDisplay()
{
#ifndef __arm__
    if (!m_quiet) {
        cv::namedWindow(BGR_IMAGE, cv::WINDOW_NORMAL);
        cv::resizeWindow(BGR_IMAGE, m_camWidth, m_camHeight);

//        cv::namedWindow("GRAY", cv::WINDOW_NORMAL);
        cv::namedWindow("GREEN MASK", cv::WINDOW_NORMAL);
        cv::resizeWindow("GREEN MASK", m_camWidth, m_camHeight);
//        cv::namedWindow("REDMASK", cv::WINDOW_NORMAL);
//        cv::namedWindow("REDMASK2", cv::WINDOW_NORMAL);

//        cv::namedWindow("YELLOW TRI", cv::WINDOW_NORMAL);
//        cv::namedWindow("RED TRI", cv::WINDOW_NORMAL);
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

        if (percentage > m_colorThreshold)
        {
            if(std::strcmp(colors[tallyMaxIndex], "cYELLOW") == 0)
            {
                sendPacketColor("COLOR_YELLOW");
            }

            else if(std::strcmp(colors[tallyMaxIndex], "cRED") == 0)
            {
                sendPacketColor("COLOR_RED");
            }

            else
            {
                sendPacketColor("COLOR_NONE");
            }
        }
    }
}

void ColorDetector::sendPacketColor(std::string & color)
{
    if( m_lastDetectedColorString == color)
    {
        m_lastDetectedColorDebounceCounter++;
    }
    else
    {
        m_lastDetectedColorString = color;
        m_lastDetectedColorDebounceCounter = 1;
    }

    if( m_lastDetectedColorDebounceCounter >= m_colorOccurrenceNeeded )
    {
        if( m_lastSentColorString != m_lastDetectedColorString)
        {
            sendPacket(std::string("packets.ColorDetected(color=")+color+")");
            m_lastSentColorString = color;
        }
    }
}

void ColorDetector::sendPacketColor(const char* color)
{
    std::string tmp(color);
    sendPacketColor(tmp);
}

void ColorDetector::scanHsv2()
{
    bool hasSeenColor = false;

    for (std::vector<cv::Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it)
    {
        cv::Mat image(m_bgrImage, *it);

        cv::Mat hsvZone;
        cv::cvtColor(image, hsvZone, CV_BGR2HSV);

        int h = hsvZone.rows;             // Pixel height
        int w = hsvZone.cols;              // Pixel width

        int totalPixels = w * h;

        std::map<std::string, cv::Vec3f>::iterator itRefColor;

        std::map<std::string, int> colorTotal;

        for(itRefColor=m_referenceColors.begin(); itRefColor!=m_referenceColors.end(); itRefColor++)
        {
            int nbPixels = 0;

            cv::Mat mask;
            cv::Mat mask2;
            cv::Mat kernel;

            float h = itRefColor->second[0];

            // TODO : s tolerance is disabled, should we use a separate tolerance ?

            if(h - m_currentTolerance > 0)
            {

                // We create the mask
                cv::inRange(hsvZone
                            , cv::Scalar(h - m_currentTolerance -1, 0 /*s - m_currentTolerance*/, 0)
                            , cv::Scalar(h + m_currentTolerance -1, 255 /*s + m_currentTolerance*/, 255)
                            , mask
                            );
            } else {

                int vMax = h + m_currentTolerance -1;
                int vMin = 128 + h - m_currentTolerance -1;

                // We create the mask
                cv::inRange(hsvZone
                            , cv::Scalar(0, 0 /*s - m_currentTolerance*/, 0)
                            , cv::Scalar(vMax, 255 /*s + m_currentTolerance*/, 255)
                            , mask
                            );

                cv::inRange(hsvZone
                            , cv::Scalar(vMin, 0 /*s - m_currentTolerance*/, 0)
                            , cv::Scalar(128, 255 /*s + m_currentTolerance*/, 255)
                            , mask2
                            );

                mask |= mask2;

            }

            // TODO : erode + dilate picture
            // int erosion_size = 1;

            // Create kernels for the morphological operation
            //kernel = cv::CreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
            //kernel = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
            //                                   cv::Point( erosion_size, erosion_size ));

            // Morphological opening (inverse because we have white pixels on black background)
            //cv::Dilate(mask, mask, kernel, 1);
            //cv::dilate(mask, mask, kernel);
            //cv::Erode(mask, mask, kernel, 1);
            //cv::erode(mask, mask, kernel);

            updateMaskWindow(itRefColor->first, mask);

            // TODO : output center of gravity

            // We go through the mask to look for the tracked object and get its gravity center
            for(int i=0; i<mask.rows; i++) {
                for(int j=0; j<mask.cols; j++) {

                    // If its a tracked pixel, count it to the center of gravity's calcul
                    if(((uchar)(mask.at<cv::Vec3b>(i,j)[0])) == 255) {
                        //sommeX += i;
                        //sommeY += j;
                        nbPixels++;
                    }
                }
            }

            colorTotal[itRefColor->first] = nbPixels;
        }

        std::map<std::string, int>::iterator itColorTotal;

        std::string detectedColor="";
        int maxValue=0;

        for(itColorTotal=colorTotal.begin(); itColorTotal!=colorTotal.end(); itColorTotal++)
        {
            if(itColorTotal->second > 0)
            {
                if(!m_quiet)
                {
                    std::cerr << itColorTotal->first << " " << itColorTotal->second << std::endl;
                }
            }

            if(itColorTotal->second > maxValue)
            {
                maxValue = itColorTotal->second;
                detectedColor = itColorTotal->first;
            }
        }

        if(maxValue>0)
        {
            float ratio = float(maxValue*100) / totalPixels;

            std::cerr << "MAIN COLOR : "<< detectedColor << " " << std::setprecision(2) << ratio << "%" << std::endl;

            if(ratio > m_colorThreshold)
            {
                std::cerr << "DETECTED COLOR : "<< detectedColor << std::endl;

                sendPacketColor(detectedColor);
                hasSeenColor = true;
            }
        }
    }

    if(!hasSeenColor)
    {
        sendPacketColor("COLOR_NONE");
    }
}

void ColorDetector::scanMask()
{
    bool hasSeenColor = false;

    for (std::vector<cv::Rect>::iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it)
    {
        cv::Mat image(m_bgrImage, *it);

        cv::Mat hsvZone;
        cv::cvtColor(image, hsvZone, CV_BGR2HSV);

        std::map<std::string, cv::Vec3f>::iterator itRefColor;

        std::map<std::string, int> colorTotal;

        cv::Mat maskRed;
        cv::Mat maskGreen;
        cv::Mat maskLum;

        std::vector<cv::Mat> componentsBgr;
        cv::split(image, componentsBgr);

        cv::threshold(componentsBgr[2], maskRed, m_redThreshold, 255, cv::THRESH_BINARY);


//// -----------------

        double meanRed = cv::mean(maskRed)[0];

        if (!m_quiet) {
            std::cerr << "MEAN RED " << meanRed << std::endl;
        }


        cv::threshold(componentsBgr[1], maskGreen, m_greenThreshold, 255, cv::THRESH_BINARY);

#ifndef __arm__
        updateWindow("GREEN MASK", maskGreen);
#endif // !__arm__

        double meanGreen = cv::mean(maskGreen)[0];

        if (!m_quiet) {
            std::cerr << "MEAN GREEN " << meanGreen << std::endl;
        }

        if (meanRed < 128)
        {
            break;
        }

        if (meanGreen > 128)
        {
            hasSeenColor = true;
            sendPacketColor("COLOR_YELLOW");
        }
        else
        {
            hasSeenColor = true;
            sendPacketColor("COLOR_RED");
        }


// ----------------------

/* WORKING

        cv::inRange(image
                            , cv::Scalar(0,0,0)
                            , cv::Scalar(255,128,255)
                            , maskGreen
                            );

        cv::cvtColor(image, maskLum, CV_BGR2GRAY);

        cv::imshow("REDMASK", maskRed);
        cv::imshow("REDMASK2", maskGreen);
        cv::imshow("GRAY", maskLum);

        cv::Mat maskGreenInv = 255 - maskGreen;

        cv::Mat maskYellow = maskRed & maskGreenInv;
        cv::Mat maskRedFinal = maskRed & maskGreen;

        cv::imshow("YELLOW TRI", maskYellow);
        cv::imshow("RED TRI", maskRedFinal);


        double meanYellow = cv::mean(maskYellow)[0];
        double meanRed = cv::mean(maskRedFinal)[0];

        if (meanYellow > 128)
        {
            std::cerr << "YELLOW" << std::endl;
            hasSeenColor = true;
            sendPacketColor("COLOR_YELLOW");
        }
        else
        {
            if (meanRed > 128) {
                std::cerr << "RED" << std::endl;
                hasSeenColor = true;
                sendPacketColor("COLOR_RED");
            }
        }
*/ /* WORKING END */


    }

    if(!hasSeenColor)
    {
        sendPacketColor("COLOR_NONE");
    }
}

const char* getPixelColorType(int H, int S, int V)
{
    const char* color;
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
    if(m_writeLastCapture) {
        cv::Mat img = m_bgrImage.clone();
        for (std::vector<cv::Rect>::const_iterator it = m_detectionZoneRects.begin(); it != m_detectionZoneRects.end(); ++it) {
            cv::rectangle(img, cv::Point(it->x, it->y), cv::Point(it->x + it->width, it->y + it->height), cv::Scalar(0, 200, 0));
        }

        imwrite("image.jpg",img);
    }

    switch(m_scanMethod) {
    default:
    case ScanRgb:
        scanRgb();
        break;
    case ScanHsv2:
        scanHsv2();
        break;
    case ScanHsv:
        scanHsv();
        break;
    case ScanMask:
        scanMask();
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
        // TODO : factor between scan methods
        if (m_lastDetectedColor != ColorRed) {
            m_lastDetectedColor = ColorRed;
            sendPacket("packets.ColorDetected(color=TEAM_RED)");
        }
    } else if (testComponent(blue,  m_yellowFireBlueRef)  &&
               testComponent(green, m_yellowFireGreenRef)        &&
               testComponent(red,   m_yellowFireRedRef)) {
        // TODO : factor between scan methods
        if (m_lastDetectedColor != ColorYellow) {
            m_lastDetectedColor = ColorYellow;
            sendPacket("packets.ColorDetected(color=TEAM_YELLOW)");
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

void ColorDetector::updateMaskWindow(const std::string& color, cv::Mat & img)
{
#ifndef __arm__
    if (!m_quiet)
    {
        if( m_maskWindow.find(color) == m_maskWindow.end() )
        {
            std::string name = std::string("MASK "+color);

            m_maskWindow[color]=name;
        }

        updateWindow(m_maskWindow[color].c_str(), img);
    }
#endif
}


void ColorDetector::updateWindow(std::string name, cv::Mat & img)
{
#ifndef __arm__
    if (!m_quiet)
    {
        const char* constName = name.c_str();

        if( m_windows.find(name) == m_windows.end() )
        {
            cv::namedWindow(constName, cv::WINDOW_NORMAL);
            cv::resizeWindow(constName, m_camWidth, m_camHeight);
            cv::moveWindow(constName,(m_windows.size()+1)*(m_camWidth+5),0);

            m_windows[name]=name;
        }

        cv::imshow(constName, img);
    }
#endif
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

    std::cerr << "end" << std::endl;

    return 0;
}

