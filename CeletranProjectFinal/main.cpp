// pragma declaration



#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3\detection\vpDetectorFace.h>

#include <opencv2/highgui/highgui.hpp>

#include "Serial.h"
#include <iomanip>
#include <dos.h>


int num = 1;
int check = 1;
int movement_left = 0;
int movement_right = 0;
double Rect_X = 0, Rect_Y = 0;
int counterX = 0;
int counterY = 0;
std::string azimuth = "00000000";
std::string altitude = "00000000";

HANDLE h;


typedef enum {
	capture_waiting,
	capture_started,
	capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;



bool s_face_available = false;

cv::Mat s_frame;
vpMutex s_mutex_capture;
vpMutex s_mutex_face;
vpRect s_face_bbox;




bool Read(HANDLE h, char *buffer, int bufflen)
{
	DWORD NumBytesToRead;
	DWORD numbytesWritten;
	bool returner;
	bool checker = WriteFile(h, buffer, bufflen, &numbytesWritten, NULL);
	while (checker)
	{
		returner = ReadFile(h, buffer, bufflen, &NumBytesToRead, NULL);
		std::cout << "Buffer    :  " << buffer << std::endl;
		
	}
	return returner;
}

#pragma region Display Thread

vpThread::Return displayFunction(vpThread::Args args)
{

	//	(void)args; // Avoid warning: unused parameter args
	vpImage<unsigned char> I_;

	t_CaptureState capture_state_;
	bool display_initialized_ = false;
	bool face_available_ = false;
	vpRect face_bbox;

	vpDisplayGDI *d_ = NULL;


	do {
		s_mutex_capture.lock();
		capture_state_ = s_capture_state;
		s_mutex_capture.unlock();

		// Check if a frame is available
		if (capture_state_ == capture_started) {
			// Get the frame and convert it to a ViSP image used by the display class
			{
				vpMutex::vpScopedLock lock(s_mutex_capture);
				vpImageConvert::convert(s_frame, I_);
			}

			// Check if we need to initialize the display with the first frame
			if (!display_initialized_) {
				// Initialize the display

#if defined(VISP_HAVE_GDI)
				d_ = new vpDisplayGDI(I_);
				display_initialized_ = true;
#endif
			}


			// Display the image
			vpDisplay::display(I_);

			// Check if face is available
			vpMutex::vpScopedLock lock(s_mutex_face);
			face_available_ = s_face_available;
			face_bbox = s_face_bbox;

			if (face_available_)
			{
				// Access to the face bounding box to display it
				vpDisplay::displayRectangle(I_, face_bbox, vpColor::red, false, 4);
				face_available_ = false;
				
			}

			// Trigger end of acquisition with a mouse click
			vpDisplay::displayText(I_, 10, 10, "exit with a click", vpColor::blue);
			if (vpDisplay::getClick(I_, false)) {
				vpMutex::vpScopedLock lock(s_mutex_capture);
				s_capture_state = capture_stopped;
			}

			// Update the display
			vpDisplay::flush(I_);
		}
		else {
			vpTime::wait(2); // Sleep 2ms
		}
	} while (capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
	delete d_;
#endif

	std::cout << "End of display thread" << std::endl;
	return 0;
}

#pragma endregion

#pragma region Video Capture Thread

vpThread::Return captureFunction(vpThread::Args args)
{
	// cv::VideoCapture cap = *((cv::VideoCapture *) args);
	cv::VideoCapture cap(1);

	if (!cap.isOpened()) { // check if we succeeded
		std::cout << "Unable to start capture" << std::endl;
		return 0;
	}

	cv::Mat frame_;
	bool stop_capture_ = false;

	double start_time = vpTime::measureTimeSecond();
	while ((vpTime::measureTimeSecond() - start_time) < 300 && !stop_capture_) {
		// Capture in progress
		cap >> frame_; // get a new frame from camera

					   // Update shared data
		{
			vpMutex::vpScopedLock lock(s_mutex_capture);
			if (s_capture_state == capture_stopped)
				stop_capture_ = true;
			else
				s_capture_state = capture_started;
			s_frame = frame_;
		}
	}

	{
		vpMutex::vpScopedLock lock(s_mutex_capture);
		s_capture_state = capture_stopped;
	}

	std::cout << "End of capture thread" << std::endl;
	return 0;
}
//! [capture-multi-threaded captureFunction]
#pragma endregion


#pragma region Face Detection Thread

vpThread::Return detectionFunction(vpThread::Args args)
{
	std::string opt_face_cascade_name = *((std::string *) args);
	vpDetectorFace face_detector_;
	face_detector_.setCascadeClassifierFile(opt_face_cascade_name);
	t_CaptureState capture_state_;
#if defined(VISP_HAVE_V4L2)
	vpImage<unsigned char> frame_;
#elif defined(VISP_HAVE_OPENCV)
	cv::Mat frame_;
#endif
	do {
		s_mutex_capture.lock();
		capture_state_ = s_capture_state;
		s_mutex_capture.unlock();
		// Check if a frame is available
		if (capture_state_ == capture_started)
		{
			// Backup the frame
			{
				vpMutex::vpScopedLock lock(s_mutex_capture);
				frame_ = s_frame;
			}
			// Detect faces
			bool face_found_ = face_detector_.detect(frame_);
			if (face_found_)
			{
				vpMutex::vpScopedLock lock(s_mutex_face);
				s_face_available = true;
				s_face_bbox = face_detector_.getBBox(0); // Get largest face bounding box
				s_face_bbox.getCenter(Rect_X, Rect_Y);
			}
			else
			{
				s_face_available = false;
			}
		}
		else 
		{
			vpTime::wait(2); // Sleep 2ms
		}
		
	} while (capture_state_ != capture_stopped);
	std::cout << "End of face detection thread" << std::endl;
	return 0;
}
#pragma endregion

#pragma region Serial Communication
vpThread::Return SerialCommunication(vpThread::Args args)
{
	while (num == 1)
	{
		std::cout << "Opening Com port" << std::endl;
		tstring commPortName(L"COM3");
		Serial serial(commPortName, 9600);
		num = 0;
		std::cout << "COM Port Opened" << std::endl;
		serial.Flush();

		while (true)
		{
			/*if (Rect_X < 290 && Rect_X != 0)
			{
				std::stringstream ss;
				ss << std::hex << std::setw(8) << std::setfill('0') << counterX;
				std::cout << ss.str() << std::endl;
				std::cout << "stringstream  x  :  " << ss.str() << std::endl;
				azimuth = ss.str();
				std::cout << "azimuth  :  " << azimuth << std::endl;
				counterX = counterX - 8888888;

			}
			else
			{
				if (Rect_X > 350 && Rect_X !=0)
				{
					std::stringstream ss;
					ss << std::hex << std::setw(8) << std::setfill('0') << counterX;
					std::cout << ss.str() << std::endl;
					std::cout << "stringstream  x  :  " << ss.str() << std::endl;
					azimuth = ss.str();
					std::cout << "azimuth  :  " << azimuth << std::endl;
					counterX = counterX + 8888888;
				}
				else
				{
					std::cout << "Horizontal center reached" << std::endl;
				}
			}*/

			
				std::stringstream ssX;
				ssX << std::hex << std::setw(8) << std::setfill('0') << counterX;
				std::cout << ssX.str() << std::endl;
				std::cout << "stringstream  x  :  " << ssX.str() << std::endl;
				azimuth = ssX.str();
				std::cout << "azimuth  :  " << azimuth << std::endl;
				if (Rect_X < 290 && Rect_X != 0)
					counterX = counterX - 8888888;
				else if (Rect_X > 350 && Rect_X != 0)
					counterX = counterX + 8888888;
				else
					std::cout << "Horizontal center reached" << std::endl;
	
				std::stringstream ss;
				ss << std::hex << std::setw(8) << std::setfill('0') << counterY;
				std::cout << ss.str() << std::endl;
				std::cout << "stringstream Y  :  " << ss.str() << std::endl;
				altitude = ss.str();
				std::cout << "altitude  :  " << altitude << std::endl;
				if (Rect_Y < 210 && Rect_Y != 0)
				counterY = counterY - 8888888;
			else if (Rect_Y > 270 && Rect_Y != 0)
					counterY = counterY + 8888888;
				else
					std::cout << "Vertical center reached" << std::endl;

			std::string sendCmd = "b" + azimuth + "," + altitude;
			bool checking = serial.Write(sendCmd.c_str(), 18);
			Sleep(500);

		}
	}


	return(0);



}

#pragma endregion

#pragma region Main Thread

int main(int argc, const char* argv[])
{

	std::string opt_face_cascade_name = "C:\\Users\\Admin\\Documents\\telescope\\SerialComm\\x64\\Debug\\haarcascade_frontalface_alt.xml";
	unsigned int opt_device = 0;
	unsigned int opt_scale = 1; // Default value is 2 in the constructor. Turn it to 1 to avoid subsampling

								// Command line options
	for (int i = 0; i<argc; i++) {
		if (std::string(argv[i]) == "--haar")
			opt_face_cascade_name = std::string(argv[i + 1]);
		else if (std::string(argv[i]) == "--device")
			opt_device = atoi(argv[i + 1]);
		else if (std::string(argv[i]) == "--scale")
			opt_scale = (unsigned int)atoi(argv[i + 1]);
		else if (std::string(argv[i]) == "--help") {
			std::cout << "Usage: " << argv[0] << " [--haar <haarcascade xml filename>] [--device <camera device>] [--scale <subsampling factor>] [--help]" << std::endl;
			return 0;
		}
	}

	// Instanciate the capture
	cv::VideoCapture cap;
	cap.open(opt_device);

#  if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
	int width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, width / opt_scale);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#  else
	int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width / opt_scale);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#  endif

	// Start the threads
	vpThread thread_capture((vpThread::Fn)captureFunction, (vpThread::Args)&cap);
	vpThread thread_display((vpThread::Fn)displayFunction);
	vpThread thread_detection((vpThread::Fn)detectionFunction, (vpThread::Args)&opt_face_cascade_name);
	vpThread thread_Serial((vpThread::Fn)SerialCommunication, (vpThread::Args)&cap);

	// Wait until thread ends up
	thread_capture.join();
	thread_display.join();
	thread_detection.join();
	thread_Serial.join();

	return 0;
}

#pragma endregion