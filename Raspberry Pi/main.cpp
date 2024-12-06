//
// main.cpp
// Resistor Sorter
//
// Created by Justin Adams
//
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>

// Converts resistor value from string into a double
double convertValue(const std::string& input) {

  // Extract the number
  size_t i = 0;
  while (i < input.size() && (isdigit(input[i]) || input[i] == '.')) {
        i++;
  }

  // Save numeric value
  double number = std::stod(input.substr(0, i));

  // Multiply by suffix
  if (i < input.size()) {
      char suffix = std::tolower(input[i]);
      switch (suffix) {
          case 'k': // kilo
              number *= 1e3;
              break;
          case 'm': // Mega
              number *= 1e6;
              break;
          default:
              throw std::invalid_argument("Invalid suffix: " + std::string(1, suffix));
      }
  }
          
  return number;
  
}

// Send data over serial
void sendData(boost::asio::serial_port& serial, const std::string& data) {
    boost::asio::write(serial, boost::asio::buffer(data));
}


int main(int argc, char *argv[]) {
    // Vectors for operation
    double resistors[4] = {0,0,0,0};
    
    // Check for input arguments
    if (argc < 2 || argc > 5) {
        std::cerr << "Usage : ./sorter <resistor_value_1> <resistor_value_2> <resistor_value_3> <resistor_value_4>\n\n"
        << "User must enter between 1 and 4 values greater than 0. Can include 'k' (for kilo) or 'M' (for mega).\n"
        << "Example: ./sorter 100 1k 10M 0.5k\n\n"
        << "Valid formats:\n"
        << " - <number> (e.g., 100, 5000)\n"
        << " - <number>k (e.g., 1k for 1000)\n"
        << " - <number>M (e.g., 10M for 10000000)\n" << std::endl;
        return -1;
    }    
    
    // Initialize serial communication
    boost::asio::io_service io;
    boost::asio::serial_port serial(io, "/dev/ttyACM0"); // Use the appropriate serial port

    serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    // Calls function to convert resistors values to doubles and sorts them
    for(int i = 1; i < argc; i++) {
        // Converts values
        resistors[i-1] = convertValue(argv[i]);
    } 
    
    // Create a string of the sorted vector
    std::ostringstream oss;
    
    // Loop through and add all values to string
    for (int i = 0; i < 4; ++i) {
        oss << resistors[i];
        if (i != 3) { // Add a space unless it's the last element
            oss << " ";
        }
    }
    
    std::string bins = oss.str();

    // Send the 
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    sendData(serial, bins + "\n");
    std::cout << "Sent resistor values: " << bins << std::endl;
    
    // Wait on ready from arduino
    boost::asio::streambuf buf;
    boost::asio::read_until(serial, buf, 'A');  // Wait until 'A' is received
    
    // Use libcamera to capture an image
    system("libcamera-jpeg -o image.jpg --nopreview --width 1080 --height 1080");

    // Load the captured image
    cv::Mat image = cv::imread("image.jpg");
    if (image.empty()) {
        std::cerr << "Error: Could not load the captured image!" << std::endl;
        return -1;
    }

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Define the range for light brown
    cv::Mat brownMask;
    cv::inRange(hsv, cv::Scalar(10, 50, 100), cv::Scalar(20, 150, 200), brownMask);

    // Define the range for blue
    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(100, 50, 50), cv::Scalar(140, 255, 255), blueMask);

    // Combine both masks
    cv::Mat combinedMask = brownMask | blueMask;

    // Apply morphological closing to fill color band gaps
    cv::Mat closedMask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    cv::morphologyEx(combinedMask, closedMask, cv::MORPH_CLOSE, kernel);


    // Apply dilation to smooth out the mask
    cv::Mat dilatedMask;
    cv::dilate(closedMask, dilatedMask, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilatedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Size constraints
    double minArea = 200.0; // Minimum area
    double maxArea = 1000.0; // Maximum area

    // Get image dimensions
    int imageWidth = image.cols;
    int imageHeight = image.rows;
    
    // Define boundary region (10 pixels around the edge of the image)
    int boundary = 12;
    
    // Remove contours that are too big or small, or within the boundary region
    contours.erase(
        std::remove_if(contours.begin(), contours.end(),
            [minArea, maxArea, boundary, imageWidth, imageHeight](const std::vector<cv::Point>& contour) {
                // Get the bounding box of the contour
                cv::Rect boundingBox = cv::boundingRect(contour);
                
                // Check if the contour's bounding box is within the boundary region
                bool isInBoundary = boundingBox.x < boundary || boundingBox.y < boundary ||
                                    boundingBox.x + boundingBox.width > imageWidth - boundary ||
                                    boundingBox.y + boundingBox.height > imageHeight - boundary;
    
                // Calculate the area of the contour
                double area = cv::contourArea(contour);
    
                // Return true to remove contours that are too small/large or within the boundary region
                return area < minArea || area > maxArea || isInBoundary;
            }),
        contours.end()
    );

    // Vector to store the centers of the remaining contours
    std::vector<cv::Point2f> contourCenters;

    // Calculate the center of each remaining contour
    for (const auto& contour : contours) {
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 != 0) {
            float cX = moments.m10 / moments.m00; // Center X
            float cY = moments.m01 / moments.m00; // Center Y
            contourCenters.push_back(cv::Point2f(cX, cY));
        }
    }
        
    // Draw resistor contours and their centers on the original image
    cv::Mat contourOutput = image.clone();
    cv::drawContours(contourOutput, contours, -1, cv::Scalar(0, 255, 0), 2);
    for (const auto& center : contourCenters) {
        cv::circle(contourOutput, center, 5, cv::Scalar(0, 0, 255), -1);
    }

    // Preview marked image
    cv::imshow("Detected Image", contourOutput);
    cv::waitKey(0);

    // Top-left corner of the image
    cv::Point2f topLeftCorner(0, 0);

    // Sort the centers by distance to the top-left corner
    std::sort(contourCenters.begin(), contourCenters.end(),
        [&topLeftCorner](const cv::Point2f& a, const cv::Point2f& b) {
            // Calculate squared distances to avoid sqrt computation
            float distA = std::pow(a.x - topLeftCorner.x, 2) + std::pow(a.y - topLeftCorner.y, 2);
            float distB = std::pow(b.x - topLeftCorner.x, 2) + std::pow(b.y - topLeftCorner.y, 2);
            return distA < distB; // Sort in ascending order
        });


    // Send the resistor locations to the arduino
    for (size_t i = 0; i < contourCenters.size(); ++i) {
        // Convert pixels to cm and move origin from top to bottom
        contourCenters[i].x /= 50.0;
        contourCenters[i].y = (1080.0 - contourCenters[i].y) / 50.0; 
        
        // Round to the tenths place
        contourCenters[i].x = std::round(contourCenters[i].x * 10.0) / 10.0; 
        contourCenters[i].y = std::round(contourCenters[i].y * 10.0) / 10.0;
        
        // Account that origin of image is not origin of arm
        contourCenters[i].x = contourCenters[i].x + 5.5; 
        contourCenters[i].y = contourCenters[i].y + 5.5;
        
        // Send coordinates to Arduino
        std::ostringstream coordinates;
        coordinates << contourCenters[i].x << " " << contourCenters[i].y << "\n";
        sendData(serial, coordinates.str());
        std::cout << "Sent: " << coordinates.str();
        
        // Wait on ready from arduino
        boost::asio::streambuf buf;
        boost::asio::read_until(serial, buf, 'A');  // Wait until 'A' is received
        
    }

    // Notify Arduino of finish
    sendData(serial, "DONE\n");

    return 0;
}
