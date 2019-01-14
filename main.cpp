#include <cstdlib>
#include <cmath> 
#include <vector>  
#include <numeric>                                                             
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

/*
 * function that finds the edges of a given frame. 
*/
Mat FindEdges(Mat &frame){

    // output frame that will hold the edges that are found
    Mat output;
    
    // convert input frame to greyscale
    cvtColor(frame, output, CV_BGR2GRAY);

    // perform a gaussian blur on the frame
    Size blurSize = Size(5, 5);
    blur(output, output, blurSize);

    // find the edges in the frame using the
    // Canny edge detector
    Canny(output, output, 50, 100);

    // return processed frame
    return output.clone();
}

// frame that simply returns a triangular ROI mask based on the
// dimensions of a given frame. 
Mat ROIMask(Mat &frame){

    // declare a blank mask, color it black
    Mat mask(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
    
    // create a triangular contour based on the points. this represents
    // the ROI
    vector<vector<Point>> contour;
    contour.push_back(vector<Point>());
    contour[0].push_back(Point(0, frame.rows));
    contour[0].push_back(Point( frame.cols/2.2, frame.rows/2.3 ));
    contour[0].push_back(Point(frame.cols, frame.rows));

    // draw a triangular contour onto the black mask, and invert it.
    drawContours( mask,contour,0, Scalar(255),CV_FILLED, 8 );
    bitwise_not(mask, mask);

    // return the created mask
    return mask.clone();
}

// function that crops a given frame according to an ROI mask
void CropROI(Mat &frame, Mat &mask){
    Mat blackMat(frame.rows, frame.cols, frame.type(), cv::Scalar::all(0));
    blackMat.copyTo(frame,mask);
}

// function that creates a line representation of 2 points, given
// a slope and y intercept.
void calculate_lines(Mat &img, float slope, float intercept, Vec4i &line){
    int y1 = img.rows;
    int y2 = int(y1 * 0.5);

    int x1 = (y1 - intercept)/slope;
    int x2 = (y2 - intercept)/slope;

    line[0] = x1;
    line[1] = y1;
    line[2] = x2;
    line[3] = y2;

}

// function that finds the lane lines 
void FindLines(Mat &edges, Vec4i &left_line, Vec4i &right_line){

    // find the hough lines of the given edge mat
    vector<Vec2f> lines;
    HoughLines(edges, lines, 2, CV_PI/180, 100, 0, 0 );

    // vectors used to find the average slopes and y intercepts
    // of the left and right lane lines
    vector<float> slopes_left;
    vector<float> y_intercepts_left;

    vector<float> slopes_right;
    vector<float> y_intercepts_right;

    // iterate through each hough line
    for( size_t i = 0; i < lines.size(); i++ )
    {
        // perform some normalization
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = ( cvRound(x0 + 1000*(-b)) ) ;
        pt1.y = ( cvRound(y0 + 1000*(a)) );
        pt2.x = ( cvRound(x0 - 1000*(-b)) );
        pt2.y = ( cvRound(y0 - 1000*(a)));

        // find the slope and y intercept of this line
        double slope = ((double)pt2.y - (double)pt1.y)/((double)pt2.x - (double)pt1.x);
        double y_intercept = (double)pt1.y - slope*(double)pt1.x;

        // if this slope is negative, this means this is a left lane line. 
        // if so, add it to its corresponding vector, as well as its y
        // intercept.
        if(slope < 0.0){
            slopes_left.push_back(slope);
            y_intercepts_left.push_back(y_intercept);
        }

        // if this slope is positive, this means this is a right lane line. 
        // if so, add it to its corresponding vector, as well as its y
        // intercept
        else{
            slopes_right.push_back(slope);
            y_intercepts_right.push_back(y_intercept);
        }

    }

    // find the average of all left line slopes an y intercepts
    float avg_slope_left = accumulate(
        slopes_left.begin(), 
        slopes_left.end(), 
        0.0)/slopes_left.size();
    float avg_y_intercept_left = accumulate(
        y_intercepts_left.begin(), 
        y_intercepts_left.end(), 
        0.0)/y_intercepts_left.size();

    // find the average of all right line slopes an y intercepts
    float avg_slope_right = accumulate(
        slopes_right.begin(), 
        slopes_right.end(), 
        0.0)/slopes_right.size();
    float avg_y_intercept_right = accumulate(
        y_intercepts_right.begin(), 
        y_intercepts_right.end(), 
        0.0)/y_intercepts_right.size();
  
    // find the two point line interpretations of the average slope 
    // and y intercepts
    calculate_lines(edges, avg_slope_left, avg_y_intercept_left, left_line);
    calculate_lines(edges, avg_slope_right, avg_y_intercept_right, right_line);

}

int main(int argc, char** argv) {

    // Open lane video for processing
    VideoCapture cap("lane_video.mp4"); 
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while(1){

        // capture a video frame
        Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        // keep an original frame for comparison. OPTIONAL
        Mat original = frame.clone();

        // find the edges within the frame
        Mat edges = FindEdges(frame); 

        // select a region of interest within the frame and crop 
        // the frame accordingly.
        Mat mask = ROIMask(edges);
        CropROI(edges, mask);

        // given the cropped frame, find the lane lines.
        // lines are represented as a 4 element vector, where
        // the first two element represent the x & y points of
        // the first point, and the next two elements represent
        // the second x & y points of the second point.
        Vec4i right_line;
        Vec4i left_line;
        FindLines(edges, left_line, right_line);

        // display calculated lane lines onto the original frame
        line( original, {left_line[0], left_line[1]}, {left_line[2], left_line[3]}, Scalar(0,0,255), 10, CV_AA);
        line( original, {right_line[0], right_line[1]}, {right_line[2], right_line[3]}, Scalar(0,0,255), 10, CV_AA);             
        imshow( "Lanes", original ); 

        char c=(char)waitKey(25);
        if(c==27)
            break;
    }

    

    return 0;
}