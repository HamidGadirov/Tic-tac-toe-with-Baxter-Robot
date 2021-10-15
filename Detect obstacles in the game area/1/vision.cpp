

#include "vision.h"


Vision::Vision():Thread("VisionThread", -1.),K(KinModel::instance().K){
    
}


void Vision::open(){
    cout << "Opening Vision Thread" << endl;
    

    /*RosCamera cam1(_rgb, _depth, "cameraRosNodeTicTacToe",
                  "/camera/rgb/image_raw",
                  "/camera/depth_registered/image_raw");

    cam = &cam1;*/
    //  cameraFrame->setPosition({-0.0472772, 0.226517, 1.79207});
//  cameraFrame->setQuaternion({0.969594, 0.24362, -0.00590741, 0.0223832});
//  arr Fxypxy = {538.273, 544.277, 307.502, 249.954};
//  Fxypxy /= 0.982094;

    // set the intrinsic camera parameters
    double f = 1. / tan(0.5 * 60.8 * RAI_PI / 180.);
    f *= 320.;
    //Fxypxy = {f, f, 320., 240.};
    Fxypxy = {538.273, 544.277, 307.502, 249.954};
    Fxypxy /= 0.982094;

    // convert to point cloud (in camera frame, used only for display)
    Depth2PointCloud d2p2(_depth, Fxypxy);
    d2p = &d2p2;

    cameraFrame = K.addFrame("pclframe", "head");
    cameraFrame->setPosition({-0.0472772, 0.226517, 1.79207});
    cameraFrame->Q.setText(
        "d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
    /*cameraFrame->setQuaternion({0.969594, 0.24362, -0.00590741, 0.0223832});
    cameraFrame->calc_X_from_parent();*/


    cameraFrame->setPosition({-0.0472772, 0.226517, 1.79207});
    cameraFrame->setQuaternion({0.969594, 0.24362, -0.00590741, 0.0223832});
    Fxypxy = {538.273, 544.277, 307.502, 249.954};
    Fxypxy /= 0.982094;

    ball = K.addFrame("ball");
    ball->setShape(rai::ST_sphere, {.01});

    playboard = K.addFrame("playboard");
}

void Vision::step(){
    RosCamera cam1(_rgb, _depth, "cameraRosNodeTicTacToe",
                "/camera/rgb/image_raw",
                "/camera/depth_registered/image_raw");
    Depth2PointCloud d2p2(_depth, Fxypxy);

    int iterator = 0;

    while(true){
        // wait for the next rgb image
        _rgb.waitForNextRevision();
        // display the point cloud by setting it as cameraFrame shape
        if (d2p2.points.get()->N > 0) {
            K.gl().dataLock.lock(RAI_HERE);
            cameraFrame->setPointCloud(d2p2.points.get(), _rgb.get());
            K.gl().dataLock.unlock();
            K.watch(false);
        }


        {
            cv::Mat rgb = CV(_rgb.get());
            cv::Mat depth = CV(_depth.get());

            if (rgb.total() > 0 && depth.total() > 0) {
                cv::Mat input_img = rgb.clone();
                cv::imshow("rgb", input_img); // white=2meters

                // binary images
                cv::Mat binary_img1 = selectColor(rgb, 1); // board - green
                cv::Mat binary_img2 = selectColor(rgb, 2); // red
                cv::Mat binary_img0 = selectColor(rgb, 0); // blue

                cv::Mat binary_img;
                cv::bitwise_or(binary_img1, binary_img2, binary_img);
                cv::bitwise_or(binary_img, binary_img0, binary_img);


                // cv::Mat img_final = b;
                cv::erode(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1),
                          2, 1, 1);
                cv::dilate(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1),
                           2, 1, 1);

                std::vector<cv::Point> cornerPoints =
                    findBiggestTetragon(binary_img);

                cv::imshow("Binary", binary_img);

                if (cornerPoints.size() != 4) {
                    cv::waitKey(1);
                    continue;
                }


                cv::Mat T = squareTransformation(cornerPoints, input_img);

                cv::Mat input_transformed = input_img.clone();
                cv::warpPerspective(input_img, input_transformed, T,
                                    input_transformed.size());

                cv::Rect brect = cv::boundingRect(cornerPoints);
                int size = std::max(brect.width, brect.height);

                // final squared board
                cv::Mat region =
                    input_transformed(cv::Rect(brect.x, brect.y, size, size));
                
                if (iterator == 0) {
                    board.region = region.clone();
                }

                // Get transformed depth image
                cv::Mat depth_transformed = depth.clone();
                cv::warpPerspective(depth, depth_transformed, T,
                                    depth_transformed.size());

                // cut transformed depth image
                cv::Mat depth_region =
                    depth_transformed(cv::Rect(brect.x, brect.y, size, size));
                float twenty = 0.05 * size;
                cv::Mat topleft =
                    depth_region(cv::Rect(0, 0, (int)twenty, (int)twenty));
                cv::Mat topright = depth_region(
                    cv::Rect(size - twenty, 0, (int)twenty, (int)twenty));
                cv::Mat bottomright = depth_region(cv::Rect(
                    size - twenty, size - twenty, (int)twenty, (int)twenty));
                cv::Mat bottomleft = depth_region(
                    cv::Rect(0, size - twenty, (int)twenty, (int)twenty));

                float topleft_depth = medianFilter(topleft);
                float topright_depth = medianFilter(topright);
                float bottomright_depth = medianFilter(bottomright);
                float bottomleft_depth = medianFilter(bottomleft);

                arr corner_coords(
                    4, 3,
                    {(double)cornerPoints[1].x, (double)cornerPoints[1].y,
                     topleft_depth, (double)cornerPoints[0].x,
                     (double)cornerPoints[0].y, topright_depth,
                     (double)cornerPoints[3].x, (double)cornerPoints[3].y,
                     bottomright_depth, (double)cornerPoints[2].x,
                     (double)cornerPoints[2].y, bottomleft_depth});

                std::vector<double> boardshape;



                for (int i = 0; i < 4; i++) {
                    arr buffer = corner_coords[i];
                    depthData2point(buffer, Fxypxy);
                    cameraFrame->X.applyOnPoint(buffer);
                }


                board.updateCorners(corner_coords);
                corner_coords.reshape(12);
                for (int i = 0; i < 12; i++) {
                    boardshape.push_back(corner_coords.elem(i));
                }

                playboard->setConvexMesh(boardshape, {0, 255, 0});

                cv::imshow("out:", region);
                cv::waitKey(1);

            }
        }

        iterator = (iterator + 1) % 100;
    }


}

void Vision::close(){
    cout << "Closing Vision Thread" << endl;
}

Board Vision::getBoard(){
    return board;
}



std::array<cv::Point, 4> detectCorners(const std::vector<cv::Point> &c) {
    int inf = std::numeric_limits<int>::max();
    std::array<cv::Point, 4> corners = {
        cv::Point{inf, inf}, // top left
        {-inf, inf},         // top right
        {-inf, -inf},        // bottom right
        {inf, -inf},         // bottom left
    };

    for (auto p : c) {
        if (p.x <= corners[0].x && p.y <= corners[0].y) {
            corners[0] = p;
        }
        if (p.x >= corners[1].x && p.y <= corners[1].y) {
            corners[1] = p;
        }
        if (p.x >= corners[2].x && p.y >= corners[2].y) {
            corners[2] = p;
        }
        if (p.x <= corners[3].x && p.y >= corners[3].y) {
            corners[3] = p;
        }
    }

    return corners;
}

std::vector<cv::Point>
Vision::findBiggestContourArea(std::vector<std::vector<cv::Point>> contours) {
    std::vector<cv::Point> c = *std::max_element(
        contours.begin(), contours.end(),
        [](std::vector<cv::Point> const &lhs,
           std::vector<cv::Point> const &rhs) {
            return contourArea(lhs, false) < contourArea(rhs, false);
        });
    return c;
}

std::vector<cv::Point> Vision::findBiggestTetragon(cv::Mat binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img.clone(), contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> approx;

    if (contours.size() > 0) {
        std::vector<cv::Point> c = findBiggestContourArea(contours);

        cv::approxPolyDP(c, approx, cv::arcLength(c, true) * 0.01, true);
    }
    return approx;
}

cv::Mat Vision::squareTransformation(std::vector<cv::Point> cornerPoints,
                                     cv::Mat input_img) {
    cv::Rect brect = cv::boundingRect(cornerPoints);
    int size = std::max(brect.width, brect.height);
    std::vector<cv::Point2f> corners = {
        cv::Point{brect.x, brect.y},
        {size + brect.x, brect.y},        // top right
        {size + brect.x, size + brect.y}, // bottom right
        {brect.x, size + brect.y},        // bottom left
    };

    std::vector<cv::Point2f> test = {
        cv::Point2f{cornerPoints[1].x, cornerPoints[1].y},
        {cornerPoints[0].x, cornerPoints[0].y}, // top right
        {cornerPoints[3].x, cornerPoints[3].y}, // bottom right
        {cornerPoints[2].x, cornerPoints[2].y}, // bottom left
    };

    return cv::getPerspectiveTransform(test, corners);
}

cv::Mat Vision::selectColor(cv::Mat rgb_img, int selector) {
    cv::Mat hsv_image;
    cv::cvtColor(rgb_img, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat binary_img;
    switch (selector) {
    case 0: // Blue
        cv::inRange(hsv_image, cv::Scalar(80, 100, 100),
                    cv::Scalar(120, 255, 255), binary_img);
        break;

    case 1: // Green
        cv::inRange(hsv_image, cv::Scalar(35, 100, 40),
                    cv::Scalar(80, 255, 255), binary_img);
        break;

    case 3: // Dark-Green
        cv::inRange(hsv_image, cv::Scalar(35, 0, 30), cv::Scalar(80, 255, 255),
                    binary_img);
        break;

    default: // Red
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100),
                    cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100),
                    cv::Scalar(179, 255, 255), upper_red_hue_range);
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0,
                        binary_img);
        break;
    }

    return binary_img;
}

float Vision::medianFilter(cv::Mat depth_img) {
    cv::Mat mask = cv::Mat(depth_img == depth_img);

    depth_img = depth_img * 128.;

    double median_index = (mask.rows * mask.cols) / 2;
    int bin = 0;
    double med = -1.0;

    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist;

    cv::calcHist(&depth_img, 1, 0, mask, hist, 1, &histSize, &histRange,
                 uniform, accumulate);

    for (int i = 0; i < histSize && med < 0.0; ++i) {
        bin += cvRound(hist.at<float>(i));
        if (bin > median_index && med < 0.0) {
            med = i;
            break;
        }
    }

    med = med / 128.;

    return med;
}

int Vision::detectToken (int cellnum) {

    if (board.region.cols<1){
        return -1;
    }

    // calculate cell centers
    int topRight = board.region.cols;
    int bottomLeft = board.region.rows;
    
    int x_vector = topRight;
    int y_vector = bottomLeft;

    int x_third = x_vector/3;
    int y_third = y_vector/3;

    int row = cellnum/3;
    int column = cellnum%3;
    int x_pos = column*x_third + x_third/2.;
    int y_pos = row*y_third + y_third/2.;
    
    cout<<"before mask"<<endl;
    // add mask
    cout<<"cols and rows: " << board.region.cols << " " << board.region.rows <<endl;
    cout<<"a "<<x_pos-x_third/4 << " " << x_third*2/4<<endl;
    cv::Mat mask = board.region(cv::Rect(x_pos-x_third/4, y_pos-x_third/4, x_third*2/4, y_third*2/4));
    cout<<"after mask"<<endl;
    for(int i=0;i<3;i++){ // blue 0, green 1, red 2
        cv::Mat binary_img = selectColor(mask, i);
        float med = medianFilter(binary_img);
        cout << "median " << med << endl;
        if(med>1){
            return i;
        }
    }

    return -1;
}

int Vision::detectNextMove() {

    RosCamera cam1(_rgb, _depth, "cameraRosNodeTicTacToe",
                "/camera/rgb/image_raw",
                "/camera/depth_registered/image_raw");
    Depth2PointCloud d2p2(_depth, Fxypxy);

    while(true){
        // wait for the next rgb image
        _rgb.waitForNextRevision();
        // display the point cloud by setting it as cameraFrame shape
        if (d2p2.points.get()->N > 0) {
            K.gl().dataLock.lock(RAI_HERE);
            cameraFrame->setPointCloud(d2p2.points.get(), _rgb.get());
            K.gl().dataLock.unlock();
            K.watch(false);
        }

        {
            cv::Mat rgb = CV(_rgb.get());
            cv::Mat depth = CV(_depth.get());

            if (rgb.total() > 0 && depth.total() > 0) {
                cv::Mat input_img = rgb.clone(); // background image
                cv::imshow("rgb", input_img); // white=2meters

                cv::Mat diff;

                while(true) { 
                    cv::Mat new_img = CV(_rgb.get()); // new image

                    absdiff(input_img, new_img, diff); // compare difference
                    imshow("diff", diff);
                    int size = diff.rows*diff.cols;
                    cout << size << endl;

                    cv::Mat gray;
                    cvtColor(diff, gray, cv::COLOR_BGR2GRAY); //GRAY
                    imshow("gray", gray);
                    //waitKey(0);

                    int nonzero_diff = countNonZero(gray);
                    cout << nonzero_diff << endl;
                    double percentage = (double)nonzero_diff*100/(double)size;
                    cout << percentage << endl;

                    if (percentage>1. && percentage<10.) {
                        return 1;
                    }

                }
            }
        }
    }

    return 0;

}
