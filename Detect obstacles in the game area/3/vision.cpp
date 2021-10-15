#include "vision.h"

Vision::Vision() : Thread("VisionThread", -1.), K(KinModel::instance().K) {}

void Vision::open() {
    cout << "Opening Vision Thread" << endl;

    // set the intrinsic camera parameters
    double f = 1. / tan(0.5 * 60.8 * RAI_PI / 180.);
    f *= 320.;
    // Fxypxy = {f, f, 320., 240.};
    Fxypxy = {538.273, 544.277, 307.502, 249.954};
    Fxypxy /= 0.982094;

    // convert to point cloud (in camera frame, used only for display)
    Depth2PointCloud d2p2(_depth, Fxypxy);
    d2p = &d2p2;

    cameraFrame = K.addFrame("pclframe", "head");
    cameraFrame->Q.setText(
        "d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
    cameraFrame->calc_X_from_parent();
    cameraFrame->setPosition({-0.0499056, 0.231561, 1.7645});
    // Good Params:
    cameraFrame->setQuaternion({0.9708458, 0.2381062, -0.0252652, -0.0112});

    // Original Params:
    // cameraFrame->setQuaternion({0.971032, 0.237993, -0.00607315, 0.0204557});
    // Euler Angle: x: 177.557478, y: -0.1179045, z: -27.545153
    Fxypxy = {539.637, 540.941, 317.533, 260.024};

    ball = K.addFrame("ball");
    ball->setShape(rai::ST_sphere, {.01});

    playboard = K.addFrame("playboard");
}

void Vision::step() {
    Var<byteA> _rgb;
    RosCamera cam1(_rgb, _depth, "cameraRosNodeTicTacToe",
                   "/camera/rgb/image_raw",
                   "/camera/depth_registered/image_raw");
    Depth2PointCloud d2p2(_depth, Fxypxy);

    int iterator = 0;

    while (true) {
        
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
            rgb = CV(_rgb.get());
            cv::Mat depth = CV(_depth.get());

            if (rgb.total() > 0 && depth.total() > 0) {
                //cout << "Opening Vision Thread" << endl;
                cv::Mat input_img = rgb.clone();
                onlinetracking(input_img);

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

                board.setCornerImgPoints(cornerPoints);
                squareTransformation(cornerPoints, input_img);

                cv::Mat input_transformed = input_img.clone();
                cv::warpPerspective(input_img, input_transformed, T,
                                    input_transformed.size());

                // final squared board
                cv::Mat region = input_transformed(brect);

                if (iterator == 0) {
                    board.region = region.clone();
                }

                // Get transformed depth image
                cv::Mat depth_transformed = depth.clone();
                cv::warpPerspective(depth, depth_transformed, T,
                                    depth_transformed.size());

                // cut transformed depth image
                cv::Mat depth_region = depth_transformed(brect);
                int detectionSize = 0.05 * brect.width;
                cv::Mat topleft =
                    depth_region(cv::Rect(0, 0, detectionSize, detectionSize));
                cv::Mat topright = depth_region(
                    cv::Rect(brect.width - detectionSize, 0, detectionSize, detectionSize));
                cv::Mat bottomright = depth_region(cv::Rect(
                    brect.width - detectionSize, brect.width - detectionSize, detectionSize, detectionSize));
                cv::Mat bottomleft = depth_region(
                    cv::Rect(0, brect.width - detectionSize, detectionSize, detectionSize));

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

void Vision::close() {
    cout << "Closing Vision Thread" << endl;
}

Board Vision::getBoard() {
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
    this->brect= cv::Rect(brect.x, brect.y, size, size);

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

    T = cv::getPerspectiveTransform(test, corners);
    return T;
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
    case 4: // Dark Blue
        cv::inRange(hsv_image, cv::Scalar(80, 100, 30),
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

int Vision::detectTkn(int cellnum, Board brd) {
    cv::Mat input_transformed = rgb.clone();
    cv::warpPerspective(rgb, input_transformed, T, input_transformed.size());

    // final squared board
    cv::Mat region = input_transformed(brect);

    if (region.cols < 1) {
        return -1.;
    }

    // calculate cell centers
    int topRight = region.cols;
    int bottomLeft = region.rows;

    int x_vector = topRight;
    int y_vector = bottomLeft;

    int x_third = x_vector / 3;
    int y_third = y_vector / 3;

    int row = cellnum / 3;
    int column = cellnum % 3;
    int x_pos = column * x_third + x_third / 2.;
    int y_pos = row * y_third + y_third / 2.;

    // add mask
    cv::Mat mask =
        board.region(cv::Rect(x_pos - x_third / 4, y_pos - x_third / 4,
                              x_third * 2 / 4, y_third * 2 / 4));

    for (int i = 0; i < 3; i++) { // blue 0, green 1, red 2
        cv::Mat binary_img = selectColor(mask, i);
        float med = medianFilter(binary_img);
        if (med > 1) {
            return i;
        }
    }

    return -1;
}

int Vision::detectToken(int cellnum) {
    return detectTkn(cellnum, board);
}

void Vision::onlinetracking(cv::Mat input_img) {
    // save and set current joint state
    KinModel::instance().sync();

    // calc world coordinates of the ball
    arr y, J;
    K.evalFeature(y, J, FS_position, {"calibL"});

    // calc position and depth of the ball in the image
    arr pt = globalPointToCameraPoint(y);
    arr res1 = cameraPointToImgCoords(pt);
    arr res2 = cameraPointToImgCoords({pt(0) + .02, pt(1), pt(2)});

    double radius = sqrt(pow(res1(0) - res2(0), 2) + pow(res1(1) - res2(1), 2));
    Board circle(res1, radius);

    // Return if ball is not within the image
    if (res1(0) < 0 || res1(0) > input_img.cols || res1(1) < 0 ||
        res1(1) > input_img.rows || res1(2) <= 0) {
        return;
    }

    // Drawing output:
    cv::circle(input_img, cv::Point(res1(0), res1(1)), radius,
               cv::Scalar(0, 0, 255), 4);

    // find three nearest neighbours in the image
    std::vector<Board> ret;

    ret = findNNObjects(input_img, circle, 3);
}

std::vector<Board> Vision::findNNObjects(cv::Mat input_img, Board circle,
                                         int nn) {
    // cut-off distance in cm:
    double cutoff = 5;
    int x = std::max(
        0, (int)(circle.getCenter()(0) - (circle.getRadius() * cutoff / 2.)));
    int y = std::max(
        0, (int)(circle.getCenter()(1) - (circle.getRadius() * cutoff / 2.)));
    int x_len =
        std::min((int)(circle.getRadius() * cutoff), input_img.cols - x);
    int y_len =
        std::min((int)(circle.getRadius() * cutoff), input_img.rows - y);

    // cut input image for better performance
    cv::Mat region = input_img(cv::Rect(x, y, x_len, y_len));

    // color selection
    cv::Mat binary_img = selectColor(region, 4); // blue

    // cv::imshow("reg:",binary_img);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    cv::Point2f center(-1.0, -1.0);
    float radius = -1.0;

    if (contours.size() > 0) {
        std::vector<cv::Point> c;
        c = findBiggestContourArea(contours);
        cv::minEnclosingCircle(c, center, radius);
        cv::circle(region, center, radius, cv::Scalar(0, 255, 0), 3);
    } else {
        cout << "No contours found..." << endl;
    }

    cv::imshow("reg1:", binary_img);
    cv::imshow("reg:", region);

    std::vector<Board> ret;

    return ret;
}

arr Vision::cameraPointToImgCoords(arr pt) {
    pt(0) = pt(0) * Fxypxy(0) / -pt(2) + Fxypxy(2);
    pt(1) = pt(1) * Fxypxy(1) / pt(2) + Fxypxy(3);
    pt(2) = -pt(2);

    return pt;
}

arr Vision::globalPointToCameraPoint(arr pt) {
    arr T = K.getFrameByName("pclframe")->X.getInverseAffineMatrix();

    arr y_homo = {pt(0), pt(1), pt(2), 1.};

    arr y_cam = T * y_homo;

    pt(0) = y_cam(0) / y_cam(3);
    pt(1) = y_cam(1) / y_cam(3);
    pt(2) = y_cam(2) / y_cam(3);

    return pt;
}

arr Vision::detectTokens(Board brd) {
    arr placed = {-1., -1., -1., -1., -1., -1., -1., -1., -1.};
    for (int i = 0; i < 9; i++) {
        switch (detectTkn(i, brd)) {
        case 0: // Blue
            placed(i) = 2.;
            break;
        case 1: // Green
            placed(i) = 0.;
            break;
        case 2: // Red
            placed(i) = 1.;
            break;

        default:
            break;
        }
    }
    return placed;
}

bool Vision::detectNextMove() {
    cv::Mat T = this->T;
    cv::Rect brect = this->brect;

    cv::Mat input_transformed = rgb.clone();
    cv::warpPerspective(rgb, input_transformed, T, input_transformed.size());

    cv::Mat initial_img = input_transformed(brect);

    if (initial_img.cols < 1) {
        return false;
    }
    
    while(true) { // update new_img each second
        rai::wait(1.);
        cv::Mat input_transformed = rgb.clone();
        cv::warpPerspective(rgb, input_transformed, T, input_transformed.size());

        cv::Mat new_img = input_transformed(brect);

        if (new_img.cols < 1) {
            return false;
        }

        cv::Mat diff;
        absdiff(initial_img, new_img, diff); // compare difference
        imshow("diff", diff);
        int img_size = diff.rows*diff.cols;
        cout << "img_size: " << img_size << endl;

        cv::Mat diff_gray;
        cvtColor(diff, diff_gray, cv::COLOR_BGR2GRAY); //diff_gray

        threshold(diff_gray, diff_gray, 10, 255, CV_THRESH_BINARY);
        imshow("diff_gray", diff_gray);

        int nonzero_diff = countNonZero(diff_gray);
        cout << "nonzero pixels: " << nonzero_diff << endl;
        double percentage = (double)nonzero_diff*100/(double)img_size;
        cout << "diff perc: " << percentage << endl;

        if (5. < percentage && percentage < 15.) {
            rai::wait(3.);
            return true;
        }
    }
}
