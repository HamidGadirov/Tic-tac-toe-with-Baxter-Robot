#include "vision.h"

using Contour = std::vector<cv::Point>;

cv::Mat selectColor(cv::Mat rgb_img, int selector) {
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
        cv::inRange(hsv_image, cv::Scalar(0, 100, 50),
                    cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 50),
                    cv::Scalar(179, 255, 255), upper_red_hue_range);
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0,
                        binary_img);
        break;
    }

    return binary_img;
}

Contour findBiggestContourArea(std::vector<Contour> contours) {
    return *std::max_element(contours.begin(), contours.end(),
                             [](Contour const &lhs, Contour const &rhs) {
                                 return contourArea(lhs, false) <
                                        contourArea(rhs, false);
                             });
}

Contour findBiggestTetragon(cv::Mat binary_img) {
    std::vector<Contour> contours;
    cv::findContours(binary_img.clone(), contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    Contour approx;
    if (contours.size() > 0) {
        Contour c = findBiggestContourArea(contours);

        cv::approxPolyDP(c, approx, cv::arcLength(c, true) * 0.05, true);
    }

    return approx;
}


arr bilinearCenterInterpolation(std::vector<cv::Point> cornerPointsRed){
    if(cornerPointsRed.size() != 4){
        return {};
    }

    cv::Point topLeft = cornerPointsRed[1];
    cv::Point topRight = cornerPointsRed[0];
    cv::Point bottomRight = cornerPointsRed[3];
    cv::Point bottomLeft = cornerPointsRed[2];

    double x12 = 0.5*topLeft.x + 0.5*bottomLeft.x;
    double x34 = 0.5*topRight.x + 0.5*bottomRight.x;
    double x = 0.5*x12 + 0.5*x34;

    double y12 = 0.5*topLeft.y + 0.5*bottomLeft.y;
    double y34 = 0.5*topRight.y + 0.5*bottomRight.y;
    double y = 0.5*y12 + 0.5*y34;
    return {x,y};
}


const char *node_name = "cameraRosNodeTicTacToe";
const char *rgb_topic = "/camera/rgb/image_raw";
const char *depth_topic = "/camera/depth_registered/image_raw";

Vision::Vision()
    : Thread("VisionThread", -1.),
      cam1(_rgb, _depth, node_name, rgb_topic, depth_topic) {}

void Vision::open() {
    cout << "Opening Vision Thread" << endl;

    // Chest
    Pinv = arr(3, 4,
               {0.00180045, 5.51994e-06, -0.569533, -0.0330757, 
               -1.82321e-06,-0.00133149, 1.00136, 0.125005, 
               5.08217e-05, -0.00117336,-0.439092, 1.55487});
}

void Vision::step() {
    // wait for the next rgb image
    _rgb.waitForNextRevision();



    rgb = CV(_rgb.get());
    cv::Mat depth = CV(_depth.get());

    if (rgb.total() > 0 && depth.total() > 0) {
        cv::Mat input_img = rgb.clone();

        cv::imshow("rgb", input_img); // white=2meters
            

        // binary images
        cv::Mat binary_img1 = selectColor(rgb, 1); // board - green
        cv::Mat binary_img2 = selectColor(rgb, 2);  // red
        cv::Mat binary_img0 = selectColor(rgb, 0); // blue

        cv::Mat binary_img;
        cv::bitwise_or(binary_img1, binary_img2, binary_img);
        cv::bitwise_or(binary_img, binary_img0, binary_img);

        // cv::Mat img_final = b;
        cv::dilate(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), 2, 1,
                   1);
        cv::erode(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), 6, 1,
                  1);
        cv::dilate(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), 4, 1,
                   1);
        

        Contour cornerPoints = findBiggestTetragon(binary_img);

        cv::imshow("Binary", binary_img);
        if (cornerPoints.size() != 4) {
            cv::waitKey(1);
            return;
        }

        cv::Mat T = boardTransformation.transform(cornerPoints);
        cv::Rect brect = boardTransformation.getBrect();
        //T = squareTransformation(cornerPoints, input_img);

        cv::Mat input_transformed = input_img.clone();
        cv::warpPerspective(input_img, input_transformed, T,
                            input_transformed.size());

        // final squared board
        cv::Mat region = input_transformed(brect);

        // Get transformed depth image
        cv::Mat depth_transformed = depth.clone();
        cv::warpPerspective(depth, depth_transformed, T,
                            depth_transformed.size());

        // cut transformed depth image
        cv::Mat depth_region = depth_transformed(brect);

        arr deptharr = calcCornerDepth(depth_region);

        arr corner_coords(4, 3,
            {(double)cornerPoints[1].x, (double)cornerPoints[1].y,
            deptharr(0), (double)cornerPoints[0].x,
            (double)cornerPoints[0].y, deptharr(1),
            (double)cornerPoints[3].x, (double)cornerPoints[3].y,
            deptharr(2), (double)cornerPoints[2].x,
            (double)cornerPoints[2].y, deptharr(3)});

        for (int i = 0; i < 4; i++) {
            arr buffer = corner_coords[i];
            arr buf2 = {buffer(0) * buffer(2), buffer(1) * buffer(2), buffer(2),
                        1};
            buffer = Pinv * buf2;
        }

        board.updateCorners(corner_coords);

        
        // Calculating the token position!
        cv::Mat masked_input_img = input_img.clone();

        // Masking the region of the board (we shouldn't detect the tokens on the board).
        cv::fillConvexPoly(masked_input_img, cornerPoints.data(), 4,
                           cv::Scalar(0, 0, 0));
        //cv::imshow("test:", masked_input_img);
        cv::Mat binary_tkn = selectColor(masked_input_img, 2); // red

        cv::erode(binary_tkn, binary_tkn, cv::Mat(), cv::Point(-1, -1), 2, 1,
                  1);
        cv::dilate(binary_tkn, binary_tkn, cv::Mat(), cv::Point(-1, -1), 1, 1,
                   1);

        cv::imshow("Binarytoken:",binary_tkn);
        std::vector<cv::Point> cornerPointsRed =
            findBiggestTetragon(binary_tkn);

        cv::Mat region2;
        //cout << "Token corner size: " << cornerPointsRed.size() << endl;
        if (cornerPointsRed.size() == 4) {
            
            cv::Mat Tred = tokenTransformation.transform(cornerPointsRed);
            //cv::Mat Tred = squareTransformation(cornerPointsRed, depth);
            cv::Rect tokenBrect = tokenTransformation.getBrect();
            cv::Mat transformed_red = depth.clone();
            cv::warpPerspective(depth.clone(), transformed_red, Tred,
                                transformed_red.size());

            region2 = transformed_red(tokenBrect);

            float tokendepth = medianFilter(region2);

            arr centerpoint = bilinearCenterInterpolation(cornerPointsRed);

            double x_token = centerpoint(0);
            double y_token = centerpoint(1);

            arr buf2 = {x_token * tokendepth, y_token * tokendepth, tokendepth,
                        1};

            arr buffer = Pinv * buf2;
            token.center = buffer;

            arr deptharr = calcCornerDepth(region2);

            arr corner_coords_red(4, 3,
                {(double)cornerPointsRed[1].x, (double)cornerPointsRed[1].y,
                deptharr(0), (double)cornerPointsRed[0].x,
                (double)cornerPointsRed[0].y, deptharr(1),
                (double)cornerPointsRed[3].x, (double)cornerPointsRed[3].y,
                deptharr(2), (double)cornerPointsRed[2].x,
                (double)cornerPointsRed[2].y, deptharr(3)});

            for (int i = 0; i < 4; i++) {
                arr buffer = corner_coords_red[i];
                arr buf2 = {buffer(0) * buffer(2), buffer(1) * buffer(2), buffer(2),
                            1};
                buffer = Pinv * buf2;
            }
            //cout << "we do not go here" << corner_coords_red<< endl;
            token.updateCorners(corner_coords_red);
            token.valid = 1.;
        }
        else{
            token.valid = 0.;
        }

        cv::imshow("out:", region);
        cv::imshow("out:", depth*0.5);
        cv::waitKey(1);
    }
}

//Calculate the depth on the corners of a squared depth_img
arr Vision::calcCornerDepth(cv::Mat depth_img){
    //Attention: Only squared images are working!
    /*
    - tl: topleft
    - tr: topright
    - br: bottomright
    - bl: bottomleft
     */

    int detectionSize = 0.2 * depth_img.cols;
    cv::Mat tl_region = 
        depth_img(cv::Rect( 0, 
                            0, 
                            detectionSize, 
                            detectionSize));

    cv::Mat tr_region = 
        depth_img(cv::Rect( depth_img.cols - detectionSize, 
                            0,
                            detectionSize, 
                            detectionSize));

    cv::Mat br_region = 
        depth_img(cv::Rect( depth_img.cols - detectionSize, 
                            depth_img.cols - detectionSize,
                            detectionSize, 
                            detectionSize));

    cv::Mat bl_region = 
        depth_img(cv::Rect( 0, 
                            depth_img.cols - detectionSize, 
                            detectionSize, 
                            detectionSize));

    double tl_depth = medianFilter(tl_region);
    double tr_depth = medianFilter(tr_region);
    double br_depth = medianFilter(br_region);
    double bl_depth = medianFilter(bl_region);

    double buf;
    buf = (tl_depth - br_depth)/8. + tl_depth;
    br_depth = (br_depth - tl_depth)/8. + br_depth;
    tl_depth = buf;

    buf = (tr_depth - bl_depth)/8. + tr_depth;
    bl_depth = (bl_depth - tr_depth)/8. + bl_depth;
    tr_depth = buf;

    return {tl_depth,tr_depth,br_depth,bl_depth};
}


void Vision::close() {
    cout << "Closing Vision Thread" << endl;
}

Board Vision::getBoard() {
    return board;
}

Token Vision::getToken() {
    return token;
}

float Vision::medianFilter(cv::Mat img) {
    cv::Mat depth_img = img.clone();
    if(depth_img.cols == 0 || depth_img.rows == 0){
        return -1.;
    }

    cv::Mat mask = cv::Mat(depth_img == depth_img);
    double minVal, maxVal;
    cv::minMaxLoc(depth_img, &minVal, &maxVal);

    int histSize = 500;

    depth_img = depth_img * ((double)histSize) / maxVal;

    double median_index = (mask.rows * mask.cols) / 2;
    int bin = 0;
    double med = -1.0;

    float range[] = {0, (float)histSize};
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

    med = med * maxVal / ((double)histSize);

    return med;
}

Cell Vision::detectTkn(int cellnum) {
    cv::Mat input_transformed = rgb.clone();
    cv::Mat T = boardTransformation.getT();
    cv::warpPerspective(rgb, input_transformed, T, input_transformed.size());

    // final squared board
    cv::Rect brect = boardTransformation.getBrect();
    cv::Mat region = input_transformed(brect);

    if (region.cols < 1) {
        return Cell::Invalid;
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
    cv::Mat mask = region(cv::Rect(x_pos - x_third / 4, y_pos - x_third / 4,
                                   x_third * 2 / 4, y_third * 2 / 4));

    for (int i = 0; i < 3; i++) { // blue 0, green 1, red 2
        cv::Mat binary_img = selectColor(mask, i);
        float med = medianFilter(binary_img);
        if (med > 1) {
            switch (i) {
            case 0:
                return Cell::Player;
            case 1:
                return Cell::Empty;
            case 2:
                return Cell::Robot;
            }
        }
    }

    return Cell::Invalid;
}

Grid Vision::detectTokens() {
    Grid g;

    for (uint i = 0; i < g.size(); i++) {
        g[i] = detectTkn(i);
    }

    return g;
}


std::vector<std::array<int,9>> getGridOrientations(){
    std::vector<std::array<int,9>> ret;
    ret.push_back({0,1,2,3,4,5,6,7,8});
    ret.push_back({6,3,0,7,4,1,8,5,2});
    ret.push_back({8,7,6,5,4,3,2,1,0});
    ret.push_back({2,5,8,1,4,7,0,3,6});
    return ret;

}


bool Vision::detectNextMove(Grid grid) {
    



    cv::Mat T = boardTransformation.getT();
    cv::Rect brect = boardTransformation.getBrect();

    cv::Mat input_transformed = rgb.clone();
    cv::warpPerspective(rgb, input_transformed, T, input_transformed.size());

    cv::Mat initial_img = input_transformed(brect);
    if (initial_img.cols < 1) {
        return false;
    }
    int cnt = 0;

    while (true) { // update new_img each second
        rai::wait(0.1);
        cv::Mat input_transformed = rgb.clone();
        cv::warpPerspective(rgb, input_transformed, T,
                            input_transformed.size());

        cv::Mat new_img = input_transformed(brect);

        if (new_img.cols < 1) {
            continue;
        }

        cv::Mat diff;
        absdiff(initial_img, new_img, diff); // compare difference
        // imshow("diff", diff);

        int img_size = diff.rows * diff.cols;
        // cout << "img_size: " << img_size << endl;

        cv::Mat diff_gray;
        cvtColor(diff, diff_gray, cv::COLOR_BGR2GRAY); // diff_gray

        threshold(diff_gray, diff_gray, 10, 255, CV_THRESH_BINARY);
        //imshow("diff_gray", diff_gray);

        int nonzero_diff = countNonZero(diff_gray);
        // cout << "nonzero pixels: " << nonzero_diff << endl;
        double percentage = (double)nonzero_diff * 100 / (double)img_size;
        cout << "diff perc: " << percentage << endl;

        if (percentage < 5.) {
            cnt++;
        } else {
            cnt = 0;
        }
        if (cnt > 25) {
            cout << "no movement" << endl;
            Grid grid_buf = detectTokens();
            
            auto buff = getGridOrientations();
            int min_change = 100;
            for (std::array<int,9> arr : buff ){
                int changes = 0;
                for (int i = 0; i < 9; i++) {
                    if (grid_buf[arr[i]] == Cell::Invalid) {
                        cnt = 0;
                        break;
                    }
                    if (grid_buf[arr[i]] != grid[i]) {
                        changes++;
                    }
                }
                if (min_change > changes) {
                    min_change = changes;
                }
            }
            if(min_change >= 1){
                cout << "min_changes: " << min_change << endl;
                return true;
            }

        }
        initial_img = new_img;
    }

    return false;
}
