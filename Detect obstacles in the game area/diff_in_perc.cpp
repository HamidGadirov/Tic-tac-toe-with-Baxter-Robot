#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

int main()
{
    cv::Mat img1, img2, diff;

    img1 = imread("grid.png");
    img2 = imread("grid1.png");

    absdiff(img1, img2, diff); // absolute difference
    imshow("diff", diff);
    int size = diff.rows*diff.cols;
    cout << size << endl;

    Mat gray;
    cvtColor(diff, gray, cv::COLOR_BGR2GRAY); //GRAY
    imshow("gray", gray);
    waitKey(0);

    int nonzero_diff = countNonZero(gray);
    cout << nonzero_diff << endl;
    double percentage = (double)nonzero_diff*100/(double)size;
    cout << percentage << endl;

    return 0;
}

/*
#--- convert the result to integer type ---
res = res.astype(np.uint8)

#--- find percentage difference based on number of pixels that are not zero ---
percentage = (numpy.count_nonzero(res) * 100)/ res.size
*/
