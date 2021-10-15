#ifndef RAI_OPENCV
    #define RAI_OPENCV 1
#endif
#ifndef RAI_ROS
    #define RAI_ROS 1
#endif
#include <Perception/opencv.h>
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>
#include <limits>



    std::vector<cv::Point> findBiggestContourArea(std::vector<std::vector<cv::Point> > contours){
        std::vector<cv::Point> c = *std::max_element(contours.begin(),
        contours.end(),
        [](std::vector<cv::Point> const& lhs, std::vector<cv::Point> const& rhs)
        {
            return contourArea(lhs, false) < contourArea(rhs, false);
        });
        return c;
    }


    cv::Mat selectColor(cv::Mat rgb_img, int selector){
        cv::Mat hsv_image;
        cv::cvtColor(rgb_img, hsv_image, cv::COLOR_BGR2HSV);
        cv::Mat binary_img;
        switch (selector)
        {
        case 0: //Blue
            cv::inRange(hsv_image, cv::Scalar(80, 100, 100), cv::Scalar(120, 255, 255), binary_img);
            break;

        case 1: //Green
            cv::inRange(hsv_image, cv::Scalar(35, 100, 100), cv::Scalar(80, 255, 255), binary_img);
            break;

        case 2: //Dark-Green
            cv::inRange(hsv_image, cv::Scalar(35, 0, 30), cv::Scalar(80, 255, 255), binary_img);
            break;
        
        default: //Red
            cv::Mat lower_red_hue_range;
            cv::Mat upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, binary_img);
            break;
        }

        return binary_img;
    }


cv::Mat crop_minAreaRect(cv::Mat img,cv::RotatedRect rect){

    float angle = rect.angle + 90.;
    int rows = img.rows;
    int cols = img.cols;


    cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(cols/2,rows/2),angle,1);
    cv::Mat rot;
    cv::warpAffine(img,rot,M,cv::Size(cols,rows));

    cv::RotatedRect rect0(rect.center,rect.size,0.);
    //cv::Box

  /*

    # rotate bounding box
    rect0 = (rect[0], rect[1], 0.0) 
    box = cv2.boxPoints(rect0)
    pts = np.int0(cv2.transform(np.array([box]), M))[0]    
    pts[pts < 0] = 0

    # crop
    img_crop = img_rot[pts[1][1]:pts[0][1], 
                       pts[1][0]:pts[2][0]]*/

    return rot;

}


std::array<cv::Point, 4> detectCorners(const std::vector<cv::Point>& c){
  int inf = std::numeric_limits<int>::max();
  std::array<cv::Point, 4> corners = {
    cv::Point{inf,inf},   //top left
    {-inf,inf},           //top right
    {-inf,-inf},          //bottom right
    {inf,-inf},           //bottom left
  };  

  for(auto p : c){
    if(p.x <= corners[0].x && p.y <= corners[0].y){
      corners[0] = p;
    }
    if(p.x >= corners[1].x && p.y <= corners[1].y){
      corners[1] = p;
    }
    if(p.x >= corners[2].x && p.y >= corners[2].y){
      corners[2] = p;
    }
    if(p.x <= corners[3].x && p.y >= corners[3].y){
      corners[3] = p;
    }
  }

  return corners;
}







void basicCV(){
    //subscribe to image and depth
    Var<byteA> _rgb;
    Var<floatA> _depth;
    RosCamera cam(_rgb, _depth, "cameraRosNodeTicTacToe", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

    //set the intrinsic camera parameters
    double f = 1./tan(0.5*60.8*RAI_PI/180.);
    f *= 320.;
    arr Fxypxy = {f, f, 320., 240.};

    //convert to point cloud (in camera frame, used only for display)
    Depth2PointCloud d2p(_depth, Fxypxy);

    //interface to baxter to query it's real pose
    BaxterInterface B(true);

    //load a configuration
    rai::KinematicWorld C;
    C.addFile("../rai-robotModels/baxter/baxter_new.g");

    //add a frame for the camera
    rai::Frame *cameraFrame = C.addFrame("camera", "head");
    cameraFrame->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
    cameraFrame->calc_X_from_parent();

    for(uint i=0;i<10000;i++){
      //wait for the next rgb image
      _rgb.waitForNextRevision();

      //sync C with the real baxter pose
      arr q_real = B.get_q();
      if(q_real.N==C.getJointStateDimension())
        C.setJointState(q_real);

      //display the point cloud by setting it as cameraFrame shape
      if(d2p.points.get()->N>0){
        C.gl().dataLock.lock(RAI_HERE); //(in case you interact with the GUI while the drawing data changes)
        cameraFrame->setPointCloud(d2p.points.get(), _rgb.get());
        C.gl().dataLock.unlock();
        int key = C.watch(false);
        if(key=='q') break;
      }

      //display the images
      {
        cv::Mat rgb = CV(_rgb.get());
        cv::Mat depth = CV(_depth.get());

        if(rgb.total()>0 && depth.total()>0){

          cv::Mat binary_img = selectColor(rgb,1);

          cv::Mat output = rgb.clone();


          //cv::Mat img_final = b;
          cv::erode(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
          cv::dilate(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

          std::vector<std::vector<cv::Point> > contours;
          cv::findContours(binary_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
          //cout<<"contours.size(): " << contours.size() << endl;
          cv::Mat M;

          cout << "Testout..." << endl;
          if(contours.size() > 0){
            std::vector<cv::Point> c = findBiggestContourArea(contours);

            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, cv::arcLength(c, true) * 0.01, true);

            //std::array<cv::Point,4> corners = detectCorners(c);


            /*for(cv::Point p: approx){
              cv::circle(output, p, 3, cv::Scalar(255, 0, 0), 3);
            }*/

            cout << "Testout..." << endl;
            if(approx.size() == 4){

            cv::circle(output, approx[0], 3, cv::Scalar(255, 0, 0), 3);
            cv::circle(output, approx[1], 3, cv::Scalar(0, 255, 0), 3);
            cv::circle(output, approx[2], 3, cv::Scalar(0, 0, 255), 3);
            cv::circle(output, approx[3], 3, cv::Scalar(255, 200, 255), 3);



              cv::Rect brect = cv::boundingRect(approx);
              cv::Mat region = output(brect);
              int size = std::max(brect.width,brect.height);
              std::vector<cv::Point2f> corners = {
                cv::Point{brect.x,brect.y},
                {size+brect.x,brect.y},           //top right
                {size+brect.x,size+brect.y},          //bottom right
                {brect.x,size+brect.y},           //bottom left
              }; 

              


              std::vector<cv::Point2f> test = {
                cv::Point2f{approx[1].x,approx[1].y},
                {approx[0].x,approx[0].y},           //top right
                {approx[3].x,approx[3].y},          //bottom right
                {approx[2].x,approx[2].y},           //bottom left
              }; 

              
              cout << "Testout..." << endl;
              cv::Mat T = cv::getPerspectiveTransform(test,corners);
              cout << "Corners..."<< corners << endl;
              cout << "Test..."<< test << endl;
              cv::Mat out = output.clone();
              cv::warpPerspective(output,out,T,out.size());

              region = out(cv::Rect(brect.x,brect.y,size,size));

              cv::imshow("out:",region);
            }

            // draw corners
            /*cv::circle(output, corners[0], 3, cv::Scalar(255, 0, 0), 3);
            cv::circle(output, corners[1], 3, cv::Scalar(255, 0, 0), 3);
            cv::circle(output, corners[2], 3, cv::Scalar(255, 0, 0), 3);
            cv::circle(output, corners[3], 3, cv::Scalar(255, 0, 0), 3);*/

            //cout << "contour:" << corners[0] << endl;





/*


            // Find rotated rectangle
            cv::RotatedRect board = cv::minAreaRect(c);

            // get bounding rectangle
            cv::Rect brect = board.boundingRect();

            // cut region
            cv::Mat region = output(brect);

            cv::imshow("region:",region);
            //cv::rectangle(output, brect, cv::Scalar(255,0,0));
            M = crop_minAreaRect(region,board);

            cv::imshow("test", M);*/

          }
/*
          std::vector<cv::Point> approx;
          for( unsigned int i = 0; i< contours.size(); i++ ){
            cv::Scalar color = cv::Scalar(0, 255, 0);
            std::vector<cv::Vec4i> hierarchy;
            
            //drawContours( output, contours, i, color, 3, 8, hierarchy, 0, cv::Point() );

            cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.01, true); //Approx a polygonal curve with less vertices, Douglas-Peucker algorithm
            cout<<approx.size()<<endl;

          }*/
          
          cv::imshow("Binary", binary_img);
          















        
          //cv::findContours(binary_img,)

          cv::imshow("rgb", output); //white=2meters
          cv::imshow("depth", 0.5*depth); //white=2meters
          int key = cv::waitKey(1);
          if((key&0xff)=='q') break;
        }
      }

      //example on how to convert image to 3D coordinates:
      double x_pixel_coordinate=0., y_pixel_coordinate=0., depth_from_depthcam=1.2;
      arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
      depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
      cameraFrame->X.applyOnPoint(pt); //transforms into world coordinates
    }

//  rai::wait();
}



int main(int argc,char **argv){
    rai::initCmdLine(argc,argv);
    
    basicCV();

    return 0;
}
