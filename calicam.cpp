
// This is NWNC (No Warranty No Copyright) Software.
// astar.ai
// Nov 16, 2018

#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>

////////////////////////////////////////////////////////////////////////////////

bool      live = false;
//bool      live = true;
//To run live mode, you need a CaliCam from www.astar.ai

int       vfov_bar =  0, width_bar =   0, height_bar =   0;
int       vfov_max = 60, width_max = 480, height_max = 360;
int       vfov_now = 60, width_now = 480, height_now = 360;

int       ndisp_bar =  1, wsize_bar = 2;
int       ndisp_max =  2, wsize_max = 4;
int       ndisp_now = 32, wsize_now = 7;

int       cap_cols, cap_rows, img_width;
bool      changed = false;
bool      is_sgbm = true;
cv::Mat   Translation, Kl, Kr, Dl, Dr, xil, xir, Rl, Rr, smap[2][2];

std::string cam_model;

////////////////////////////////////////////////////////////////////////////////

void OnTrackAngle(int, void*) {
  vfov_now = 60 + vfov_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackWidth(int, void*) {
  width_now = 480 + width_bar;
  if (width_now % 2 == 1)
    width_now--;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackHeight(int, void*) {
  height_now = 360 + height_bar;
  if (height_now % 2 == 1)
    height_now--;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackNdisp(int, void*) {
  ndisp_now = 16 + 16 * ndisp_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackWsize(int, void*) {
  wsize_now = 3 + 2 * wsize_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void LoadParameters(std::string file_name) {
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to open ini parameters" << std::endl;
    exit(-1);
  }

  cv::Size cap_size;
  fs["cam_model"] >> cam_model;
  fs["cap_size" ] >> cap_size;
  fs["Kl"       ] >> Kl;
  fs["Dl"       ] >> Dl;
  fs["xil"      ] >> xil;
  if (cam_model == "Stereo") {
    fs["Rl"       ] >> Rl;
    fs["Kr"       ] >> Kr;
    fs["Dr"       ] >> Dr;
    fs["xir"      ] >> xir;
    fs["Rr"       ] >> Rr;
    fs["T"        ] >> Translation;
  }
  fs.release();

  img_width = cap_size.width;
  cap_cols  = cap_size.width;
  cap_rows  = cap_size.height;

  if (cam_model == "Stereo")
    img_width  = cap_size.width / 2;
}

////////////////////////////////////////////////////////////////////////////////

void InitRectifyMap() {
  double vfov_rad = vfov_now * CV_PI / 180.;
  double focal = height_now / 2. / tan(vfov_rad / 2.);
  cv::Mat Knew =
      (cv::Mat_<double>(3, 3) << focal, 0., width_now  / 2. - 0.5,
                                 0., focal, height_now / 2. - 0.5,
                                 0., 0., 1.);

  int flags = cv::omnidir::RECTIFY_PERSPECTIVE;
  cv::Size img_size(width_now, height_now);

  cv::omnidir::initUndistortRectifyMap(Kl, Dl, xil, Rl, Knew, img_size,
                                   CV_16SC2, smap[0][0], smap[0][1], flags);

  std::cout << "Width: "  << width_now  << "\t"
            << "Height: " << height_now << "\t"
            << "V.Fov: "  << vfov_now   << "\n";
  std::cout << "K Matrix: \n" << Knew << std::endl;

  if (cam_model == "Stereo") {
    cv::omnidir::initUndistortRectifyMap(Kr, Dr, xir, Rr, Knew, img_size,
                                     CV_16SC2, smap[1][0], smap[1][1], flags);
    std::cout << "Ndisp: " << ndisp_now << "\t"
              << "Wsize: " << wsize_now << "\n";
  }
  std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void DisparityImage(const cv::Mat& recl, const cv::Mat& recr, cv::Mat& disp) {
  cv::Mat disp16s;
  int N = ndisp_now, W = wsize_now, C = recl.channels();
  if (is_sgbm) {
    cv::Ptr<cv::StereoSGBM> sgbm =
        cv::StereoSGBM::create(0, N, W, 8 * C * W * W, 32 * C * W * W);
    sgbm->compute(recl, recr, disp16s);
  } else {
    cv::Mat grayl, grayr;
    cv::cvtColor(recl, grayl, CV_BGR2GRAY);
    cv::cvtColor(recr, grayr, CV_BGR2GRAY);

    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(N, W);
    sbm->setPreFilterCap(31);
    sbm->setMinDisparity(0);
    sbm->setTextureThreshold(10);
    sbm->setUniquenessRatio(15);
    sbm->setSpeckleWindowSize(100);
    sbm->setSpeckleRange(32);
    sbm->setDisp12MaxDiff(1);
    sbm->compute(grayl, grayr, disp16s);
  }

  double minVal, maxVal;
  minMaxLoc(disp16s, &minVal, &maxVal);
  disp16s.convertTo(disp, CV_8UC1, 255 / (maxVal - minVal));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  std::string file_name = argc == 2 ? argv[1] : "./astar_calicam.yml";
  LoadParameters(file_name);
  InitRectifyMap();

  cv::Mat raw_img;
  cv::VideoCapture vcapture;
  if (live) {
    vcapture.open(0);

    if (!vcapture.isOpened()) {
      std::cout << "Camera doesn't work" << std::endl;
      exit(-1);
    }

    vcapture.set(CV_CAP_PROP_FRAME_WIDTH,  cap_cols);
    vcapture.set(CV_CAP_PROP_FRAME_HEIGHT, cap_rows);
    vcapture.set(CV_CAP_PROP_FPS, 30);
  } else {
    raw_img = cv::imread("dasl_wood_shop.jpg", cv::IMREAD_COLOR);
  }

  char win_name[256];
  sprintf(win_name, "Raw Image: %d x %d", img_width, cap_rows);
  std::string param_win_name(win_name);
  cv::namedWindow(param_win_name);

  cv::createTrackbar("V. FoV:  60    +", param_win_name,
                     &vfov_bar,   vfov_max,   OnTrackAngle);
  cv::createTrackbar("Width:  480 +", param_win_name,
                     &width_bar,  width_max,  OnTrackWidth);
  cv::createTrackbar("Height: 360 +", param_win_name,
                     &height_bar, height_max, OnTrackHeight);

  std::string disp_win_name  = "Disparity Image";
  if (cam_model == "Stereo") {
    cv::namedWindow(disp_win_name);
    cv::createTrackbar("Num Disp:  16 + 16 *", disp_win_name,
                       &ndisp_bar,  ndisp_max,   OnTrackNdisp);
    cv::createTrackbar("Blk   Size :     3  +  2 *", disp_win_name,
                       &wsize_bar,  wsize_max,  OnTrackWsize);
  }

  cv::Mat raw_imgl, raw_imgr, rect_imgl, rect_imgr;
  while (1) {
    if (changed) {
      InitRectifyMap();
      changed = false;
    }

    if (live)
      vcapture >> raw_img;

    if (raw_img.total() == 0) {
      std::cout << "Image capture error" << std::endl;
      exit(-1);
    }

    if (cam_model == "Stereo") {
      raw_img(cv::Rect(        0, 0, img_width, cap_rows)).copyTo(raw_imgl);
      raw_img(cv::Rect(img_width, 0, img_width, cap_rows)).copyTo(raw_imgr);

      cv::remap(raw_imgl, rect_imgl, smap[0][0], smap[0][1], 1, 0);
      cv::remap(raw_imgr, rect_imgr, smap[1][0], smap[1][1], 1, 0);
    } else {
      raw_imgl = raw_img;
      cv::remap(raw_img, rect_imgl, smap[0][0], smap[0][1], 1, 0);
    }

    cv::Mat small_img;
    cv::resize(raw_imgl, small_img, cv::Size(), 0.5, 0.5);
    imshow(param_win_name, small_img);
    imshow("Rectified Image", rect_imgl);

    if (cam_model == "Stereo") {
      cv::Mat disp_img;
      DisparityImage(rect_imgl, rect_imgr, disp_img);
      imshow(disp_win_name,  disp_img);
    }

    char key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27)
      break;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////


