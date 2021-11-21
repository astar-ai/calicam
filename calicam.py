
import sys
import cv2
import math
import numpy as np

#-------------------------------------------------------------------------------#

Kl = None
Dl = None
Rl = None
xil = None

Kr = None
Dr = None
Rr = None
xir = None

T = None

mapx = [None, None]
mapy = [None, None]

cam_model = None
cap_cols = None
cap_rows = None
img_width = None

vfov_bar =  0
width_bar = 0
height_bar = 0
vfov_max = 60
width_max = 480
height_max = 360
vfov_now = 60
width_now = 480
height_now = 360

ndisp_bar = 1
wsize_bar = 2
ndisp_max = 2
wsize_max = 4
ndisp_now = 32
wsize_now = 7

changed = False
is_sgbm = True

#-------------------------------------------------------------------------------#

def load_parameters(param_file):
  global Kl, Dl, Rl, xil, Kr, Dr, Rr, xir, T
  global cam_model, cap_cols, cap_rows, img_width
  
  fs = cv2.FileStorage(param_file, cv2.FILE_STORAGE_READ)
  
  cam_model = fs.getNode("cam_model").string()
  cap_size_node = fs.getNode("cap_size")
  cap_cols = int(cap_size_node.at(0).real())
  cap_rows = int(cap_size_node.at(1).real())
  img_width = cap_cols
  
  Kl = fs.getNode("Kl").mat()
  Dl = fs.getNode("Dl").mat()
  Rl = np.identity(3, dtype = np.float64)
  xil = fs.getNode("xil").mat()
  
  if cam_model == "stereo":
    Rl = fs.getNode("Rl").mat()

    Kr = fs.getNode("Kr").mat()
    Dr = fs.getNode("Dr").mat()
    Rr = fs.getNode("Rr").mat()
    xir = fs.getNode("xir").mat()
    
    T = fs.getNode("T").mat()
    
    img_width = cap_cols / 2
      
#-------------------------------------------------------------------------------#

def init_undistort_rectify_map(k, d, r, knew, xi0, size, mode):
  fx = k[0, 0]
  fy = k[1, 1]
  cx = k[0, 2]
  cy = k[1, 2]
  s  = k[0, 1]
  
  k1 = d[0, 0]
  k2 = d[0, 1]
  p1 = d[0, 2]
  p2 = d[0, 3]
  
  ki = np.linalg.inv(knew)
  ri = np.linalg.inv(r)
  kri = np.linalg.inv(np.matmul(knew, r))
  
  rows = size[0]
  cols = size[1]
  
  mapx = np.zeros((rows, cols), dtype = np.float32)
  mapy = np.zeros((rows, cols), dtype = np.float32)
  
  print("Wait, this takes a while ... ")
  for r in range(rows):
    for c in range(cols):
      xc = 0.0
      yc = 0.0
      zc = 0.0
      
      if mode == 'kRectPerspective':
        cr1 = np.array([c, r, 1.])
        xc = np.dot(kri[0, :], cr1)
        yc = np.dot(kri[1, :], cr1)
        zc = np.dot(kri[2, :], cr1)
  
      if mode == 'kRectLonglat':
        tt = (c * 1. / (cols - 1) - 0.5) * math.pi
        pp = (r * 1. / (rows - 1) - 0.5) * math.pi

        xn = math.sin(tt)
        yn = math.cos(tt) * math.sin(pp)
        zn = math.cos(tt) * math.cos(pp)
      
        cr1 = np.array([xn, yn, zn])
        xc = np.dot(ri[0, :], cr1)
        yc = np.dot(ri[1, :], cr1)
        zc = np.dot(ri[2, :], cr1)
  
      if mode == 'kRectFisheye':
        cr1 = np.array([c, r, 1.])
        ee = np.dot(ki[0, :], cr1)
        ff = np.dot(ki[1, :], cr1)
        zz = 2. / (ee * ee + ff * ff + 1.)

        xn = zz * ee
        yn = zz * ff
        zn = zz - 1.

        cr1 = np.array([xn, yn, zn])
        xc = np.dot(ri[0, :], cr1)
        yc = np.dot(ri[1, :], cr1)
        zc = np.dot(ri[2, :], cr1)
        
      if zc < 0.0:
        mapx[r, c] = np.float32(-1.)
        mapy[r, c] = np.float32(-1.)

        continue
      
      rr = math.sqrt(xc * xc + yc * yc + zc * zc)
      xs = xc / rr
      ys = yc / rr
      zs = zc / rr
      
      xu = xs / (zs + xi0)
      yu = ys / (zs + xi0)
      
      r2 = xu * xu + yu * yu
      r4 = r2 * r2
      xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu + p2 * (r2 + 2 * xu * xu)
      yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu + p1 * (r2 + 2 * yu * yu)

      u = fx * xd + s * yd + cx
      v = fy * yd + cy

      mapx[r, c] = np.float32(u)
      mapy[r, c] = np.float32(v)

  return mapx, mapy

#-------------------------------------------------------------------------------#

def init_rectify_map():
  vfov_rad = vfov_now * math.pi / 180.
  focal = height_now / 2. / math.tan(vfov_rad / 2.)
  
  Knew = np.identity(3, dtype = np.float64)
  Knew[0, 0] = focal
  Knew[1, 1] = focal
  Knew[0, 2] = width_now / 2 - 0.5
  Knew[1, 2] = height_now / 2 - 0.5
  
  img_size = [height_now, width_now]
  
  mapx[0], mapy[0] = init_undistort_rectify_map(
    Kl, Dl, Rl, Knew, xil, img_size, 'kRectPerspective')
  
  print('Width: {}, Height: {}, V.FoV: {}'.format(width_now, height_now, vfov_now))
  print('K Matrix:')
  print(Knew)
  
  if cam_model == "stereo":
    mapx[1], mapy[1] = init_undistort_rectify_map(
      Kr, Dr, Rr, Knew, xir, img_size, 'kRectPerspective')
    print('Ndisp: {}, Wsize: {}'.format(ndisp_now, wsize_now))
  
  print('')
  
#-------------------------------------------------------------------------------#

def OnTrackAngle(vfov_bar):
  global vfov_now, changed
  vfov_now = 60 + vfov_bar
  changed = True

#-------------------------------------------------------------------------------#

def OnTrackWidth(width_bar):
  global width_now, changed
  width_now = 480 + width_bar
  if width_now % 2 == 1:
    width_now = width_now - 1
  changed = True

#-------------------------------------------------------------------------------#

def OnTrackHeight(height_bar):
  global height_now, changed
  height_now = 360 + height_bar
  if height_now % 2 == 1:
    height_now = height_now - 1
  changed = True

#-------------------------------------------------------------------------------#

def OnTrackNdisp(ndisp_bar):
  global ndisp_now, changed
  ndisp_now = 16 + 16 * ndisp_bar
  changed = True

#-------------------------------------------------------------------------------#

def OnTrackWsize(wsize_bar):
  global wsize_now, changed
  wsize_now = 3 + 2 * wsize_bar
  changed = True

#-------------------------------------------------------------------------------#

def disparity_image(rect_imgl, rect_imgr):
  gray_imgl = cv2.cvtColor(rect_imgl, cv2.COLOR_BGR2GRAY)
  gray_imgr = cv2.cvtColor(rect_imgr, cv2.COLOR_BGR2GRAY)
  
  if is_sgbm == True:
    stereo = cv2.StereoSGBM_create(0, ndisp_now, wsize_now)
  else:
    stereo = cv2.StereoBM_create(ndisp_now, wsize_now)
    
  disparity = stereo.compute(gray_imgl, gray_imgr)
  
  norm_image = cv2.normalize(disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

  return norm_image
  
#-------------------------------------------------------------------------------#

def main():  
  global changed
  
  param_file = "astar_calicam.yml"
  image_name = "dasl_wood_shop.jpg"

  if len(sys.argv) == 2:
    param_file = sys.argv[1]

  if len(sys.argv) == 3:
    param_file = sys.argv[1]
    image_name = sys.argv[2]
  
  load_parameters(param_file)
  init_rectify_map()
  
  raw_img = cv2.imread(image_name, 1)
  
  param_win_name = "Raw Image: " + str(img_width) + " x " + str(cap_rows)
  
  cv2.namedWindow(param_win_name)
  cv2.createTrackbar("V. FoV:  60    +", param_win_name, vfov_bar, vfov_max, OnTrackAngle)
  cv2.createTrackbar("Width:  480 +", param_win_name, width_bar, width_max, OnTrackWidth)
  cv2.createTrackbar("Height: 360 +", param_win_name, height_bar, height_max, OnTrackHeight)
 
  disp_win_name  = "Disparity Image"
  if cam_model == "stereo":
    cv2.namedWindow(disp_win_name)
    cv2.createTrackbar("Num Disp:  16 + 16 *", disp_win_name, ndisp_bar, ndisp_max, OnTrackNdisp)
    cv2.createTrackbar("Blk   Size :     3  +  2 *", disp_win_name, wsize_bar, wsize_max, OnTrackWsize)
  
  while True:
    raw_imgl = None
    raw_imgr = None
    rect_imgl = None
    rect_imgr = None

    if changed == True:
      init_rectify_map()
      changed = False
    
    if cam_model == "stereo":
      raw_imgl = raw_img[:, : img_width]
      raw_imgr = raw_img[:, img_width : img_width * 2]
      
      rect_imgl = cv2.remap(raw_imgl, mapx[0], mapy[0], cv2.INTER_LINEAR)
      rect_imgr = cv2.remap(raw_imgr, mapx[1], mapy[1], cv2.INTER_LINEAR)
    else:
      raw_imgl = raw_img
      rect_imgl = cv2.remap(raw_imgl, mapx[0], mapy[0], cv2.INTER_LINEAR)
    
    dim = (img_width / 2, cap_rows / 2)
    small_img = cv2.resize(raw_imgl, dim, interpolation = cv2.INTER_NEAREST)
    
    cv2.imshow(param_win_name, small_img)
    cv2.imshow("Rectified Image", rect_imgl)
    
    if cam_model == "stereo":
      disp_img = disparity_image(rect_imgl, rect_imgr)
      cv2.imshow(disp_win_name, disp_img)
    
    key = cv2.waitKey(1)
    
    if key & 0xFF == ord('q') or key  == 27:
      break

#-------------------------------------------------------------------------------#

if __name__ == "__main__":
  main()

#-------------------------------------------------------------------------------#

