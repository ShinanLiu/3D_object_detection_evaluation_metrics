#include <glog/logging.h>
#include <cfloat>  // FLT_MAX
#include <cmath>
#include <algorithm>
#include <fstream>
#include <functional>  // greater<>
#include <iomanip>  // std::setprecision
#include <iostream>
#include <map>
#include <string>
#include <utility>  // pair<>
#include <vector>
#include <unordered_map>
#include <dirent.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/io/wkt/read.hpp>
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point> Polygon;

using std::pair;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::ofstream;
using namespace std;
const float EPS = 1e-8;
vector<string> CLASS_NAMES;
vector<string> DIFF_NAMES;
vector<string> METRIC_NAMES;
vector<string> DISTANCE;
vector<int> distance_thresh;
vector<vector<double> > cam_mat;
vector<vector<double> >points_gd;
vector<float>over_thresh = {0.5, 0.75f};
void initGlobals () {
  CLASS_NAMES.push_back("__background__");
  CLASS_NAMES.push_back("CAR");
  CLASS_NAMES.push_back("PEDESTRIAN");
  CLASS_NAMES.push_back("CYCLIST");
  DIFF_NAMES.push_back("ALL_LEVEL");
  DIFF_NAMES.push_back("L1");
  DIFF_NAMES.push_back("L2");
  DIFF_NAMES.push_back("L3");
  DISTANCE.push_back("ALL_DISTANCE");
  DISTANCE.push_back("20m");
  DISTANCE.push_back("30m");
  DISTANCE.push_back("40m");
  METRIC_NAMES.push_back("IMAGE");
  METRIC_NAMES.push_back("GROUND");
  METRIC_NAMES.push_back("BOX3D"); 
  METRIC_NAMES.push_back("VISUAL"); 
  distance_thresh.push_back(0);
  distance_thresh.push_back(20);
  distance_thresh.push_back(40);
  distance_thresh.push_back(60);
  double arr1[3] = {721.5377, 0.0, 609.5593};
  double arr2[3] = {0.0, 721.377, 172.8540};
  double arr3[3] = {0.0, 0.0, 1};
  vector<double>ar1(&arr1[0],&arr1[3]);
  vector<double>ar2(&arr2[0],&arr2[3]);
  vector<double>ar3(&arr3[0],&arr3[3]);
  cam_mat.push_back(ar1);
  cam_mat.push_back(ar2);
  cam_mat.push_back(ar3);
}

template <typename Dtype>
class Box {
 public:
  Box() {
    this->x1 = Dtype(0.);
    this->y1 = Dtype(0.);
    this->x2 = Dtype(0.);
    this->y2 = Dtype(0.);
    this->t1 = Dtype(0.);
    this->t2 = Dtype(0.);
    this->t3 = Dtype(0.);
    this->l  = Dtype(0.);
    this->w  = Dtype(0.);
    this->h  = Dtype(0.);
    this->ry = Dtype(0.);
    this->alpha = Dtype(0.);
    this->label = 0; 
    this->severity_level = 0;
  }

  Box(Dtype x1, Dtype y1, Dtype x2, Dtype y2) {
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
    this->label = 0;
  }

  Box(Dtype x1, Dtype y1, Dtype x2, Dtype y2, int label) {
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
    this->label = label;
  }

  Box(Dtype x1, Dtype y1, Dtype x2, Dtype y2, Dtype t1, Dtype t2, Dtype t3, Dtype l, Dtype w, Dtype h, Dtype ry, Dtype alpha, int label, int severity_level) {
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
    this->t1 = t1;
    this->t2 = t2;
    this->t3 = t3;
    this->l  = l;
    this->w  = w;
    this->h  = h;
    this->ry = ry;
    this->alpha = alpha;
    this->label = label;
    this->severity_level = severity_level;
  }


  // Binary operators (+) overloading
  Box<Dtype> operator+ (Dtype p) const {
    return Box<Dtype>(x1 + p, y1 + p, x2 + p, y2 + p, label);
  }

  // Binary operators (-) overloading
  Box<Dtype> operator- (Dtype p) const {
    return Box<Dtype>(x1 - p, y1 - p, x2 - p, y2 - p, label);
  }

  // Binary operators (*) overloading
  Box<Dtype> operator* (Dtype scale) const {
    return Box<Dtype>(x1 * scale, y1 * scale, x2 * scale, y2 * scale, label);
  }

  // Binary operators (/) overloading
  Box<Dtype> operator/ (Dtype scale) const {
    CHECK(scale > (1e-6) || scale < -(1e-6));
    return Box<Dtype>(x1 / scale, y1 / scale, x2 / scale, y2 / scale, label);
  }

  // Assignment operators overloading
  Box<Dtype>& operator= (const vector<Dtype>& vec) {
    bool valid_vec = (vec.size() == 4) || (vec.size() == 5);
    CHECK(valid_vec) << "Size of vector should be either 4 or 5 (with label).";

    this->x1 = Dtype(vec[0]);
    this->y1 = Dtype(vec[1]);
    this->x2 = Dtype(vec[2]);
    this->y2 = Dtype(vec[3]);
    if (vec.size() == 5) {
      this->label = static_cast<int>(vec[4]);
    } else {
      this->label = 0;
    }
    return *this;
  }

  // Subscripting [] operator overloading for non-const objects:
  // can be used for assignment
  Dtype& operator[] (const int index) {
    switch (index) {
    case 0:
      return x1;
    case 1:
      return y1;
    case 2:
      return x2;
    case 3:
      return y2;
    default:
      LOG(FATAL) << "Error Index";
    }
  }

  // Subscripting [] operator overloading for const objects:
  // can only be used for access
  const Dtype& operator[] (const int index) const {
    switch (index) {
    case 0:
      return x1;
    case 1:
      return y1;
    case 2:
      return x2;
    case 3:
      return y2;
    default:
      LOG(FATAL) << "Error Index";
    }
  }

  Dtype width() const {
    // x2 >= x1
    CHECK_GE(x2, x1) << "width() is not allowed when x1 > x2";
    return (x2 - x1 + Dtype(1.));
  }

  Dtype height() const {
    // y2 >= y1
    CHECK_GE(y2, y1) << "height() is not allowed when y1 > y2";
    return (y2 - y1 + Dtype(1.));
  }

  Dtype center_x() const {
    return x1 + width() * 0.5;
  }

  Dtype center_y() const {
    return y1 + height() * 0.5;
  }

  Dtype area() const {
    return height() * width();
  }

  bool x1y1x2y2() const {
    // valid box: x1 <= x2 && y1 <= y2
    if (x1 > x2 || y1 > y2) {
      return false;
    } else {
      return true;
    }
  }

  Dtype getIoU(const Box& box) const {
    Dtype lx = std::max(x1, box.x1);
    Dtype ly = std::max(y1, box.y1);
    Dtype rx = std::min(x2, box.x2);
    Dtype ry = std::min(y2, box.y2);
    if (lx > rx || ly > ry) {
      return Dtype(0.);
    }
    Dtype inter = (rx - lx + 1) * (ry - ly + 1);
    if ((this->area() + box.area() - inter) <= 0) {
      return Dtype(0.);
    }
    return inter / (this->area() + box.area() - inter);
  }

  Dtype get_ground_iou(const Box box) const{
    return groundBoxOverlap(*this, box);
  }

  Dtype get_3d_iou(const Box box) const{
    return box3DOverlap(*this, box);
  }

  Dtype get_visual_iou(const Box box) const{
    return visual_iou(*this, box);
  }

  Dtype myIoU(const Box& box) const {
    Dtype lx = std::max(x1, box.x1);
    Dtype ly = std::max(y1, box.y1);
    Dtype rx = std::min(x2, box.x2);
    Dtype ry = std::min(y2, box.y2);
    if (lx > rx || ly > ry) {
      return Dtype(0);
    }
    return (rx - lx + 1.)*(ry - ly + 1.) / this->area();
  }

  Dtype x1;
  Dtype y1;
  Dtype x2;
  Dtype y2;
  Dtype t1;
  Dtype t2;
  Dtype t3;
  Dtype l;
  Dtype w;
  Dtype h;
  Dtype alpha;
  Dtype ry;
  int label;
  int severity_level;
};

template <typename T>
Polygon toPolygon(const T& g) {
    points_gd.clear();
    using namespace boost::numeric::ublas;
    using namespace boost::geometry;
    matrix<double> mref(2, 2);
    mref(0, 0) = cos(g.ry); mref(0, 1) = sin(g.ry);
    mref(1, 0) = -sin(g.ry); mref(1, 1) = cos(g.ry);

    static int count = 0;
    matrix<double> corners(2, 4);
    double data[] = {g.l / 2, g.l / 2, -g.l / 2, -g.l / 2,
                     g.w / 2, -g.w / 2, -g.w / 2, g.w / 2};
    std::copy(data, data + 8, corners.data().begin());
    matrix<double> gc = prod(mref, corners);
    for (int i = 0; i < 4; ++i) {
        gc(0, i) += g.t1;
        gc(1, i) += g.t3;
    }

    double points6[][2] = {{gc(0, 0), gc(1, 0)},{gc(0, 1), gc(1, 1)},{gc(0, 2), gc(1, 2)},{gc(0, 3), gc(1, 3)},{gc(0, 0), gc(1, 0)}};
    for(int m=0;m<4;m++){
      std::vector<double> tmp_vec;
      for(int n=0;n<2;n++){
        tmp_vec.push_back(points6[m][n]);
      }
      points_gd.push_back(tmp_vec);
    }
    Polygon poly;
    append(poly, points6);
    return poly;
}

// measure overlap between bird's eye view bounding boxes, parametrized by (ry, l, w, tx, tz)
double groundBoxOverlap(Box<float> d, Box<float> g, int32_t criterion = -1) {
    using namespace boost::geometry;
    // vector<vector<double> >points;
    Polygon gp = toPolygon(g);
    Polygon dp = toPolygon(d);

    std::vector<Polygon> in, un;
    intersection(gp, dp, in);
    union_(gp, dp, un);

    double inter_area = in.empty() ? 0 : area(in.front());
    double union_area = area(un.front());
    double o;
    if(criterion==-1)     // union
        o = inter_area / union_area;
    else if(criterion==0) // bbox_a
        o = inter_area / area(dp);
    else if(criterion==1) // bbox_b
        o = inter_area / area(gp);

    return o;
}

// measure overlap between 3D bounding boxes, parametrized by (ry, h, w, l, tx, ty, tz)
double box3DOverlap(Box<float> d, Box<float> g, int32_t criterion = -1) {
    using namespace boost::geometry;
    // vector<vector<double> >points;
    Polygon gp = toPolygon(g);
    Polygon dp = toPolygon(d);

    std::vector<Polygon> in, un;
    intersection(gp, dp, in);
    union_(gp, dp, un);

    double ymax = min(d.t2, g.t2);
    double ymin = max(d.t2 - d.h, g.t2 - g.h);

    double inter_area = in.empty() ? 0 : area(in.front());
    double inter_vol = inter_area * max(0.0, ymax - ymin);

    double det_vol = d.h * d.l * d.w;
    double gt_vol = g.h * g.l * g.w;

    double o;
    if(criterion==-1)     // union
        o = inter_vol / (det_vol + gt_vol - inter_vol);
    else if(criterion==0) // bbox_a
        o = inter_vol / det_vol;
    else if(criterion==1) // bbox_b
        o = inter_vol / gt_vol;

    return o;
}

double multify(vector<double> vec1, vector<double> vec2){
  float res = 0.0;
  res = vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2];
  return res;
}

vector<double> trans_kp(vector<double> points){
  float w, u, v;
  vector<double> res_point;
  w = multify(points, cam_mat[2]);
  if (w==0.0) return res_point;
  u = multify(points, cam_mat[0])/w;
  v = multify(points, cam_mat[1])/w;
  res_point.push_back(u);
  res_point.push_back(v);
  return res_point;
}

vector<vector<double> > trans_3d_2d(vector<vector<double> >kp_list, float h, float t2){
  vector<vector<double> >point_list;
  for (int i=0; i<kp_list.size();i++){
    vector<double> cur_point;
    vector<double> o_point;
    vector<double> c_trans_point;
    vector<double> o_trans_point;
    cur_point.push_back(kp_list[i][0]);
    cur_point.push_back(t2);
    cur_point.push_back(kp_list[i][1]);
    o_point.push_back(kp_list[i][0]);
    o_point.push_back(t2-h);
    o_point.push_back(kp_list[i][1]);
    c_trans_point = trans_kp(cur_point);
    o_trans_point = trans_kp(o_point);
    point_list.push_back(c_trans_point);
    point_list.push_back(o_trans_point);
  }
  return point_list;
}

Polygon find_max_area(vector<vector<double> > kp_list){
  double cur_umax=0.0,cur_umin=2000.0;
  double cur_vmax=0.0,cur_vmin=2000.0;
  std::vector<double> six_points;
  int umax_idx = 0, vmax_idx=0, umin_idx=0, vmin_idx=0, umax2_idx=0,umin2_idx=0,tmp=0;
  for(int i=0;i<kp_list.size();i++){
    if (kp_list[i][0]>cur_umax) {
      cur_umax = kp_list[i][0];
      umax_idx = i;
    }
    else if (kp_list[i][0]==cur_umax){
      umax2_idx = i;
    }
    if (kp_list[i][0]<cur_umin) {
      cur_umin = kp_list[i][0];
      umin_idx = i;
    }
    else if (kp_list[i][0]==cur_umin){
      umin2_idx =i;
    }
  }
  if (kp_list[umax_idx][1]>kp_list[umax2_idx][1]) {
      tmp = umax_idx;
      umax_idx = umax2_idx;
      umax2_idx = tmp;
  }
  if (kp_list[umin_idx][1]>kp_list[umin2_idx][1]) {
      tmp = umin_idx;
      umin_idx = umin2_idx;
      umin2_idx = tmp;
  }
  for(int j=0;j<kp_list.size();j++){
    if (j==umax_idx||j==umax2_idx||j==umin_idx||j==umin2_idx) continue;
    if (kp_list[j][1]>cur_vmax) {
      cur_vmax = kp_list[j][1];
      vmax_idx = j;
    }
    if (kp_list[j][1]<cur_vmin) {
      cur_vmin = kp_list[j][1];
      vmin_idx = j;
    }
  }
  six_points.push_back(vmin_idx);
  six_points.push_back(umax_idx);
  six_points.push_back(umax2_idx);
  six_points.push_back(vmax_idx);
  six_points.push_back(umin2_idx);
  six_points.push_back(umin_idx);

  Polygon poly_six;
  for(int m=0;m<6;m++){
    int cur = six_points[m];
    bg::append(poly_six.outer(), Point(kp_list[cur][0], -1.0*kp_list[cur][1]));
  }
  bg::append(poly_six.outer(), Point(kp_list[vmin_idx][0], -1.0*kp_list[vmin_idx][1]));
  return poly_six;
}

double visual_iou(Box<float> d, Box<float> g, int32_t criterion = -1){
    using namespace boost::geometry;
    std::vector<std::vector<double> >gp_list, dp_list;
    Polygon gp = toPolygon(g);
    gp_list = trans_3d_2d(points_gd, g.h, g.t2);
    Polygon dp = toPolygon(d);
    dp_list = trans_3d_2d(points_gd, d.h, d.t2);
    Polygon poly_gp = find_max_area(gp_list);
    Polygon poly_dp = find_max_area(dp_list);

    std::vector<Polygon> in, un;
    bg::intersection(poly_gp, poly_dp, in);
    bg::union_(poly_gp, poly_dp, un);
    double inter_area = in.empty() ? 0 : bg::area(in.front());
    double union_area = bg::area(un.front());
    double o;
    if(criterion==-1)     // union
        o = inter_area / union_area;
    else if(criterion==0) // bbox_a
        o = inter_area / area(poly_dp);
    else if(criterion==1) // bbox_b
        o = inter_area / area(poly_gp);
    return o;
}

template <typename Dtype>
class ImageInfo {
 public:
  void read(std::ifstream& instr, bool has_ignore,
      Dtype min_side = Dtype(-1.)) {
    // Read meta-info of next image
    CHECK(instr >> image_path >> channels >> height >> width >> flip);
    // There are some boxes we may want to ignore.
    if (has_ignore) {
      readIgnores(instr);
    }
    // Read annotations of this image
    readGts(instr, min_side);
  }

  void flipping() {
    CHECK(flip) << "The flip flag should be set to ON";
    // Flip ground-truth boxes
    for (int i = 0; i < gt_boxes.size(); ++i) {
      Dtype x1 = gt_boxes[i].x1;
      Dtype x2 = gt_boxes[i].x2;
      gt_boxes[i].x1 = width - x2 - Dtype(1.);
      gt_boxes[i].x2 = width - x1 - Dtype(1.);
    }
    // Flip ignored boxes
    if (!has_fake_ignore) {
      for (int i = 0; i < ignores.size(); ++i) {
        Dtype x1 = ignores[i].x1;
        Dtype x2 = ignores[i].x2;
        ignores[i].x1 = width - x2 - Dtype(1.);
        ignores[i].x2 = width - x1 - Dtype(1.);
      }
    }
  }

  string image_path;
  int channels;
  int height;
  int width;
  bool flip;
  vector<Box<Dtype> > ignores;
  vector<Box<Dtype> > gt_boxes;
  bool has_fake_ignore;

 private:
  void readIgnores(std::ifstream& instr) {
    int num_ignores;
    Dtype x1;
    Dtype y1;
    Dtype x2;
    Dtype y2;
    int label;
    CHECK(instr >> num_ignores) << "INPUT IGNORE ERROR" << image_path;
    if (num_ignores > 0) {
      while (num_ignores--) {
        CHECK(instr >> label>> x1 >> y1 >> x2 >> y2);
        // TODO(kun): We skip sanity check for ignored boxes
        Box<Dtype> ignored_box(x1, y1, x2, y2);
        ignores.push_back(ignored_box);
      }
      has_fake_ignore = false;
    } else {
      CHECK(ignores.empty()) << "ignores must be empty when num_ignores is 0";
      Box<Dtype> fake_ignored_box;
      ignores.push_back(fake_ignored_box);
      has_fake_ignore = true;
    }
  }

  void readGts(std::ifstream& instr, Dtype min_side = Dtype(-1.)) {
    int num_gts;
    int label;
    Dtype x1;
    Dtype y1;
    Dtype x2;
    Dtype y2;
    Dtype truncation, occlusion;
    Dtype t1, t2, t3, l, w, h, ry, alpha;
    int severity_level;

    CHECK(instr >> num_gts) << "INPUT GT ERROR" << image_path;
    while (num_gts--) {
      CHECK(instr >> label >> truncation >> occlusion >> alpha >> x1 >> y1 >> x2 >> y2 >> h >> w >> l>> t1 >> t2 >> t3 >> ry >> severity_level) << image_path;
      if (x2 == width) {
        LOG(INFO) << "We detect x2 == width and decrease x2 by 1: "
            << image_path;
        x2 = x2 - Dtype(1.);
      }
      if (y2 == height) {
        LOG(INFO) << "We detect y2 == height and decrease y2 by 1: "
            << image_path;
        y2 = y2 - Dtype(1.);
      }
      // Sanity check
      CHECK_GE(x1, 0) << image_path << ": Invalid coords";  // x1 >= 0
      CHECK_GE(y1, 0) << image_path << ": Invalid coords";  // y1 >= 0
      CHECK_LE(x1, x2) << image_path << ": Invalid coords";  // x1 <= x2
      CHECK_LE(y1, y2) << image_path << ": Invalid coords";  // y1 <= y2
      CHECK_LT(x2, width) << image_path << ": Invalid coords";  // x1 < width
      CHECK_LT(y2, height) << image_path << ": Invalid coords";  // y2 < height
      CHECK(severity_level == 1 || severity_level == 2 || severity_level == 3)
          << "Invalid severity_level: " << severity_level;

      // Remove annotations with height or width less than min_side
      if ((y2 - y1 + Dtype(1.)) < min_side ||
          (x2 - x1 + Dtype(1.)) < min_side) {
        continue;
      }
      Box<Dtype> gt(x1, y1, x2, y2, t1, t2, t3, l, w, h, ry, alpha, label, severity_level);
      gt_boxes.push_back(gt);
    }
  }
};

class ResultInfo {
public:
  vector<float> recalls;
  ResultInfo(float ap, int offset_h, int offset_w, 
    int total_gt, int total_det, int label, string name,
    float ovthresh, float ave_overlap, const vector<int> &fppi_ind,
    const vector<pair<float, float> > &score_thresh_fppi, 
    const vector<float> &cumsumfp, const vector<float> &cumsumtp, const vector<float> &recalls, int metrics, int severity_level, int cal_distance) {
    this -> ap = ap;
    this -> offset_h = offset_h;
    this -> offset_w = offset_w;
    this -> total_gt = total_gt;
    this -> total_det = total_det;
    this -> label = label;
    this -> name = name;
    this -> ovthresh = ovthresh;
    this -> ave_overlap = ave_overlap;
    this -> fppi_ind = fppi_ind;
    this -> score_thresh_fppi = score_thresh_fppi;
    this -> cumsumtp = cumsumtp;
    this -> cumsumfp = cumsumfp;
    this -> recalls = recalls;
    this -> metrics = metrics;
    this -> severity_level = severity_level;
    this -> cal_distance = cal_distance;
    // print();
  }

  bool operator < (const ResultInfo &a) const {
    return ap < a.ap;
  } 

  void print() {
    // if (name != "sign_add3") return;
    if (severity_level!=0||cal_distance!=0) return;
    std::streamsize ss = std::cout.precision();
    cout << "[" << this->get_name() << "]\n";
    cout << "  " << "AP@" << std::setprecision(2) << ovthresh << " = " 
    << std::setprecision(ss) << ap << "\n";
    cout << "  " << "AVE IoU = " << ave_overlap << " (For matched Bbox)\n";
    cout << "  " << "AVE Pixel Offset: h = " << offset_h
        << " w = " << offset_w << " (For matched Bbox)\n";
    cout << "  " << total_gt << " ground truths in total\n";
    cout << "  " << total_det << " detections in total\n";
    for (int s = 0; s < score_thresh_fppi.size(); ++s) {
      cout << "  " << "FPPI@" << score_thresh_fppi[s].first
          << std::fixed << std::setprecision(3)
          << " score_thresh = " << score_thresh_fppi[s].second
          << " recall = " << recalls[fppi_ind[s]]
          << std::fixed << std::setprecision(0)
          << " fp = " << cumsumfp[fppi_ind[s]]
          << " missed = " << total_gt - cumsumtp[fppi_ind[s]]
          << std::setprecision(ss) << "\n";
    }
  }


  void save_file(ofstream &fout) {
    // if (name != "sign_add3") return;
    
    std::streamsize ss = fout.precision();
    fout << "[" << this->get_name() << "]," << "AP@" << std::setprecision(2) << ovthresh << "," 
    << std::setprecision(ss) << ap << "\n";
    fout<<"FPPI@,score_thresh,recall,fp,missed\n";
    for (int s = 0; s < score_thresh_fppi.size(); ++s) {
      fout << score_thresh_fppi[s].first
          << std::fixed << std::setprecision(3)
          << "," << score_thresh_fppi[s].second
          << "," << recalls[fppi_ind[s]]
          << std::fixed << std::setprecision(0)
          << "," << cumsumfp[fppi_ind[s]]
          << "," << total_gt - cumsumtp[fppi_ind[s]]
          << std::setprecision(ss) << "\n";
    }

  }
  string get_name() {
    ostringstream oss;
    oss<<name<<"&&"<<METRIC_NAMES[metrics]<<"&&"<<DIFF_NAMES[severity_level]<<"&&"<<DISTANCE[cal_distance];
    return oss.str();
  }

  int get_label() {
    return label;
  }
  float get_ap() {
    return ap;
  }
  int get_total_det() {
    return total_det;
  }
  int get_total_gt() {
    return total_gt;
  }
private:
  float ap,  ovthresh, ave_overlap;
  int offset_h, offset_w, label, total_det, total_gt, metrics, severity_level, cal_distance;
  string name;
  vector<pair<float, float> > score_thresh_fppi;
  vector<float> cumsumfp;
  vector<float> cumsumtp;
  vector<int> fppi_ind;
};

void get_keep_instance(const vector<int> &labels,
    vector<vector<bool> > &keep_res,
    vector<vector<bool> > &keep_gts,
    const vector<vector<Box<float> > >& all_gt_boxes,
    const vector<vector<Box<float> > >& all_ignores,
    const vector<vector<Box<float> > >& all_results,
    float ovthresh,
    float pixel_upper_thresh,
    float pixel_lower_thresh,
    int severity_level = 0,
    int cal_distance = 0) {
  int num_images = all_gt_boxes.size();
  for (int j = 0; j < num_images; ++j) {
      // The following results/ground truths will be ignored:
      //   1. any side less than pixel_lower_thresh
      //   2. any side greater than pixel_upper_thresh
      //   3. class label is invalid
      for (size_t k = 0; k < all_results[j].size(); ++k) {
        float box_h = all_results[j][k].height();
        float box_w = all_results[j][k].width();
        if ((box_h < pixel_lower_thresh) || (box_h > pixel_upper_thresh) ||
            (box_w < pixel_lower_thresh) || (box_w > pixel_upper_thresh) ||
            (std::find(labels.begin(), labels.end(), all_results[j][k].label) == labels.end())) {
            // (all_results[j][k].label != cur_cls)) {
          keep_res[j].push_back(false);
        } else {
          keep_res[j].push_back(true);
        }
      }
      for (size_t k = 0; k < all_gt_boxes[j].size(); ++k) {
        float box_h = all_gt_boxes[j][k].height();
        float box_w = all_gt_boxes[j][k].width();
        if ((box_h < pixel_lower_thresh) || (box_h > pixel_upper_thresh) ||
            (box_w < pixel_lower_thresh) || (box_w > pixel_upper_thresh) ||
            (std::find(labels.begin(), labels.end(), all_gt_boxes[j][k].label) == labels.end())) {
            // (all_gt_boxes[j][k].label != cur_cls)) {
          keep_gts[j].push_back(false);
        } else {
          keep_gts[j].push_back(true);

        }
      }
      if (severity_level > 0) {
        // Remove detections located on GTs of other severity level
        for (size_t k = 0; k < all_results[j].size(); ++k) {
          const Box<float> box = all_results[j][k];
          float ovmax = 0.0f;
          int argmax = -1;
          for (size_t g = 0; g < all_gt_boxes[j].size(); ++g) {
            float overlap = box.getIoU(all_gt_boxes[j][g]);
            if (overlap > ovmax) {
              ovmax = overlap;
              argmax = g;
            }
          }
          if (ovmax > ovthresh &&
              all_gt_boxes[j].at(argmax).severity_level != severity_level) {
            keep_res[j][k] = false;
          }
        }
        // Remove GTs of other severity level
        for (size_t k = 0; k < all_gt_boxes[j].size(); ++k) {
          if (all_gt_boxes[j][k].severity_level != severity_level) {
            keep_gts[j][k] = false;
          }
        }
      }
    }

    // ADD: remove ignores in res
    for (int j = 0; j < num_images; ++j) {
      float ovmax = -1.0f;
      int argmax = -1;
      for (size_t k = 0; k < all_results[j].size(); ++k) {
        if (keep_res[j][k] == false) continue;
        for (size_t l = 0; l < all_ignores[j].size(); ++l) {
           const Box<float> box = all_results[j][k];
           float overlap =  box.getIoU(all_ignores[j][l]);
           if (overlap > ovthresh) {
            keep_res[j][k] = false;
            continue;
           }
        }
      }
    }
    if (cal_distance){
      for (int h=0; h< num_images;++h){
        for(size_t p=0; p<all_gt_boxes[h].size();++p){
          if (keep_gts[h][p]==false) continue;
          const Box<float> gt_box = all_gt_boxes[h][p];
          if (gt_box.t3>distance_thresh[cal_distance]){
            keep_gts[h][p] = false;
          }
        }
      }
      for (int n=0; n< num_images;++n){
        for(size_t m=0; m<all_results[n].size();++m){
          if (keep_res[n][m]==false) continue;
          const Box<float> det_box = all_results[n][m];
          if (det_box.t3>distance_thresh[cal_distance]){
            keep_res[n][m]=false;
          }
        }
      }
    }
  return ;
}

void cal_accuracy(
    const vector<pair<float, float>> &recall_at_score,
    const vector<int> &belong_to,
    const vector<int> &res_to_gt,
    const vector<Box<float> > &results,
    const vector<vector<Box<float> > > &gt_boxes,
    const vector<string>& sub_cls_names,
    const vector<pair<float, int>> &scores,
    ofstream &fout) {
  int num_dets = results.size();
  for (auto rec_score : recall_at_score) {
    auto recall = rec_score.first;
    auto th_score = rec_score.second;
    std::map<string, pair<int, int>> sub_cls_cnt;
    for (int j = 0; j < num_dets; ++j) {
      int det_ind = scores[j].second;

      int image_ind = belong_to[det_ind];
      // cout<<res_to_gt.size()<<" "<<j<<endl;
      int argmax = res_to_gt[j];
      const Box<float> &res = results[det_ind];
      if (th_score > scores[j].first) continue;

      string cls_name;
      cls_name = sub_cls_names[res.label];
      if (sub_cls_cnt.find(cls_name) == sub_cls_cnt.end()) {
        sub_cls_cnt[cls_name] = {0, 0};
      }
      sub_cls_cnt[cls_name].first += 1;
      if (argmax == -1 ) {
        continue;
      }
      const Box<float> gt = gt_boxes[image_ind][argmax];
      if (res.label == gt.label) {
        sub_cls_cnt[cls_name].second += 1;
      }
    }
    // cout<<"["<<super_cls_name<<"]"<<"\tRecall@"<<recall<<"\t"<<"Score="<<th_score<<"\n";
    // fout<<"["<<super_cls_name<<"]"<<",Recall@"<<recall<<","<<"Score="<<th_score<<"\n";
    fout<<"Name,"<<"Accuracy,"<<"Right,"<<"Total\n";

    int total_num = 0;
    int total_right = 0;
    for (auto item : sub_cls_cnt) {
      cout<<"\t"<<item.first<<":\tAccuracy="<<float(item.second.second)/item.second.first<<
      "\tRight="<<item.second.second<<"\tTotal="<<item.second.first<<endl;

      fout<<item.first<<","<<float(item.second.second)/item.second.first<<","<<item.second.second<<","<<item.second.first<<endl;
      
      total_num += item.second.first;
      total_right += item.second.second;
    }
    cout<<"\t"<<"Total"<<":\tAccuracy="<<float(total_right)/total_num<<
      "\tRight="<<total_right<<"\tTotal="<<total_num<<endl;
    fout<<"Total,"<<float(total_right)/total_num<<","<<total_right<<","<<total_num<<endl;

  }
}

void eval(const vector<vector<Box<float> > >& all_gt_boxes,
    const vector<vector<Box<float> > >& all_ignores,
    const vector<vector<Box<float> > >& all_results,
    const vector<vector<vector<float> > >& all_scores,
    const vector<string>& sub_cls_names,
    const vector<vector<int>>& cls_ids,
    float ovthresh,
    float pixel_upper_thresh,
    float pixel_lower_thresh,
    ofstream &fout,
    int severity_level = 0,
    int metrics = 0,
    int cal_distance = 0) {
  // Metrics
  //
  // PASCAL VOC: AP at IoU=0.50
  //
  // COCO: AP at IoU=0.50 (PASCAL VOC metric)
  // COCO: AP at IoU=0.75 (strict metric)
  //
  vector<string> cls_name;
  cls_name = sub_cls_names;
  typedef pair<float, int> P;
  
  CHECK_GT(ovthresh, 0) << "Invalid over_thresh!";
  CHECK_LT(ovthresh, 1) << "Invalid over_thresh!";
  // Notice: cls_name MUST contains background
  int num_classes = cls_name.size();
  CHECK_EQ(cls_name[0], "__background__")
      << "The first class should be background";
  CHECK_GT(num_classes, 1) << "num_classes should be greater than 1";
  for (int i = 0; i < all_gt_boxes.size(); ++i) {
    for (int j = 0; j < all_gt_boxes[i].size(); ++j) {
      CHECK_LT(all_gt_boxes[i][j].label, num_classes)
          << "gt_boxes.label too large";
    }
  }
  for (int i = 0; i < all_results.size(); ++i) {
    for (int j = 0; j < all_results[i].size(); ++j) {
      CHECK_LT(all_results[i][j].label, num_classes)
          << "detection results label too large";
    }
  }


  int num_images = all_gt_boxes.size();
  CHECK_EQ(num_images, all_results.size())
      << "image number and their results mismatch";
  CHECK_EQ(num_images, all_scores.size())
      <<  "image number and their results mismatch";

  // pixelTolerance
  if (pixel_upper_thresh < 1.0f) {
    pixel_upper_thresh = FLT_MAX;
  }
  CHECK_GT(pixel_upper_thresh, pixel_lower_thresh)
      << "Upper thresh should be greater than lower thresh!";

  vector<float> all_ap;
  vector<ResultInfo> results_all;
  int num_dets;
  //cout<<"xxxxxxxxxx\n";
  // Skip cur_cls = 0, because it's the background class
  for (int cur_cls = 1; cur_cls < num_classes; ++cur_cls) {
    // 1. Remove useless results and ground truths
    vector<vector<bool> > keep_res(num_images, vector<bool>());
    vector<vector<bool> > keep_gts(num_images, vector<bool>());
    get_keep_instance(cls_ids[cur_cls], keep_res, keep_gts, all_gt_boxes, 
      all_ignores, all_results, ovthresh, pixel_upper_thresh, pixel_lower_thresh, severity_level, cal_distance);

    // 2. Collect ground truths and detections of current class
    vector<vector<Box<float> > > gt_boxes(num_images, vector<Box<float> >());
    vector<vector<bool> > seen(num_images, vector<bool>());
    int num_pos = 0,num_res;
    for (int j = 0; j < num_images; ++j) {
      for (int k = 0; k < all_gt_boxes[j].size(); ++k) {
        if (keep_gts[j][k] == false) {
          continue;
        }
        gt_boxes[j].push_back(all_gt_boxes[j][k]);
        seen[j].push_back(false);
        num_pos += 1;
      }
    }
    if (num_pos < 20) continue;
    CHECK_GT(num_pos, 0)
        << "There should be at least one ground truth for class "
        << cls_name[cur_cls];
    // cout<<cls_name[cur_cls]<<endl;

    vector<Box<float> > results;
    vector<pair<float, int> > scores;
    // belong_to stores image index for each detection of current class
    vector<int> belong_to;
    vector<int> res_to_gt;
    for (int j = 0; j < num_images; ++j) {
      for (int k = 0; k < all_results[j].size(); ++k) {
        if (keep_res[j][k] == false) {
          continue;
        }
        // Collect the results with cur_cls
        belong_to.push_back(j);
        results.push_back(all_results[j][k]);
        res_to_gt.push_back(-1);
        scores.push_back(P(all_scores[j][k][0], scores.size()));
      }
    }
    sort(scores.begin(), scores.end(), std::greater<P>());

    // 3. Calculate FP and TP
    num_dets = results.size();
    if (num_dets == 0) {
      continue;
    }
    vector<float> fp(num_dets, 0.0f);
    vector<float> cumsumfp(num_dets, 0.0f);
    vector<float> cumsumtp(num_dets, 0.0f);

    vector<float> tp(num_dets, 0.0f);
    vector<float> nailed_overlap;
    vector<pair<int, int> > nailed_offset;
    // FPPI: False Positive Per Image
    vector<pair<float, float> > score_thresh_fppi;
    score_thresh_fppi.push_back(std::make_pair(1.00f, 0.0f));
    score_thresh_fppi.push_back(std::make_pair(0.10f, 0.0f));
    score_thresh_fppi.push_back(std::make_pair(0.01f, 0.0f));

    vector<int> fppi_ind(score_thresh_fppi.size(), 0);

    for (int j = 0; j < num_dets; ++j) {
      int det_ind = scores[j].second; 
      // Fetch predicted box according to scores one-by-one
      const Box<float> box = results[det_ind];
      int image_ind = belong_to[det_ind];
      // Fetch ground truth from this image
      const vector<Box<float> > gts = gt_boxes[image_ind];

      float ovmax = -1.0f;
      int argmax = -1;
      pair<int, int> pixel_offset;
      // metrics = 0;
      for (int ip = 0; ip < gts.size(); ++ip) {
        float overlap = 0.0;
        if (metrics == 0){
         overlap = box.getIoU(gts[ip]);
        }
        else if (metrics == 1){
          overlap = box.get_ground_iou(gts[ip]);
        }
        else if (metrics ==2){
          overlap = box.get_3d_iou(gts[ip]);
        }
        else {
          overlap = box.get_visual_iou(gts[ip]);
          // cout<<box.x1<<" "<<box.y1<<" "<<box.x2<<" "<< box.y2<<"det"<<endl;
          // cout<<gts[ip].x1<<endl;
        }
        if (overlap > ovmax) {
          ovmax = overlap;
          argmax = ip;
          pixel_offset.first = static_cast<int>(std::max(
              std::fabs(box.x1-gts[ip].x1),
              std::fabs(box.x2-gts[ip].x2)));
          pixel_offset.second = static_cast<int>(std::max(
              std::fabs(box.y1-gts[ip].y1),
              std::fabs(box.y2-gts[ip].y2)));
        }
      }
      // TODO add overmax with ignore
      // Nailed it!
      if (ovmax >= ovthresh && !seen[image_ind][argmax]) {
        tp[j] = 1.0f;
        if (j == 0) {
          cumsumtp[j] = tp[j];
          cumsumfp[j] = 0;
        } else {
          cumsumtp[j] = cumsumtp[j-1] + tp[j];
          cumsumfp[j] = cumsumfp[j-1];
        }
        // This matched ground truth is seen
        seen[image_ind][argmax] = true;
        res_to_gt[j] = argmax;
        // cumsumfp keeps unchanged when no fp
        
        nailed_overlap.push_back(ovmax);
        nailed_offset.push_back(pixel_offset);
      } else {
        fp[j] = 1.0f;
        
        if (j == 0) {
          cumsumfp[j] = fp[j];
          cumsumtp[j] = 0;
        } else {
          cumsumfp[j] = cumsumfp[j-1] + fp[j];
          cumsumtp[j] = cumsumtp[j-1];
        }
        for (size_t s = 0; s < score_thresh_fppi.size(); ++s) {
          if (cumsumfp[j] < num_images * score_thresh_fppi[s].first) {
            score_thresh_fppi[s].second = scores[j].first;
            fppi_ind[s] = j;
          }
        }
      }
    }

    // 4. Calculate recall and precision
    vector<float> recalls(num_dets, 0.0f);
    vector<float> precisions(num_dets, 0.0f);
    vector<pair<float, float>> recall_at_score;
    recall_at_score.push_back({0.5, -1});
    recall_at_score.push_back({0.6, -1});
    recall_at_score.push_back({0.8, -1});
    // recall_at_score.push_back({0.2, -1});
    // recall_at_score.push_back({0.3, -1});
    // recall_at_score.push_back({0.5, -1});
    for (int j = 0; j < num_dets; ++j) {
      if (j > 0) {
        fp[j] += fp[j-1];
        tp[j] += tp[j-1];
      }
      recalls[j] = tp[j] / num_pos;
      if ((tp[j] + fp[j]) > 0.0f) {
        precisions[j] = tp[j] / (tp[j] + fp[j]);
      } else {
        precisions[j] = 0.0f;
      }
      CHECK_LE(precisions[j], 1.0f);
    }

    for (int j = 0; j < num_dets; ++j) {
      // cout<<recalls[j]<<" "<<scores[j].first<<endl;
      for (size_t k = 0; k < recall_at_score.size(); ++k) {
        if (recall_at_score[k].second == -1 && recall_at_score[k].first < recalls[j] - EPS) {
          recall_at_score[k].second = scores[j].first;
          // cout<<j<<" "<<scores[j].first<<" "<<recall_at_score[k].first<<" "<<recalls[j]<<endl;
        }
      }
    }
	for (size_t k = 0; k < recall_at_score.size(); ++k) {
	  if (recall_at_score[k].second == -1) {
	    recall_at_score[k].first = recalls[num_dets-1];
	    recall_at_score[k].second = scores[num_dets-1].first;
	  }
	}
    for (int j = num_dets - 2; j >= 0; --j) {
      precisions[j] = std::max(precisions[j], precisions[j+1]);
    }

    float ap = 0.0f;
    // for (int j = 0; j + 1 < num_dets; ++j) {
    //   if (j == 0) {
    //     ap += recalls[j] * precisions[j];
    //   } else {
    //     ap += (recalls[j] - recalls[j-1]) * precisions[j];
    //   }
    // }
    for (int j = 0; j + 1 < num_dets; ++j) {
      if (j == 0) {
        ap += precisions[j];
      } else {
        ap += precisions[j];
      }
    }

    // Collect AP category-by-category
    all_ap.push_back(ap/num_dets);

    float sum_overlap = 0.0f;
    int sum_offset_h = 0;
    int sum_offset_w = 0;
    int num_nailed = nailed_overlap.size();
    // num_nailed may not be equal to num_dets
    for (int j = 0; j < num_nailed; ++j) {
      sum_overlap += nailed_overlap[j];
      if (nailed_overlap[j]>1.0) std::cout<<metrics<<endl;
      sum_offset_h += nailed_offset[j].first;
      sum_offset_w += nailed_offset[j].second;
    }
    float ave_overlap = sum_overlap / std::max(num_nailed, 1);
    int ave_offset_h = sum_offset_h / std::max(num_nailed, 1);
    int ave_offset_w = sum_offset_w / std::max(num_nailed, 1);

    results_all.push_back(ResultInfo(ap/num_dets, ave_offset_h, ave_offset_w, 
      num_pos, num_dets, cur_cls, cls_name[cur_cls], ovthresh, 
      ave_overlap, fppi_ind, score_thresh_fppi, cumsumfp, cumsumtp, recalls, metrics, severity_level, cal_distance));
  }  // Now go to evaluate next category

  // results_all[0].print();
  sort(results_all.begin(), results_all.end());
  for (auto r : results_all) {
    r.print();
    r.save_file(fout);
  }
  // Accumulate AP
  float sumAP = 0.0f;
  for (int i = 0; i < all_ap.size(); ++i) {
    sumAP += all_ap[i];
  }
  float mAP = sumAP / all_ap.size();
  if (severity_level==0&&cal_distance==0){
  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
  cout << "Number of images: " << num_images << endl;
  cout << "Mean AP: " << mAP<< endl;
  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
  // LOG(INFO) << "**unofficial** eval";
  }
  // std::cout<<all_ap.size()<<std::endl;
}

int main(int argc, char* argv[]) {
  initGlobals();
  if (argc < 3) {
    std::cout << "Usage: sun_eval.bin test_anno.txt test_res.txt class.txt [super_class.txt]" << std::endl;
    return -1;
  }

  string test_anno_txt = argv[1];
  std::ifstream ifstr_anno(test_anno_txt);
  CHECK(ifstr_anno) << "No such file:" << test_anno_txt;

  // Load detection results
  string test_res_txt = argv[2];
  std::ifstream ifstr_dets(test_res_txt);
  CHECK(ifstr_dets) << "No such file:" << test_res_txt;
  std::map<string, pair<vector<Box<float> >, vector<float> > > detections;
  std::map<string, pair<vector<Box<float> >, vector<vector<float > > > > detections_3d;
  string image_path;
  vector<float> score(2, 0.0);
  float cls_id, x1, y1, x2, y2, t1, t2, t3, l, w, h, ry, alpha, score_2d, score_3d;
  while (ifstr_dets >> image_path >> cls_id >> alpha >> x1 >> y1 >> x2 >> y2 >> 
        h >> w >> l >> t1 >> t2 >> t3  >> ry >> score_2d>>score_3d) {
    detections_3d[image_path].first.push_back(Box<float>(x1, y1, x2, y2, t1, t2, t3, l, w, h, ry, alpha, static_cast<int>(cls_id), 0));
    score.clear();
    score.push_back(score_2d);
    score.push_back(score_3d);
    detections_3d[image_path].second.push_back(score);
  }

  int num_classes = CLASS_NAMES.size();
  int num_metrics = METRIC_NAMES.size();
  int num_distance = DISTANCE.size();
  int num_severity = DIFF_NAMES.size();
  string out_name = "eval_result.csv";
  if (argc > 3) {
    out_name = argv[3];
  }
  ofstream fout(out_name);

  vector<vector<int> > cls_ids(num_classes, vector<int>());
  for (int i = 0; i < num_classes; ++i) {
    cls_ids[i].push_back(i);
  }

  vector<vector<Box<float> > > all_gt_boxes;
  vector<vector<Box<float> > > all_results;
  vector<vector<vector<float> > > all_scores;
  vector<vector<Box<float> > > all_ignores;
  string hash_tag;
  int image_index;
  while (ifstr_anno >> hash_tag >> image_index) {
    // Read annotation
    ImageInfo<float> image;
    image.read(ifstr_anno, true);

    // Load detection results of current p image
    vector<Box<float> > results = detections_3d[image.image_path].first;
    vector<vector<float> > scores = detections_3d[image.image_path].second;

    // Collect detections and annotations
    all_gt_boxes.push_back(image.gt_boxes);
    all_ignores.push_back(image.ignores);
    all_results.push_back(results);
    all_scores.push_back(scores);
  }
  // Calculate mAP and other metrics
  
  const float pixel_upper_thresh = 0;  // 0 means no limits
  const float pixel_lower_thresh = 0;
  
  for (float th : over_thresh) {
    for (int j=0; j<num_severity; j++){           //severity
      for (int h=1; h<num_metrics;h++){          //metrics
        for(int k=0; k<num_distance;k++){         //distance
            eval(all_gt_boxes, all_ignores, all_results, all_scores, CLASS_NAMES,
		  cls_ids, th, pixel_upper_thresh, pixel_lower_thresh, fout, j, h, k);
        }       
      }
    }
  }
  return 0;
}