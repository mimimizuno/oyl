// //oyl_video.cpp

// #if defined(UNIVERSAL_HEADER_ONLY) || defined(OYL_VIDEO_HEADER_ONLY)
//   #define OYL_VIDEO_HEADER_ONLY_INLINE inline
// #else
//   #define OYL_VIDEO_HEADER_ONLY_INLINE
// #endif

// #include "oyl_video.hpp"
// #include <iostream>

// namespace oyl {
// #pragma region normalizeto255
//     // 参照で渡せるように修正
//     std::vector<std::vector<std::vector<int>>> normalizeto255(const std::vector<std::vector<std::vector<double>>>& video_data_double){
//         int t_frame = video_data_double.size();
//         int x_width = video_data_double[0].size();
//         int y_height = video_data_double[0][0].size();
//         std::vector<std::vector<std::vector<int>>> normalized_video_data;
//         std::vector<std::vector<std::vector<int>>> normalized_video_data_dummy;
//         normalized_video_data.resize(t_frame, std::vector<std::vector<int>>(x_width, std::vector<int>(y_height)));
//         normalized_video_data_dummy.resize(t_frame, std::vector<std::vector<int>>(x_width, std::vector<int>(y_height)));
//         double min_val=video_data_double[0][0][0];
//         double max_val=video_data_double[0][0][0];
//         for (const auto& t : video_data_double) {
//             for (const auto& x : t) {
//                 for (double y : x) {
//                     min_val = std::min(min_val, y);
//                     max_val = std::max(max_val, y);
//                 }
//             }
//         }

//         if (max_val-min_val<=0) {
//             std::cerr << "Error: Data has no range (min == max). Normalization skipped." << std::endl;
//             return normalized_video_data_dummy;
//         }

//         for (int t = 0; t < t_frame; t++) {
//             for (int x = 0; x < x_width; x++) {
//                 for (int y = 0; y < y_height; y++) {
//                     normalized_video_data[t][x][y] = static_cast<int>(255.0 * (video_data_double[t][x][y] - min_val) / (max_val - min_val));
//                 }
//             }
//         }
//         return normalized_video_data;
//     }
// #pragma endregion

// #pragma region functions of basic_makevideo
//     void basic_makevideo_int(std::vector<std::vector<std::vector<int>>> basic_video_data_int){
//         int t_frame = basic_video_data_int.size();
//         int x_width = basic_video_data_int[0].size();
//         int y_height = basic_video_data_int[0][0].size();
//         int fps = 30;
//         cv::VideoWriter writer;
//         int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1'); // H.264 codec
//         cv::Size framesize(x_width, y_height);
//         writer.open("output_video.mp4", codec, fps, framesize);
//         if (!writer.isOpened()) {
//             std::cerr << "Failed to create the video file." << std::endl;
//             return;
//         }

//         int cv_r, cv_b, cv_g;
//         for (const auto& frame_data : basic_video_data_int) {
//             cv::Mat frame(y_height, x_width, CV_8UC3); //RGB

//             for (int x = 0; x < x_width; ++x) {
//                 for (int y = 0; y < y_height; ++y) {
//                     cv_r = frame_data[x][y];
//                     cv_g = frame_data[x][y];
//                     cv_b = frame_data[x][y];
//                     frame.at<cv::Vec3b>(y, x) = cv::Vec3b(cv_r, cv_g, cv_b); 
//                 }
//             }
            
//             writer.write(frame);
//         }
//         std::cout << "The video has been completed." <<std::endl;
//     }//basic_makevideo_int

//     void basic_makevideo_double(std::vector<std::vector<std::vector<double>>> basic_video_data_double){
//         int t_frame = basic_video_data_double.size();
//         int x_width = basic_video_data_double[0].size();
//         int y_height = basic_video_data_double[0][0].size();
//         int fps = 30;
//         cv::VideoWriter writer;
//         int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1'); // H.264 codec
//         cv::Size framesize(x_width, y_height);
//         writer.open("output_video.mp4", codec, fps, framesize);
//         if (!writer.isOpened()) {
//             std::cerr << "Failed to create the video file." << std::endl;
//             return;
//         }

//         std::vector<std::vector<std::vector<int>>> video_data = normalizeto255(basic_video_data_double);
//         int cv_r, cv_b, cv_g;
//         for (const auto& frame_data : video_data) {
//             cv::Mat frame(y_height, x_width, CV_8UC3); //RGB

//             for (int x = 0; x < x_width; ++x) {
//                 for (int y = 0; y < y_height; ++y) {
//                     cv_r = frame_data[x][y];
//                     cv_g = frame_data[x][y];
//                     cv_b = frame_data[x][y];
//                     frame.at<cv::Vec3b>(y, x) = cv::Vec3b(cv_r, cv_g, cv_b); 
//                 }
//             }
            
//             writer.write(frame);
//         }
//         std::cout << "The video has been completed." <<std::endl;

//     }//basic_makevideo_double
// #pragma end region

// #pragma region VideoClass
//     VideoClass::VideoClass(std::vector<std::vector<std::vector<int>>> video_data){
//         this->video_data = video_data;
//         filename = "output_video.mp4";
//         t_size  = video_data.size();       //t for frame
//         x_size  = video_data[0].size();    //x for width 
//         y_size  = video_data[0][0].size(); //y for height
//         cell_size = 10;
//         x_width = cell_size * x_size;
//         y_height= cell_size * y_size;
//         // x_width and y_height depend on cell_size.
//         // Therefore, if cell_size changes, x_width and y_height must be updated accordingly.
//         codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1'); // H.264 codec
//         flag_scaleBar = false;
//         bar_width = 0;
//         gap_width = 0;
//     }

//     void VideoClass::update_xwidth_yheight(){
//         x_width = cell_size * x_size;
//         y_height= cell_size * y_size;
//     }

//     cv::Mat VideoClass::create_frame(const std::vector<std::vector<int>>& frame_data) const {
//         cv::Mat frame(y_height, x_width, CV_8UC3);  // RGB

//         for (int x = 0; x < x_size; x++) {
//             for (int y = 0; y < y_size; y++) {
//                 int cv_r = frame_data[x][y];
//                 int cv_g = frame_data[x][y];
//                 int cv_b = frame_data[x][y];
//                 cv::Rect cell(cell_size * x, cell_size * y, cell_size, cell_size);
//                 frame(cell) = cv::Scalar(cv_b, cv_g, cv_r);
//             }
//         }
//         return frame;
//     }

//     void VideoClass::initialize_scaleBar() {
//         if (bar_width < 1) bar_width = static_cast<int>(x_width * 0.1);
//         if (gap_width < 1) gap_width = static_cast<int>(x_width * 0.1);
//     }

//     cv::Mat VideoClass::add_scaleBar(const cv::Mat& frame) const {
//         cv::Mat scale_bar(y_height, bar_width, CV_8UC3);
//         cv::Mat gap(y_height, gap_width, CV_8UC3, cv::Scalar(255, 255, 255));

//         int cv_r, cv_b, cv_g;
//         for (int x = 0; x < bar_width; ++x) {
//             for (int y = 0; y < y_height; ++y) {
//                 cv_r = 255 * (y_height-y) / y_height;
//                 cv_g = 255 * (y_height-y) / y_height;
//                 cv_b = 255 * (y_height-y) / y_height;
//                 scale_bar.at<cv::Vec3b>(y, x) = cv::Vec3b(cv_r, cv_g, cv_b); 
//             }
//         }
//         //std::cout <<  "scaleBar" << scale_bar << std::endl;
        
//         cv::Mat frame_with_scaleBar;
//         cv::hconcat(std::vector<cv::Mat>{frame, gap, scale_bar}, frame_with_scaleBar);
        
//         return frame_with_scaleBar;
//         // No need to return by reference since cv::Mat uses reference counting.
//     }

//     void VideoClass::makevideo() const {
//         int img_width  = x_width+bar_width+gap_width;
//         int img_height = y_height;
//         cv::VideoWriter writer;
//         cv::Size framesize(img_width, img_height);
//         writer.open(filename, codec, fps, framesize);

//         if (!writer.isOpened()) {
//             std::cerr << "Failed to create the video file." << std::endl;
//             return;
//         }

//         for (const std::vector<std::vector<int>>& frame_data : video_data) {
//             cv::Mat frame = create_frame(frame_data);

//             if (flag_scaleBar) {
//                 frame = add_scaleBar(frame);
//             }
            
//             writer.write(frame);
//         }
//         std::cout << "The video has been completed." <<std::endl;
//     } //VideoClass_int::makevideo

//     void VideoClass::set_filename(std::string filename){
//         this->filename = filename;
//     }

//     void VideoClass::set_codec(int codec){
//         this->codec = codec;
//     }

//     void VideoClass::set_fps(double fps){
//         this->fps = fps; 
//     }

//     void VideoClass::set_cellsize(int cell_size){
//         this->cell_size = cell_size;
//         update_xwidth_yheight();
//     }

//     VideoClass& VideoClass::set_scaleBar(bool flag_scaleBar){
//         this->flag_scaleBar = flag_scaleBar;
//         if (flag_scaleBar) initialize_scaleBar();
//         return *this;
//     }

//     VideoClass& VideoClass::set_barwidth(int bar_width){
//         this->bar_width = bar_width;
//         return *this;
//     }

//     VideoClass& VideoClass::set_gapwidth(int gap_width){
//         this->gap_width = gap_width;
//         return *this;
//     }

//     void VideoClass::show_parameters() const {
//         int img_width  = x_width+bar_width+gap_width;
//         int img_height = y_height;
//         std::cout << "filename=" << filename << std::endl;
//         std::cout << "fps=" << fps << " codec=" << codec <<std::endl;
//         std::cout << "t_size=" << t_size << " x_size=" << x_size << " y_size=" << y_size << std::endl;     //size of frame
//         std::cout << "cell_size=" << cell_size << " x_width=" << x_width << " y_height" << y_height << std::endl;
//         std::cout << "img_width=" << img_width << " img_height=" << img_height << std::endl; 
//         std::cout << "Pixel resolution is img_width x img_height, (i.e.," << img_width << "x" << img_height << ")" << std::endl;
//     }

//     void VideoClass::show_size() const {
//         std::cout << "t=" << t_size << " x=" << x_size << " y=" << y_size;
//         std::cout << ": t for frame, x for width, y for height" <<std::endl;
//     }
// #pragma endregion // VideoClass_int

// }// namespace oyl


// 修正版 oyl_video.cpp ([t][y][x]対応版)

#if defined(UNIVERSAL_HEADER_ONLY) || defined(OYL_VIDEO_HEADER_ONLY)
  #define OYL_VIDEO_HEADER_ONLY_INLINE inline
#else
  #define OYL_VIDEO_HEADER_ONLY_INLINE
#endif

#include "oyl_video.hpp"
#include <iostream>

namespace oyl {

#pragma region normalizeto255
std::vector<std::vector<std::vector<int>>> normalizeto255(const std::vector<std::vector<std::vector<double>>>& video_data_double){
    int t_frame = video_data_double.size();
    int y_height = video_data_double[0].size();
    int x_width = video_data_double[0][0].size();
    std::vector<std::vector<std::vector<int>>> normalized_video_data(t_frame, std::vector<std::vector<int>>(y_height, std::vector<int>(x_width)));
    std::vector<std::vector<std::vector<int>>> dummy(t_frame, std::vector<std::vector<int>>(y_height, std::vector<int>(x_width)));

    double min_val = video_data_double[0][0][0];
    double max_val = video_data_double[0][0][0];

    for (const auto& t : video_data_double) {
        for (const auto& y : t) {
            for (double x : y) {
                min_val = std::min(min_val, x);
                max_val = std::max(max_val, x);
            }
        }
    }

    if (max_val - min_val <= 0) {
        std::cerr << "Error: Data has no range (min == max). Normalization skipped." << std::endl;
        return dummy;
    }

    for (int t = 0; t < t_frame; ++t) {
        for (int y = 0; y < y_height; ++y) {
            for (int x = 0; x < x_width; ++x) {
                normalized_video_data[t][y][x] = static_cast<int>(255.0 * (video_data_double[t][y][x] - min_val) / (max_val - min_val));
            }
        }
    }
    return normalized_video_data;
}
#pragma endregion

#pragma region basic_makevideo
void basic_makevideo_int(std::vector<std::vector<std::vector<int>>> basic_video_data_int){
    int t_frame = basic_video_data_int.size();
    int y_height = basic_video_data_int[0].size();
    int x_width = basic_video_data_int[0][0].size();
    int fps = 30;

    cv::VideoWriter writer;
    int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    cv::Size framesize(x_width, y_height);
    writer.open("output_video.mp4", codec, fps, framesize);

    if (!writer.isOpened()) {
        std::cerr << "Failed to create the video file." << std::endl;
        return;
    }

    for (const auto& frame_data : basic_video_data_int) {
        cv::Mat frame(y_height, x_width, CV_8UC3);
        for (int y = 0; y < y_height; ++y) {
            for (int x = 0; x < x_width; ++x) {
                int value = frame_data[y][x];
                frame.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
            }
        }
        writer.write(frame);
    }
    std::cout << "The video has been completed." << std::endl;
}

void basic_makevideo_double(std::vector<std::vector<std::vector<double>>> basic_video_data_double){
    auto video_data = normalizeto255(basic_video_data_double);
    basic_makevideo_int(video_data);
}
#pragma endregion

#pragma region VideoClass
VideoClass::VideoClass(std::vector<std::vector<std::vector<int>>> video_data)
    : video_data(video_data), filename("output_video.mp4"), fps(30.0), codec(cv::VideoWriter::fourcc('a', 'v', 'c', '1')),
      flag_scaleBar(false), bar_width(0), gap_width(0)
{
    t_size = video_data.size();
    y_size = video_data[0].size();
    x_size = video_data[0][0].size();
    cell_size = 10;
    update_xwidth_yheight();
}

void VideoClass::update_xwidth_yheight(){
    x_width = cell_size * x_size;
    y_height = cell_size * y_size;
}

cv::Mat VideoClass::create_frame(const std::vector<std::vector<int>>& frame_data) const {
    cv::Mat frame(y_height, x_width, CV_8UC3);
    for (int y = 0; y < y_size; ++y) {
        for (int x = 0; x < x_size; ++x) {
            int value = frame_data[y][x];
            cv::Rect cell(cell_size * x, cell_size * y, cell_size, cell_size);
            frame(cell) = cv::Scalar(value, value, value);
        }
    }
    return frame;
}

void VideoClass::initialize_scaleBar() {
    if (bar_width < 1) bar_width = static_cast<int>(x_width * 0.1);
    if (gap_width < 1) gap_width = static_cast<int>(x_width * 0.1);
}

cv::Mat VideoClass::add_scaleBar(const cv::Mat& frame) const {
    cv::Mat scale_bar(y_height, bar_width, CV_8UC3);
    cv::Mat gap(y_height, gap_width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int y = 0; y < y_height; ++y) {
        for (int x = 0; x < bar_width; ++x) {
            int value = 255 * (y_height - y) / y_height;
            scale_bar.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
        }
    }
    cv::Mat frame_with_scaleBar;
    cv::hconcat(std::vector<cv::Mat>{frame, gap, scale_bar}, frame_with_scaleBar);
    return frame_with_scaleBar;
}

void VideoClass::makevideo() const {
    int img_width = x_width + gap_width + bar_width;
    int img_height = y_height;
    cv::VideoWriter writer;
    writer.open(filename, codec, fps, cv::Size(img_width, img_height));

    if (!writer.isOpened()) {
        std::cerr << "Failed to create the video file." << std::endl;
        return;
    }

    for (const auto& frame_data : video_data) {
        cv::Mat frame = create_frame(frame_data);
        if (flag_scaleBar) frame = add_scaleBar(frame);
        writer.write(frame);
    }
    std::cout << "The video has been completed." << std::endl;
}

void VideoClass::set_filename(std::string filename) { this->filename = filename; }
void VideoClass::set_codec(int codec) { this->codec = codec; }
void VideoClass::set_fps(double fps) { this->fps = fps; }
void VideoClass::set_cellsize(int cell_size) { this->cell_size = cell_size; update_xwidth_yheight(); }
VideoClass& VideoClass::set_scaleBar(bool flag_scaleBar) { this->flag_scaleBar = flag_scaleBar; if (flag_scaleBar) initialize_scaleBar(); return *this; }
VideoClass& VideoClass::set_barwidth(int bar_width) { this->bar_width = bar_width; return *this; }
VideoClass& VideoClass::set_gapwidth(int gap_width) { this->gap_width = gap_width; return *this; }

void VideoClass::show_parameters() const {
    int img_width = x_width + bar_width + gap_width;
    int img_height = y_height;
    std::cout << "filename=" << filename << "\n";
    std::cout << "fps=" << fps << " codec=" << codec << "\n";
    std::cout << "t_size=" << t_size << " x_size=" << x_size << " y_size=" << y_size << "\n";
    std::cout << "cell_size=" << cell_size << " x_width=" << x_width << " y_height=" << y_height << "\n";
    std::cout << "img_width=" << img_width << " img_height=" << img_height << std::endl;
}

void VideoClass::show_size() const {
    std::cout << "t=" << t_size << " y=" << y_size << " x=" << x_size << ": t for frame, y for height, x for width" << std::endl;
}
#pragma endregion

} // namespace oyl
