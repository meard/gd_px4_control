#ifndef GDCONTROL_CONTEXT_HPP
#define GDCONTROL_CONTEXT_HPP

#include <cmath>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace gd_video_cam
{

#define GDCONTROL_ALL_PARAMS                                                         \
    CXT_MACRO_MEMBER(file, bool, false) /* Read from file vs. read from device */         \
    CXT_MACRO_MEMBER(fps, int, 30)      /* Desired frames per second */                   \
                                                                                          \
    CXT_MACRO_MEMBER(filename, std::string, "") /* Filename */                            \
                                                                                          \
    CXT_MACRO_MEMBER(index, int, 0)  /* Device index, see cv::VideoCaptureAPIs */         \
    CXT_MACRO_MEMBER(width, int, 0)  /* Device width */                                   \
    CXT_MACRO_MEMBER(height, int, 0) /* Device height */                                  \
                                                                                          \
    CXT_MACRO_MEMBER(camera_info_path, std::string, "info.ini")    /* Camera info path */ \
    CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_frame") /* Camera frame id */  \
                                                                   /* End of list */

    struct GDCameraContext
    {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
        CXT_MACRO_DEFINE_MEMBERS(GDCONTROL_ALL_PARAMS)
    };

} // namespace gd_video_cam

#endif // GDCONTROL_CONTEXT_HPP