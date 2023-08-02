#ifndef KARST_SLAM_INTERNAL_H
#define KARST_SLAM_INTERNAL_H

namespace karst_slam
{
    namespace internal
    {
        #define DLL_PUBLIC [[gnu::visibility("default")]]
        #define DLL_LOCAL [[gnu::visibility("hidden")]]
    }
}

#endif // KARST_SLAM_INTERNAL_H
