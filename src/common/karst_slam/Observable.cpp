#include "karst_slam/Observable.h"

using namespace karst_slam;

void Observable::notify()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    for(auto& obs : m_observers)
        obs.first->updateFrom(this);
}
